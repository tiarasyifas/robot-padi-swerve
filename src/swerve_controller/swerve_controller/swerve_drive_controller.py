#!/usr/bin/env python3
"""
Swerve Drive Controller untuk Robot Padi
ROS 2 Jazzy | Sliding Mode Control + Swerve Kinematics
Kompatibel dengan sistem lama (differential drive / cmd_vel biasa)

Topik:
  Subscribe: /cmd_vel (Twist) - dari sistem lama
             /swerve_mode (String) - mode: 'legacy', 'swerve_45', 'swerve_90', 'trajectory'
             /target_trajectory (Path) - waypoint path untuk trajectory mode
  Publish:   /wheel_speeds (Float32MultiArray) - kecepatan 4 motor drive
             /steering_angles (Float32MultiArray) - sudut 4 servo steering
             /swerve_odom (Odometry)

Sudut Steering yang Didukung:
  - 0°   : Maju/mundur biasa (legacy)
  - 45°  : Diagonal / menghindari tanaman padi jalur miring
  - 90°  : Gerak samping (crab walk)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import JointState
import numpy as np
import math
from enum import Enum
from dataclasses import dataclass
from typing import List, Tuple


class SwerveMode(Enum):
    LEGACY = "legacy"         # Sistem lama: gerak maju/mundur/rotasi
    SWERVE_45 = "swerve_45"   # Steering 45 derajat
    SWERVE_90 = "swerve_90"   # Steering 90 derajat (crab walk)
    TRAJECTORY = "trajectory"  # Ikut trajectory dengan SMC


@dataclass
class WheelModule:
    """Representasi satu modul roda swerve"""
    name: str
    x: float  # posisi x dari pusat robot (meter)
    y: float  # posisi y dari pusat robot (meter)
    drive_speed: float = 0.0   # rad/s
    steer_angle: float = 0.0   # radian


class SwerveDriveController(Node):
    def __init__(self):
        super().__init__('swerve_drive_controller')

        self.get_logger().info("=" * 55)
        self.get_logger().info("  Swerve Drive Controller - Robot Padi")
        self.get_logger().info("  ROS 2 Jazzy | SMC + Swerve Kinematics")
        self.get_logger().info("=" * 55)

        # ── Parameter Robot Fisik ──────────────────────────────────
        self.declare_parameter('wheel_base', 0.40)       # jarak depan-belakang (m)
        self.declare_parameter('track_width', 0.35)      # jarak kiri-kanan (m)
        self.declare_parameter('wheel_radius', 0.075)    # jari-jari roda (m)
        self.declare_parameter('max_drive_speed', 2.0)   # rad/s max motor
        self.declare_parameter('max_steer_rate', 1.5)    # rad/s laju putar steering
        self.declare_parameter('gear_ratio', 20.0)       # rasio gearbox motor

        # ── Parameter SMC ─────────────────────────────────────────
        self.declare_parameter('smc_k_linear', 1.2)
        self.declare_parameter('smc_k_angular', 2.5)
        self.declare_parameter('smc_lambda', 3.0)
        self.declare_parameter('smc_eta', 0.05)          # boundary layer
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('waypoint_tolerance', 0.30)

        # ── Parameter Steering Presets ─────────────────────────────
        self.declare_parameter('steering_45_deg', 45.0)
        self.declare_parameter('steering_90_deg', 90.0)

        # Ambil parameter
        self._load_parameters()

        # ── Inisialisasi Modul Roda (FL, FR, RL, RR) ──────────────
        half_wb = self.wheel_base / 2.0
        half_tw = self.track_width / 2.0
        self.modules: List[WheelModule] = [
            WheelModule("front_left",   half_wb,  half_tw),
            WheelModule("front_right",  half_wb, -half_tw),
            WheelModule("rear_left",   -half_wb,  half_tw),
            WheelModule("rear_right",  -half_wb, -half_tw),
        ]

        # ── State Robot ───────────────────────────────────────────
        self.current_mode = SwerveMode.LEGACY
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_omega = 0.0
        self.last_time = self.get_clock().now()

        # ── SMC Trajectory Variables ──────────────────────────────
        self.waypoints: List[Tuple[float, float]] = []
        self.current_wp_index = 0
        self.trajectory_done = True
        self.prev_error_linear = 0.0
        self.prev_error_angular = 0.0

        # ── Subscriber ────────────────────────────────────────────
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.mode_sub = self.create_subscription(
            String, '/swerve_mode', self.mode_callback, 10)

        self.trajectory_sub = self.create_subscription(
            Path, '/target_trajectory', self.trajectory_callback, 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # ── Publisher ─────────────────────────────────────────────
        self.wheel_speeds_pub = self.create_publisher(
            Float32MultiArray, '/wheel_speeds', 10)

        self.steering_angles_pub = self.create_publisher(
            Float32MultiArray, '/steering_angles', 10)

        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10)

        self.swerve_odom_pub = self.create_publisher(
            Odometry, '/swerve_odom', 10)

        self.cmd_vel_swerve_pub = self.create_publisher(
            Twist, '/cmd_vel_swerve', 10)

        # ── Timer Kontrol ─────────────────────────────────────────
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info("Controller siap. Mode default: LEGACY")
        self.get_logger().info("Kirim ke /swerve_mode: 'legacy'|'swerve_45'|'swerve_90'|'trajectory'")

    def _load_parameters(self):
        self.wheel_base = self.get_parameter('wheel_base').value
        self.track_width = self.get_parameter('track_width').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_drive_speed = self.get_parameter('max_drive_speed').value
        self.max_steer_rate = self.get_parameter('max_steer_rate').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.smc_k_linear = self.get_parameter('smc_k_linear').value
        self.smc_k_angular = self.get_parameter('smc_k_angular').value
        self.smc_lambda = self.get_parameter('smc_lambda').value
        self.smc_eta = self.get_parameter('smc_eta').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.cmd_vel_twist = Twist()

    # ═══════════════════════════════════════════════════════════════
    # CALLBACKS
    # ═══════════════════════════════════════════════════════════════

    def cmd_vel_callback(self, msg: Twist):
        """Terima perintah dari sistem lama. Di mode LEGACY langsung dieksekusi."""
        self.cmd_vel_twist = msg
        if self.current_mode == SwerveMode.LEGACY:
            self._compute_swerve_from_twist(msg.linear.x, msg.linear.y, msg.angular.z)

    def mode_callback(self, msg: String):
        """Ganti mode operasi robot"""
        mode_str = msg.data.lower().strip()
        mode_map = {
            'legacy': SwerveMode.LEGACY,
            'swerve_45': SwerveMode.SWERVE_45,
            'swerve_90': SwerveMode.SWERVE_90,
            'trajectory': SwerveMode.TRAJECTORY,
        }
        if mode_str in mode_map:
            self.current_mode = mode_map[mode_str]
            self.get_logger().info(f"[MODE] Berganti ke: {self.current_mode.value.upper()}")

            # Preset sudut steering langsung saat mode berganti
            if self.current_mode == SwerveMode.SWERVE_45:
                angle = math.radians(45.0)
                for m in self.modules:
                    m.steer_angle = angle
                self._publish_steering()
                self.get_logger().info("Steering diset ke 45°. Robot siap bergerak diagonal.")

            elif self.current_mode == SwerveMode.SWERVE_90:
                angle = math.radians(90.0)
                for m in self.modules:
                    m.steer_angle = angle
                self._publish_steering()
                self.get_logger().info("Steering diset ke 90°. Robot siap crab walk.")
        else:
            self.get_logger().warn(f"Mode tidak dikenal: {mode_str}")

    def trajectory_callback(self, msg: Path):
        """Terima waypoints dari Nav2 atau publisher kustom"""
        self.waypoints = []
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            self.waypoints.append((x, y))
        self.current_wp_index = 0
        self.trajectory_done = (len(self.waypoints) == 0)
        self.current_mode = SwerveMode.TRAJECTORY
        self.get_logger().info(f"Trajectory diterima: {len(self.waypoints)} waypoints")

    def odom_callback(self, msg: Odometry):
        """Update posisi dari odometry"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)
        self.current_vx = msg.twist.twist.linear.x
        self.current_vy = msg.twist.twist.linear.y
        self.current_omega = msg.twist.twist.angular.z

    # ═══════════════════════════════════════════════════════════════
    # LOOP KONTROL UTAMA
    # ═══════════════════════════════════════════════════════════════

    def control_loop(self):
        """Loop kontrol 20 Hz"""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0 or dt > 0.5:
            return

        if self.current_mode == SwerveMode.LEGACY:
            # Teruskan langsung dari cmd_vel
            t = self.cmd_vel_twist
            self._compute_swerve_from_twist(t.linear.x, t.linear.y, t.angular.z)

        elif self.current_mode == SwerveMode.SWERVE_45:
            # Gerak dengan sudut steering 45°, linear.x dari cmd_vel
            t = self.cmd_vel_twist
            self._compute_swerve_45(t.linear.x)

        elif self.current_mode == SwerveMode.SWERVE_90:
            # Gerak crab walk 90°, linear.y dari cmd_vel
            t = self.cmd_vel_twist
            self._compute_swerve_90(t.linear.x)

        elif self.current_mode == SwerveMode.TRAJECTORY:
            if not self.trajectory_done:
                self._smc_trajectory_control(dt)

        self._publish_joint_states()

    # ═══════════════════════════════════════════════════════════════
    # KINEMATIKA SWERVE DRIVE
    # ═══════════════════════════════════════════════════════════════

    def _compute_swerve_from_twist(self, vx: float, vy: float, omega: float):
        """
        Hitung kecepatan dan sudut tiap roda dari input (vx, vy, omega).
        Ini adalah kinematika swerve drive standar.
        
        Untuk sistem lama: vy=0, omega=angular.z (differential style)
        Untuk swerve penuh: semua 3 DOF digunakan
        """
        for module in self.modules:
            # Kecepatan tiap roda = kecepatan translasi + kontribusi rotasi
            vx_wheel = vx - omega * module.y
            vy_wheel = vy + omega * module.x

            speed = math.sqrt(vx_wheel**2 + vy_wheel**2)
            angle = math.atan2(vy_wheel, vx_wheel) if speed > 0.001 else module.steer_angle

            # Optimasi: jika delta sudut > 90°, balik arah dan kurangi putaran
            delta = self._normalize_angle(angle - module.steer_angle)
            if abs(delta) > math.pi / 2:
                angle = self._normalize_angle(angle + math.pi)
                speed = -speed
                delta = self._normalize_angle(angle - module.steer_angle)

            # Rate limit steering
            max_delta = self.max_steer_rate * 0.05  # dt = 50ms
            delta = np.clip(delta, -max_delta, max_delta)
            module.steer_angle += delta

            # Konversi kecepatan linear -> rad/s
            module.drive_speed = np.clip(
                speed / self.wheel_radius,
                -self.max_drive_speed,
                self.max_drive_speed
            )

        self._publish_wheel_commands()

    def _compute_swerve_45(self, forward_speed: float):
        """
        Mode 45°: semua roda diarahkan 45 derajat dari sumbu maju.
        Robot bergerak diagonal, melewati antar-barisan tanaman padi.
        """
        target_angle = math.radians(45.0)
        wheel_speed = (forward_speed / self.wheel_radius) * math.sqrt(2)  # kompensasi diagonal

        for module in self.modules:
            # Gerakkan steering ke 45° secara bertahap
            delta = self._normalize_angle(target_angle - module.steer_angle)
            max_delta = self.max_steer_rate * 0.05
            module.steer_angle += np.clip(delta, -max_delta, max_delta)
            module.drive_speed = np.clip(wheel_speed, -self.max_drive_speed, self.max_drive_speed)

        self._publish_wheel_commands()

    def _compute_swerve_90(self, lateral_speed: float):
        """
        Mode 90°: semua roda diarahkan 90 derajat (tegak lurus badan robot).
        Robot bergerak ke samping seperti kepiting (crab walk).
        Berguna untuk berpindah antar-jalur tanpa memutar badan robot.
        """
        target_angle = math.radians(90.0)
        wheel_speed = lateral_speed / self.wheel_radius

        for module in self.modules:
            delta = self._normalize_angle(target_angle - module.steer_angle)
            max_delta = self.max_steer_rate * 0.05
            module.steer_angle += np.clip(delta, -max_delta, max_delta)
            module.drive_speed = np.clip(wheel_speed, -self.max_drive_speed, self.max_drive_speed)

        self._publish_wheel_commands()

    # ═══════════════════════════════════════════════════════════════
    # SLIDING MODE CONTROL UNTUK TRAJECTORY FOLLOWING
    # ═══════════════════════════════════════════════════════════════

    def _smc_trajectory_control(self, dt: float):
        """
        SMC untuk mengikuti waypoint trajectory.
        Menghasilkan (vx, vy, omega) lalu diteruskan ke kinematika swerve.
        """
        if self.current_wp_index >= len(self.waypoints):
            self._stop_robot()
            self.trajectory_done = True
            self.get_logger().info("✓ Trajectory selesai!")
            return

        target_x, target_y = self.waypoints[self.current_wp_index]

        # ── Error ─────────────────────────────────────────────────
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        dist_error = math.sqrt(dx**2 + dy**2)
        angle_to_target = math.atan2(dy, dx)
        heading_error = self._normalize_angle(angle_to_target - self.current_yaw)

        # ── Cek waypoint tercapai ──────────────────────────────────
        if dist_error < self.waypoint_tolerance:
            self.current_wp_index += 1
            self.prev_error_linear = 0.0
            self.prev_error_angular = 0.0
            self.get_logger().info(
                f"WP {self.current_wp_index-1}/{len(self.waypoints)} tercapai! "
                f"Sisa: {len(self.waypoints) - self.current_wp_index}"
            )
            return

        # ── Sliding Surface ────────────────────────────────────────
        # Linear
        error_linear_dot = (dist_error - self.prev_error_linear) / dt
        s_linear = error_linear_dot + self.smc_lambda * dist_error

        # Angular
        error_angular_dot = (heading_error - self.prev_error_angular) / dt
        s_angular = error_angular_dot + self.smc_lambda * heading_error

        self.prev_error_linear = dist_error
        self.prev_error_angular = heading_error

        # ── SMC Law: u = -k * sat(s/eta) ──────────────────────────
        # Gunakan sat (saturasi) bukan sign untuk mengurangi chattering
        vx_cmd = self.smc_k_linear * self._sat(s_linear, self.smc_eta)
        omega_cmd = self.smc_k_angular * self._sat(s_angular, self.smc_eta)

        # Kurangi kecepatan linear saat error sudut besar
        vx_cmd *= math.exp(-1.5 * abs(heading_error))

        # Clamp
        vx_cmd = np.clip(vx_cmd, -self.max_linear_speed, self.max_linear_speed)
        omega_cmd = np.clip(omega_cmd, -self.max_angular_speed, self.max_angular_speed)

        # Dalam mode trajectory, gunakan swerve kinematik penuh
        self._compute_swerve_from_twist(vx_cmd, 0.0, omega_cmd)

        if self.current_wp_index % 5 == 0:
            self.get_logger().info(
                f"[SMC] WP {self.current_wp_index} | "
                f"Dist:{dist_error:.2f}m | "
                f"HeadErr:{math.degrees(heading_error):.1f}° | "
                f"vx:{vx_cmd:.2f} ω:{omega_cmd:.2f}"
            )

    # ═══════════════════════════════════════════════════════════════
    # PUBLISHER
    # ═══════════════════════════════════════════════════════════════

    def _publish_wheel_commands(self):
        """Publish kecepatan roda dan sudut steering"""
        speeds_msg = Float32MultiArray()
        angles_msg = Float32MultiArray()

        speeds_msg.data = [float(m.drive_speed) for m in self.modules]
        angles_msg.data = [float(m.steer_angle) for m in self.modules]

        self.wheel_speeds_pub.publish(speeds_msg)
        self.steering_angles_pub.publish(angles_msg)
        self._publish_steering()

    def _publish_steering(self):
        angles_msg = Float32MultiArray()
        angles_msg.data = [float(m.steer_angle) for m in self.modules]
        self.steering_angles_pub.publish(angles_msg)

    def _publish_joint_states(self):
        """Publish joint states untuk RViz dan Gazebo"""
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = []
        js.position = []
        js.velocity = []

        for m in self.modules:
            # Steering joint
            js.name.append(f"{m.name}_steering_joint")
            js.position.append(float(m.steer_angle))
            js.velocity.append(0.0)
            # Drive joint
            js.name.append(f"{m.name}_wheel_joint")
            js.position.append(0.0)
            js.velocity.append(float(m.drive_speed))

        self.joint_state_pub.publish(js)

    def _stop_robot(self):
        for m in self.modules:
            m.drive_speed = 0.0
        self._publish_wheel_commands()

    # ═══════════════════════════════════════════════════════════════
    # UTILITY
    # ═══════════════════════════════════════════════════════════════

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    @staticmethod
    def _sat(s: float, eta: float) -> float:
        """Fungsi saturasi untuk SMC (mengurangi chattering)"""
        if abs(s) <= eta:
            return s / eta
        return math.copysign(1.0, s)

    @staticmethod
    def _quat_to_yaw(x, y, z, w) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = SwerveDriveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Controller dihentikan.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
