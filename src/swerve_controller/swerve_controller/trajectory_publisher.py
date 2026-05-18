#!/usr/bin/env python3
"""
Swerve Trajectory Publisher - Robot Padi
Mengirim waypoint trajectory ke controller dengan pola navigasi persawahan.
Mendukung preset trajektori untuk menghindari tanaman padi.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math
import time


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')

        # Parameter
        self.declare_parameter('field_length', 10.0)    # panjang petak sawah (m)
        self.declare_parameter('field_width', 5.0)      # lebar petak sawah (m)
        self.declare_parameter('row_spacing', 0.25)     # jarak antar baris tanaman (m)
        self.declare_parameter('trajectory_type', 'boustrophedon')  # pola lintasan

        self.field_length = self.get_parameter('field_length').value
        self.field_width = self.get_parameter('field_width').value
        self.row_spacing = self.get_parameter('row_spacing').value
        self.traj_type = self.get_parameter('trajectory_type').value

        # Publisher
        self.path_pub = self.create_publisher(Path, '/target_trajectory', 10)
        self.mode_pub = self.create_publisher(String, '/swerve_mode', 10)

        # Timer untuk publish sekali setelah node siap
        self.timer = self.create_timer(2.0, self.publish_trajectory)
        self.published = False

        self.get_logger().info(f"Trajectory Publisher siap. Tipe: {self.traj_type}")

    def publish_trajectory(self):
        if self.published:
            return
        self.published = True
        self.timer.cancel()

        # Set mode trajectory
        mode_msg = String()
        mode_msg.data = 'trajectory'
        self.mode_pub.publish(mode_msg)

        # Buat waypoints
        if self.traj_type == 'boustrophedon':
            waypoints = self._boustrophedon_path()
        elif self.traj_type == 'diagonal_45':
            waypoints = self._diagonal_45_path()
        elif self.traj_type == 'crab_shift':
            waypoints = self._crab_shift_path()
        else:
            waypoints = self._boustrophedon_path()

        # Bangun nav_msgs/Path
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for (x, y) in waypoints:
            ps = PoseStamped()
            ps.header.stamp = path_msg.header.stamp
            ps.header.frame_id = 'map'
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)

        self.path_pub.publish(path_msg)
        self.get_logger().info(
            f"Trajectory '{self.traj_type}' dipublish: {len(waypoints)} waypoints"
        )

    def _boustrophedon_path(self):
        """
        Pola boustrophedon (zig-zag) – cocok untuk panen/semprot di barisan tanaman.
        Robot bergerak maju di satu jalur, geser ke samping, balik arah.
        """
        waypoints = []
        n_rows = int(self.field_width / self.row_spacing)
        direction = 1  # 1 = maju, -1 = mundur

        for row in range(n_rows):
            y = row * self.row_spacing
            if direction == 1:
                waypoints.append((0.0, y))
                waypoints.append((self.field_length, y))
            else:
                waypoints.append((self.field_length, y))
                waypoints.append((0.0, y))
            direction *= -1

        return waypoints

    def _diagonal_45_path(self):
        """
        Lintasan diagonal 45°.
        Robot bergerak pada sudut 45° antar baris sehingga roda
        mendarati titik antara dua tanaman padi.
        Digunakan bersama mode SWERVE_45.
        """
        waypoints = []
        step = self.row_spacing * math.sqrt(2)  # proyeksi diagonal
        n_steps = int(self.field_length / step)

        for i in range(n_steps + 1):
            x = i * step
            y = i * step  # 45° diagonal
            if x <= self.field_length and y <= self.field_width:
                waypoints.append((x, y))

        # Kembali via jalur paralel
        for i in range(n_steps, -1, -1):
            x = i * step + self.row_spacing
            y = i * step
            if 0 <= x <= self.field_length and 0 <= y <= self.field_width:
                waypoints.append((x, y))

        return waypoints

    def _crab_shift_path(self):
        """
        Lintasan dengan pergeseran samping (crab walk 90°).
        Robot maju sepanjang satu baris, lalu geser ke samping
        tanpa berputar – badan robot selalu menghadap depan.
        Sangat berguna agar tidak menginjak tanaman saat ganti jalur.
        """
        waypoints = []
        n_rows = int(self.field_width / self.row_spacing)
        direction = 1

        x = 0.0
        for row in range(n_rows):
            y = row * self.row_spacing
            # Maju / mundur
            end_x = self.field_length if direction == 1 else 0.0
            waypoints.append((end_x, y))
            # Geser ke samping (crab walk)
            next_y = (row + 1) * self.row_spacing
            if row + 1 < n_rows:
                waypoints.append((end_x, next_y))
            direction *= -1

        return waypoints


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
