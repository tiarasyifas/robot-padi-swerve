<<<<<<< HEAD
# Robot Padi Swerve Drive — Panduan Lengkap

## ROS 2 Jazzy | Sliding Mode Control | Swerve Drive | Gazebo Harmonic

---

## Daftar Isi
1. [Arsitektur Sistem](#1-arsitektur-sistem)
2. [Konsep Swerve Drive & Sudut Steering](#2-konsep-swerve-drive--sudut-steering)
3. [Struktur Package](#3-struktur-package)
4. [Instalasi & Build](#4-instalasi--build)
5. [Menjalankan Simulasi Gazebo](#5-menjalankan-simulasi-gazebo)
6. [Mode Operasi](#6-mode-operasi)
7. [Sliding Mode Control (SMC)](#7-sliding-mode-control-smc)
8. [Hardware (Arduino/ESP)](#8-hardware-arduinoesp)
9. [Topik ROS 2](#9-topik-ros-2)
10. [Migrasi Humble → Jazzy](#10-migrasi-humble--jazzy)
11. [Troubleshooting](#11-troubleshooting)

---

## 1. Arsitektur Sistem

```
┌────────────────────────────────────────────────────────────────┐
│                    ROS 2 JAZZY (Ubuntu 24.04)                  │
│                                                                  │
│  ┌──────────────┐    /cmd_vel     ┌────────────────────────┐   │
│  │  Sistem Lama │───────────────► │  Swerve Drive          │   │
│  │ (SMC legacy) │                 │  Controller            │   │
│  └──────────────┘                 │                        │   │
│                                   │  Mode:                 │   │
│  ┌──────────────┐  /swerve_mode   │  • LEGACY (compat)     │   │
│  │  Mode Switch │───────────────► │  • SWERVE_45           │   │
│  └──────────────┘                 │  • SWERVE_90           │   │
│                                   │  • TRAJECTORY          │   │
│  ┌──────────────┐ /target_path    │                        │   │
│  │  Trajectory  │───────────────► │  SMC Controller        │   │
│  │  Publisher   │                 └──────────┬─────────────┘   │
│  └──────────────┘                            │                  │
│                              /wheel_speeds   │ /steering_angles │
│                                   ┌──────────▼──────────┐      │
│                                   │    Gazebo Harmonic  │      │
│                                   │    / Joint States   │      │
│                                   └─────────────────────┘      │
└────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────┐
│                    HARDWARE LAYER                               │
│  ┌──────────┐   Serial/   ┌──────────┐  PWM  ┌─────────────┐ │
│  │ ROS 2    │◄──micro-ROS─► ESP32 /  │──────►│ 4x Stepper  │ │
│  │ Host PC  │             │ ATmega   │       │ (steering)  │ │
│  └──────────┘             └──────────┘  PWM  ├─────────────┤ │
│                                         ─────►│ 4x Motor DC │ │
│                                               │ (drive)     │ │
│                                         ◄─────├─────────────┤ │
│                                         Enc   │ 4x Encoder  │ │
│                                               └─────────────┘ │
└────────────────────────────────────────────────────────────────┘
```

---

## 2. Konsep Swerve Drive & Sudut Steering

### Apa itu Swerve Drive?

Swerve drive adalah sistem penggerak dimana **setiap roda dapat berputar secara independen** (steering) sambil diputar oleh motor penggerak. Ini memberikan robot kemampuan **gerak holonomic** — bergerak ke segala arah tanpa perlu memutar badan robot.

### Mengapa 45° dan 90° untuk sawah?

```
Barisan Padi (jarak 0.25m):
●   ●   ●   ●   ●   ●
  ●   ●   ●   ●   ●
●   ●   ●   ●   ●   ●

Mode LEGACY (0°):         Mode SWERVE_45 (45°):    Mode SWERVE_90 (crab):
Robot maju lurus          Robot diagonal            Robot geser samping
→→→→→→→→→→             ↗↗↗↗↗↗↗↗↗             →→→→ (body tetap)
Roda menginjak barisan   Roda lewat antara 2 baris Ganti jalur tanpa putar
```

- **0° (Legacy)**: Gerak maju/mundur biasa. Kompatibel penuh dengan sistem lama.
- **45°**: Roda melewati titik antara 4 tanaman. Mengurangi risiko menginjak padi.
- **90°**: Gerak ke samping. Berguna untuk berpindah baris tanpa memutar badan robot sehingga tidak menyapu tanaman.

### Kinematika Swerve Drive

Untuk setiap modul roda pada posisi (xᵢ, yᵢ) dari pusat robot:

```
vx_roda_i = vx - ω × yᵢ
vy_roda_i = vy + ω × xᵢ

speed_roda_i  = √(vx_roda_i² + vy_roda_i²)
angle_roda_i  = atan2(vy_roda_i, vx_roda_i)
```

---

## 3. Struktur Package

```
swerve_robot_padi/
├── src/
│   ├── swerve_controller/          ← Package Python (ROS 2)
│   │   ├── swerve_controller/
│   │   │   ├── __init__.py
│   │   │   ├── swerve_drive_controller.py   ← NODE UTAMA
│   │   │   └── trajectory_publisher.py      ← Node publisher trajektori
│   │   ├── config/
│   │   │   ├── swerve_params.yaml           ← Parameter controller
│   │   │   └── ekf_params.yaml              ← Parameter EKF
│   │   ├── resource/swerve_controller
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   └── swerve_description/         ← Package CMake (URDF, worlds)
│       ├── urdf/
│       │   └── robot_padi_swerve.urdf.xacro ← Model robot
│       ├── launch/
│       │   └── gazebo_sim.launch.py         ← Launch simulasi
│       ├── worlds/
│       │   └── sawah.world                  ← World Gazebo
│       ├── rviz/
│       │   └── swerve_robot.rviz            ← Konfigurasi RViz
│       ├── CMakeLists.txt
│       └── package.xml
│
└── Micro/
    └── swerve_steering_controller/
        └── swerve_steering_controller.ino   ← Firmware Arduino/ESP
```

---

## 4. Instalasi & Build

### Prasyarat

```bash
# ROS 2 Jazzy (Ubuntu 24.04)
sudo apt update && sudo apt install -y \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-robot-localization \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-rviz2 \
  python3-numpy

# Gazebo Harmonic (sudah bundled dengan Jazzy)
gz sim --version  # pastikan versi 8.x
```

### Clone & Build

```bash
# Clone ke workspace yang sudah ada atau buat baru
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# Copy semua folder dari swerve_robot_padi/src/ ke sini
cp -r /path/to/swerve_robot_padi/src/* .

# Buat file __init__.py
touch src/swerve_controller/swerve_controller/__init__.py

# Build
cd ~/ros2_ws
colcon build --packages-select swerve_description swerve_controller
source install/setup.bash
```

---

## 5. Menjalankan Simulasi Gazebo

### Terminal 1 — Simulasi penuh (Gazebo + RViz + Controller)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch swerve_description gazebo_sim.launch.py
```

### Terminal 2 — Coba mode legacy (sistem lama tetap berjalan)

```bash
# Kirim cmd_vel biasa (sistem lama kompatibel)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Terminal 3 — Ganti ke mode Swerve 45°

```bash
ros2 topic pub --once /swerve_mode std_msgs/msg/String "{data: 'swerve_45'}"
# Kemudian kirim kecepatan
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Terminal 4 — Ganti ke mode Swerve 90° (crab walk)

```bash
ros2 topic pub --once /swerve_mode std_msgs/msg/String "{data: 'swerve_90'}"
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Terminal 5 — Jalankan trajectory otomatis

```bash
# Launch trajectory publisher (boustrophedon pattern)
ros2 run swerve_controller trajectory_publisher \
  --ros-args \
  -p trajectory_type:=boustrophedon \
  -p field_length:=10.0 \
  -p field_width:=5.0 \
  -p row_spacing:=0.25
```

### Monitor hasil

```bash
# Lihat sudut steering semua roda
ros2 topic echo /steering_angles

# Lihat kecepatan roda
ros2 topic echo /wheel_speeds

# Monitor odometry
ros2 topic echo /swerve_odom

# rqt_graph untuk melihat node graph
ros2 run rqt_graph rqt_graph
```

---

## 6. Mode Operasi

| Mode | Perintah | Sudut Steering | Kegunaan |
|------|----------|---------------|----------|
| `legacy` | `/swerve_mode: "legacy"` | Ikut cmd_vel (0°) | Kompatibel sistem lama, gerak maju/mundur/berputar |
| `swerve_45` | `/swerve_mode: "swerve_45"` | Tetap 45° | Gerak diagonal antar barisan padi |
| `swerve_90` | `/swerve_mode: "swerve_90"` | Tetap 90° | Geser samping tanpa memutar badan |
| `trajectory` | Otomatis saat Path diterima | Dinamis (SMC) | Ikuti waypoint otomatis |

---

## 7. Sliding Mode Control (SMC)

### Prinsip SMC untuk Trajectory Following

SMC dipilih karena **robust terhadap disturbance** (lumpur sawah, permukaan tidak rata):

```
Sliding Surface:
  s(t) = ė + λ·e

Kontrol Law (dengan saturasi untuk anti-chattering):
  u = -k · sat(s/η)

Dimana:
  e    = error posisi/sudut
  λ    = slope sliding surface (tunable, default: 3.0)
  k    = gain kontrol (linear: 1.2, angular: 2.5)
  η    = boundary layer thickness (default: 0.05)
  sat  = fungsi saturasi (bukan sign, agar tidak chattering)
```

### Tuning Parameter SMC

Edit `config/swerve_params.yaml`:

```yaml
smc_k_linear: 1.2     # Perbesar → lebih agresif kejar waypoint
smc_k_angular: 2.5    # Perbesar → koreksi sudut lebih cepat
smc_lambda: 3.0       # Perbesar → konvergensi lebih cepat (tapi bisa oscillasi)
smc_eta: 0.05         # Perbesar → anti-chattering lebih baik, tapi error steady-state naik
```

---

## 8. Hardware (Arduino/ESP)

### Wiring Stepper (NEMA 17 + A4988)

```
Arduino/ESP      A4988 Driver     NEMA 17 Stepper
─────────        ─────────────    ───────────────
Pin STEP ───────► STEP              (internal)
Pin DIR  ───────► DIR
GND      ───────► GND
5V       ───────► VDD
EN_PIN   ───────► ENABLE (aktif LOW)
                  VMOT ◄──── 12V/24V supply
                  GND(motor) ◄── GND supply
                  1A, 1B, 2A, 2B ──► Kumparan stepper
```

### Konfigurasi Microstepping (1/16)

Pada A4988: MS1=HIGH, MS2=HIGH, MS3=HIGH → 1/16 microstepping.
Ini memberikan 3200 step/rev → resolusi sudut ≈ 0.11°/step.

### Upload Firmware

1. Install library: `AccelStepper` via Arduino Library Manager
2. Untuk ROSserial: install `rosserial_arduino`
3. Buka `Micro/swerve_steering_controller/swerve_steering_controller.ino`
4. Pilih `#define USE_ROSSERIAL` atau `#define USE_MICRO_ROS`
5. Upload ke board

---

## 9. Topik ROS 2

| Topik | Tipe | Arah | Keterangan |
|-------|------|------|-----------|
| `/cmd_vel` | `Twist` | Sub | Input dari sistem lama/teleop |
| `/swerve_mode` | `String` | Sub | Ganti mode operasi |
| `/target_trajectory` | `Path` | Sub | Waypoints untuk trajectory mode |
| `/odom` | `Odometry` | Sub | Feedback posisi |
| `/wheel_speeds` | `Float32MultiArray` | Pub | Kecepatan 4 roda (rad/s) |
| `/steering_angles` | `Float32MultiArray` | Pub | Sudut 4 steering (rad) |
| `/joint_states` | `JointState` | Pub | Untuk RViz/Gazebo |
| `/swerve_odom` | `Odometry` | Pub | Odometry dari swerve |
| `/imu/data` | `Imu` | Sub | Warisan sistem lama |

---

## 10. Migrasi Humble → Jazzy

### Perubahan Utama

| Aspek | ROS 2 Humble | ROS 2 Jazzy |
|-------|-------------|-------------|
| Ubuntu | 22.04 | 24.04 |
| Gazebo | Classic / Ignition Fortress | Harmonic (gz sim) |
| DDS default | FastDDS | FastDDS (sama) |
| Python | 3.10 | 3.12 |
| Launch | `gazebo_ros` | `ros_gz_sim` |
| Bridge | `ros_ign_bridge` | `ros_gz_bridge` |
| Spawn | `spawn_entity.py` | `ros_gz_sim create` |

### Update package.xml

```xml
<!-- Lama (Humble) -->
<depend>gazebo_ros</depend>
<depend>gazebo_plugins</depend>

<!-- Baru (Jazzy) -->
<depend>ros_gz_sim</depend>
<depend>ros_gz_bridge</depend>
```

### Update launch file

```python
# Lama
from launch.actions import IncludeLaunchDescription
gz_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [FindPackageShare('gazebo_ros'), '/launch/gazebo.launch.py']))

# Baru (Jazzy)
from launch.actions import ExecuteProcess
gazebo = ExecuteProcess(
    cmd=['gz', 'sim', '-r', world_file], output='screen')
```

---

## 11. Troubleshooting

### Robot tidak bergerak di Gazebo

```bash
# Cek bridge berjalan
ros2 topic list | grep cmd_vel
# Cek joint states
ros2 topic echo /joint_states
```

### Stepper tidak berputar ke sudut yang benar

- Cek nilai `STEPS_PER_RAD` di firmware — sesuaikan dengan gear ratio aktual
- Cek apakah driver A4988/DRV8825 mendapat tegangan cukup (VMOT ≥ 12V)
- Pastikan ENABLE pin aktif (LOW untuk kebanyakan driver)

### SMC chattering (osilasi cepat pada output)

- Perbesar `smc_eta` (boundary layer): dari 0.05 → 0.1
- Kecilkan `smc_k_angular` atau `smc_k_linear`

### Error `package not found` saat colcon build

```bash
# Pastikan semua file __init__.py ada
touch src/swerve_controller/swerve_controller/__init__.py
# Pastikan resource file ada
mkdir -p src/swerve_controller/resource
touch src/swerve_controller/resource/swerve_controller
```

### RViz tidak menampilkan robot

- Cek bahwa `robot_state_publisher` berjalan: `ros2 node list`
- Set Fixed Frame di RViz ke `base_link` atau `odom`
- Add display: RobotModel, subscribe ke `/robot_description`
=======
# robot-padi-swerve
>>>>>>> 64c52c540bf598f823f8579b169c9a7d9df0c6bf
