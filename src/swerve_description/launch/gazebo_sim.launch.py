"""
Launch file: Simulasi Gazebo + RViz2 untuk Robot Padi Swerve Drive
ROS 2 Jazzy | Gazebo Harmonic

Menjalankan:
  1. Gazebo Harmonic (gz sim)
  2. Robot State Publisher (URDF)
  3. Joint State Publisher
  4. Swerve Drive Controller
  5. RViz2
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    ExecuteProcess, TimerAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Package paths ─────────────────────────────────────────────
    swerve_desc_pkg = get_package_share_directory('swerve_description')
    swerve_ctrl_pkg = get_package_share_directory('swerve_controller')

    # ── Argumen ───────────────────────────────────────────────────
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Jalankan RViz2')

    use_gazebo_arg = DeclareLaunchArgument(
        'use_gazebo', default_value='true',
        description='Jalankan Gazebo Harmonic')

    world_arg = DeclareLaunchArgument(
        'world', default_value='sawah.world',
        description='File world Gazebo')

    swerve_mode_arg = DeclareLaunchArgument(
        'swerve_mode', default_value='legacy',
        description='Mode awal: legacy|swerve_45|swerve_90|trajectory')

    # ── URDF via xacro ────────────────────────────────────────────
    urdf_file = os.path.join(swerve_desc_pkg, 'urdf', 'robot_padi_swerve.urdf.xacro')
    robot_description = Command(['xacro ', urdf_file])

    # ── Robot State Publisher ─────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # ── Joint State Publisher (untuk RViz tanpa Gazebo) ───────────
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
    )

    # ── Swerve Drive Controller ───────────────────────────────────
    swerve_controller = Node(
        package='swerve_controller',
        executable='swerve_drive_controller',
        name='swerve_drive_controller',
        parameters=[
            os.path.join(swerve_ctrl_pkg, 'config', 'swerve_params.yaml'),
            {'use_sim_time': True},
        ],
        output='screen',
        remappings=[
            ('/odom', '/swerve_odom'),
        ],
    )

    # ── Gazebo Harmonic ───────────────────────────────────────────
    world_file = os.path.join(swerve_desc_pkg, 'worlds', 'sawah.world')
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_gazebo')),
    )

    # Spawn robot ke Gazebo (dengan delay agar Gazebo siap)
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_robot',
                arguments=[
                    '-name', 'robot_padi',
                    '-topic', 'robot_description',
                    '-x', '0', '-y', '0', '-z', '0.15',
                ],
                output='screen',
            )
        ]
    )

    # Bridge Gazebo ↔ ROS 2
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_gazebo')),
    )

    # ── RViz2 ─────────────────────────────────────────────────────
    rviz_config = os.path.join(swerve_desc_pkg, 'rviz', 'swerve_robot.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    # ── EKF Odometry (opsional, warisan sistem lama) ──────────────
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[
            os.path.join(swerve_ctrl_pkg, 'config', 'ekf_params.yaml'),
            {'use_sim_time': True},
        ],
        remappings=[('odometry/filtered', '/odom')],
        output='screen',
    )

    return LaunchDescription([
        # Argumen
        use_rviz_arg,
        use_gazebo_arg,
        world_arg,
        swerve_mode_arg,
        # Nodes
        robot_state_publisher,
        joint_state_publisher,
        swerve_controller,
        gazebo,
        spawn_robot,
        gz_bridge,
        ekf_node,
        rviz,
    ])
