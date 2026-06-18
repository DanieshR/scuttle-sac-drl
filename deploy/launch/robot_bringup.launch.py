#!/usr/bin/env python3
"""
robot_bringup.launch.py — the PI-SIDE half of the distributed offload (Phase 3).

DEPLOY TO AND RUN ON THE PI. Brings up everything that must stay local to the
robot and survive a flaky hotspot link: robot description, LiDAR, base/motor/odom,
the cmd_vel arbitration mux, and full Nav2. SLAM, frontier, and GDM do NOT run here
— they live in basestation_bringup.launch.py on the base station.

This reuses the robot's own, proven launch pieces (it is essentially
standby.launch.py + nav2.launch.py with the safety mux switched back on) rather
than reinventing them — only the composition changes for the offload.

  ros2 launch deploy/launch/robot_bringup.launch.py
  # (after sourcing zenoh-env.sh pi <base_ip> in the same shell — see deploy/config/zenoh/)

TF contract (must hold): scuttle_base publishes odom->base_link LOCALLY (publish_tf
True), so the motor loop never depends on the network. Only map->odom crosses the
hotspot, from SLAM on the base station.

‼️ REQUIRED COMPANION CHANGE — re-enabling twist_mux for the watchdog e-stop:
   The robot currently runs with twist_mux OFF and Nav2's collision_monitor writing
   straight to /cmd_vel. The offload needs twist_mux back so the Phase 5 watchdog can
   assert the /estop lock (priority 255). This launch turns twist_mux back on, so you
   MUST also point Nav2's collision_monitor at the muxed input, or the two will both
   drive /cmd_vel and fight:
       scuttle_nav/config/nav2_params.yaml  ->  collision_monitor:
           cmd_vel_out_topic: "cmd_vel"   ==>   cmd_vel_out_topic: "cmd_vel_safe"
   (twist_mux navigation input is /cmd_vel_safe @ pri 10; teleop /cmd_vel_teleop @ 100.)

‼️ RMW NOTE — Nav2 autostart reliability: the robot's exploration.launch.py documents
   that Fast DDS intermittently drops Nav2 lifecycle change_state responses and hangs
   autostart (navigation2#3033); it used CycloneDDS to avoid this. The offload uses
   rmw_zenoh — whether autostart is equally reliable under zenoh is UNVERIFIED and must
   be checked at Phase 4 bring-up. If autostart hangs, fall back per deploy/config/zenoh/.

The Phase 5 link_watchdog runs as a SEPARATE local daemon (see deploy/watchdog/), not
from this launch, so a robot_bringup restart never takes the safety node down with it.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_description  = get_package_share_directory('scuttle_description')
    pkg_ydlidar      = get_package_share_directory('ydlidar_ros2_driver')
    pkg_scuttle_base = get_package_share_directory('scuttle_base')
    pkg_scuttle_nav  = get_package_share_directory('scuttle_nav')

    xacro_file       = os.path.join(pkg_description, 'urdf', 'scuttle.xacro')
    twist_mux_config = os.path.join(pkg_scuttle_base, 'config', 'twist_mux.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Real hardware — keep false. Do NOT set true (no sim clock on the robot).'
    )

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]), value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        name='joint_state_publisher', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # LiDAR — MUST use TminiPlus.yaml. The default ydlidar.yaml (sample_rate 9,
    # intensity false) causes checksum errors -> std::out_of_range crash.
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ydlidar, 'launch', 'ydlidar_launch.py')),
        launch_arguments={'params_file': os.path.join(pkg_ydlidar, 'params', 'TminiPlus.yaml')}.items(),
    )

    # Base / motor / odom — publishes odom->base_link LOCALLY (publish_tf True).
    base_node = Node(
        package='scuttle_base', executable='scuttle_base_node', name='scuttle_base',
        output='screen',
        parameters=[{
            'port': '/dev/ttyESP32', 'baud': 115200,
            'wheel_radius': 0.04175, 'wheel_separation': 0.4247, 'counts_per_rev': 16384,
            'max_speed': 0.4, 'max_pwm': 255.0,
            'publish_tf': True,
        }],
    )

    # cmd_vel arbitration — RE-ENABLED for the offload (see header). /estop lock (pri 255)
    # is what the Phase 5 watchdog asserts. Output remapped to /cmd_vel (scuttle_base input).
    twist_mux = Node(
        package='twist_mux', executable='twist_mux', name='twist_mux', output='screen',
        parameters=[twist_mux_config, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out', '/cmd_vel')],
    )

    # Full Nav2 (RegulatedPurePursuitController) — consumes remote /map + map->odom.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_scuttle_nav, 'launch', 'nav2.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher,
        joint_state_publisher,
        lidar_launch,
        base_node,
        twist_mux,
        nav2_launch,
    ])
