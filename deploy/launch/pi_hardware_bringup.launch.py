#!/usr/bin/env python3
"""
pi_hardware_bringup.launch.py — DEVICE 1 (Raspberry Pi), STAGE 1 of the gated sequence.

RUN ON THE PI. Launches IMMEDIATELY — no gating. Brings up everything hardware-local
that must survive a flaky Zenoh hotspot link:

  * robot_state_publisher (Scuttle v3 description / TF tree)
  * LiDAR driver (YDLidar Tmini Plus — TminiPlus.yaml, NOT the default yaml)
  * micro-ROS agent for the Maker Feather AIoT S3 (gas sensor node over micro-ROS)
  * scuttle_base: physical wheel-encoder odometry + base/motor control
    (publishes odom->base_link LOCALLY so the motor loop never waits on the network)
  * twist_mux: cmd_vel arbitration so the watchdog can assert /estop (priority 255)

Nav2 is deliberately NOT here — it is gated on /map in pi_nav2_delayed.launch.py.

  source deploy/scripts/zenoh-env.sh pi <base_ip>     # SAME shell, before launching
  ros2 launch deploy/launch/pi_hardware_bringup.launch.py

TF contract: odom->base_link is published locally (publish_tf:=True). Only map->odom
crosses Zenoh, from SLAM on the laptop.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_description  = get_package_share_directory('scuttle_description')
    pkg_ydlidar      = get_package_share_directory('ydlidar_ros2_driver')
    pkg_scuttle_base = get_package_share_directory('scuttle_base')

    xacro_file       = os.path.join(pkg_description, 'urdf', 'scuttle.xacro')
    twist_mux_config = os.path.join(pkg_scuttle_base, 'config', 'twist_mux.yaml')

    # ---- launch args ----
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_micro_ros = LaunchConfiguration('enable_micro_ros')        # Phase-2 gas sensor
    micro_ros_transport = LaunchConfiguration('micro_ros_transport')  # 'serial' | 'udp4'
    micro_ros_dev = LaunchConfiguration('micro_ros_dev')              # serial device
    micro_ros_port = LaunchConfiguration('micro_ros_port')            # udp4 port

    args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Real hardware — keep false. There is no sim clock on the robot.'),
        DeclareLaunchArgument(
            'enable_micro_ros', default_value='false',
            description='Start the micro-ROS agent for the Maker Feather AIoT S3 gas '
                        'sensor (Phase 2). OFF by default: the agent package and the '
                        'gas board are optional and need not be present to bring up the '
                        'core robot. Set true once micro_ros_agent is installed and the '
                        'AIoT S3 is connected.'),
        DeclareLaunchArgument(
            'micro_ros_transport', default_value='serial',
            description="micro-ROS agent transport for the Maker Feather AIoT S3: "
                        "'serial' (USB-CDC) or 'udp4' (WiFi)."),
        DeclareLaunchArgument(
            'micro_ros_dev', default_value='/dev/ttyAIoT',
            description='Serial device for the AIoT S3 when transport=serial.'),
        DeclareLaunchArgument(
            'micro_ros_port', default_value='8888',
            description='UDP port for the AIoT S3 when transport=udp4.'),
    ]

    # ---- robot description / TF ----
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

    # ---- LiDAR — MUST use TminiPlus.yaml (default yaml -> checksum crash) ----
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ydlidar, 'launch', 'ydlidar_launch.py')),
        launch_arguments={
            'params_file': os.path.join(pkg_ydlidar, 'params', 'TminiPlus.yaml')
        }.items(),
    )

    # ---- micro-ROS agent for the Maker Feather AIoT S3 (gas sensor node) ----
    # Serial transport: micro_ros_agent serial --dev /dev/ttyAIoT -b 115200
    micro_ros_serial = Node(
        package='micro_ros_agent', executable='micro_ros_agent',
        name='micro_ros_agent', output='screen',
        arguments=['serial', '--dev', micro_ros_dev, '-b', '115200'],
        condition=IfCondition(
            PythonExpression(["'", enable_micro_ros, "' == 'true' and '",
                              micro_ros_transport, "' == 'serial'"])),
    )
    # WiFi/UDP transport: micro_ros_agent udp4 --port 8888
    micro_ros_udp = Node(
        package='micro_ros_agent', executable='micro_ros_agent',
        name='micro_ros_agent', output='screen',
        arguments=['udp4', '--port', micro_ros_port],
        condition=IfCondition(
            PythonExpression(["'", enable_micro_ros, "' == 'true' and '",
                              micro_ros_transport, "' != 'serial'"])),
    )

    # ---- base / motor / wheel-encoder odometry (publishes odom->base_link locally) ----
    base_node = Node(
        package='scuttle_base', executable='scuttle_base_node', name='scuttle_base',
        output='screen',
        parameters=[{
            'port': '/dev/ttyESP32', 'baud': 115200,
            'wheel_radius': 0.04175, 'wheel_separation': 0.4247, 'counts_per_rev': 16384,
            'max_speed': 0.4, 'max_pwm': 255.0,
            'publish_tf': True,            # encoder odom -> base_link, LOCAL
            'use_sim_time': use_sim_time,
        }],
    )

    # ---- cmd_vel arbitration (so the link watchdog can assert /estop @ pri 255) ----
    twist_mux = Node(
        package='twist_mux', executable='twist_mux', name='twist_mux', output='screen',
        parameters=[twist_mux_config, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out', '/cmd_vel')],
    )

    return LaunchDescription(args + [
        robot_state_publisher,
        joint_state_publisher,
        lidar_launch,
        micro_ros_serial,
        micro_ros_udp,
        base_node,
        twist_mux,
    ])
