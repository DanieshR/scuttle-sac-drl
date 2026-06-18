#!/usr/bin/env python3
"""
SCUTTLE Standby Launch
Starts: robot description + LiDAR + base node + twist_mux + foxglove
Use Foxglove joystick to drive to start point then launch exploration.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_description  = get_package_share_directory('scuttle_description')
    pkg_ydlidar      = get_package_share_directory('ydlidar_ros2_driver')
    pkg_scuttle_base = get_package_share_directory('scuttle_base')

    xacro_file       = os.path.join(pkg_description, 'urdf', 'scuttle.xacro')
    twist_mux_config = os.path.join(pkg_scuttle_base, 'config', 'twist_mux.yaml')

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ydlidar, 'launch', 'ydlidar_launch.py')
        ),
        launch_arguments={
            'params_file': os.path.join(pkg_ydlidar, 'params', 'TminiPlus.yaml')
        }.items()
    )

    base_node = Node(
        package='scuttle_base',
        executable='scuttle_base_node',
        name='scuttle_base',
        output='screen',
        parameters=[{
            'port': '/dev/ttyESP32',
            'baud': 115200,
            'wheel_radius': 0.04175,
            'wheel_separation': 0.4247,
            'counts_per_rev': 16384,
            'max_speed': 0.4,
            'max_pwm': 255.0,
            'publish_tf': True,
        }]
    )

    # twist_mux removed for now — Nav2's collision_monitor publishes directly to
    # /cmd_vel (stock Nav2 path). Re-add this node (and set collision_monitor
    # cmd_vel_out_topic back to "cmd_vel_safe") when layering teleop priority back in.
    # twist_mux = Node(
    #     package='twist_mux',
    #     executable='twist_mux',
    #     name='twist_mux',
    #     output='screen',
    #     parameters=[twist_mux_config],
    #     remappings=[('/cmd_vel_out', '/cmd_vel')]
    # )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{'port': 8765}]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        lidar_launch,
        base_node,
        # twist_mux,   # removed for now — see note above
        #foxglove_bridge,
    ])