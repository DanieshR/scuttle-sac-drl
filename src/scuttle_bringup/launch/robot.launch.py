#!/usr/bin/env python3
"""
Unified launch file for SCUTTLE robot
Works with both simulation (Gazebo) and real hardware (Raspberry Pi)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_scuttle_gazebo = get_package_share_directory('scuttle_gazebo')
    pkg_scuttle_description = get_package_share_directory('scuttle_description')
    
    # Declare launch argument
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use simulation (Gazebo) or real hardware'
    )
    
    use_sim = LaunchConfiguration('use_sim')
    
    # Robot description (used by both sim and real)
    xacro_file = os.path.join(pkg_scuttle_description, 'urdf', 'scuttle.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    
    # Robot state publisher (always needed)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim
        }]
    )
    
    # Gazebo simulation (only if use_sim=true)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_scuttle_gazebo, 'launch', 'gazebo.launch.py')
        ),
        condition=IfCondition(use_sim)
    )
    
    # RPLidar driver (only if use_sim=false, for real hardware)
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'lidar_1',
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'use_sim_time': False
        }],
        condition=UnlessCondition(use_sim)
    )
    

    # Static transform for lidar frame (fixes Gazebo frame_id)
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_1', 'scuttle/base_link/rplidar_a1'],
        parameters=[{'use_sim_time': use_sim}]
    )

    return LaunchDescription([
        use_sim_arg,
        robot_state_publisher,
        gazebo_launch,
        rplidar_node,
        static_tf_lidar,
    ])
