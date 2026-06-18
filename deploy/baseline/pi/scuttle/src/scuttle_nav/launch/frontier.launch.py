#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_nav = get_package_share_directory('scuttle_nav')
    params_file = os.path.join(pkg_nav, 'config', 'explore_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true'
    )

    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    return LaunchDescription([
        use_sim_time_arg,
        explore_node,
    ])
