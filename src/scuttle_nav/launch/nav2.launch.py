#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_nav = get_package_share_directory('scuttle_nav')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # Paths
    nav2_params_file = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_nav, 'maps', 'training.yaml')
    
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=map_file,
        description='Full path to map yaml file'
    )
    
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to Nav2 params file'
    )
    
    # Nav2 — navigation_launch.py works alongside live SLAM (no map server / AMCL needed)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_arg,
        params_arg,
        nav2_bringup,
    ])