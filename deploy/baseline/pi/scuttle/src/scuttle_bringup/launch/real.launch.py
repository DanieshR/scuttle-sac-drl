#!/usr/bin/env python3
"""Launch SCUTTLE on real hardware (Raspberry Pi)"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('scuttle_bringup')
    
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'robot.launch.py')
        ),
        launch_arguments={'use_sim': 'false'}.items()
    )
    
    return LaunchDescription([robot_launch])
