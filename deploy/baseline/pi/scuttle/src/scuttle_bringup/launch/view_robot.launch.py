#!/usr/bin/env python3
"""Launch RViz to visualize SCUTTLE robot"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_description = get_package_share_directory('scuttle_description')
    rviz_config = os.path.join(pkg_description, 'rviz', 'scuttle.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else []
    )
    
    return LaunchDescription([rviz_node])
