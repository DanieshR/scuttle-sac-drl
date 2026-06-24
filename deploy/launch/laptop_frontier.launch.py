#!/usr/bin/env python3
"""
laptop_frontier.launch.py — spawned by laptop_mission_supervisor on frontier mode.

Just explore_lite (real clock). GDM lives in laptop_base now; Nav2 is on the Pi. The
supervisor gates this on /mission/mode, and explore_lite itself only acts once Nav2's
action server is up, so no extra gate is needed here.
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_nav = get_package_share_directory('scuttle_nav')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'frontier.launch.py')),
            launch_arguments={'use_sim_time': 'false'}.items(),
        ),
    ])
