#!/usr/bin/env python3
"""
pi_base.launch.py — the Pi's always-on base. Auto-started by scuttle-base.service.

Hardware bringup (sensors/odom/gas/twist_mux) + the mission supervisor that brings up
pi_nav2_delayed.launch.py when the dashboard requests frontier mode.
"""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

_HERE = os.path.dirname(os.path.abspath(__file__))
_DEPLOY = os.path.dirname(_HERE)


def generate_launch_description():
    supervisor = os.path.join(_DEPLOY, 'nodes', 'mission_supervisor.py')
    nav2_launch = os.path.join(_HERE, 'pi_nav2_delayed.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(_HERE, 'pi_hardware_bringup.launch.py'))),
        ExecuteProcess(
            cmd=['python3', supervisor,
                 '--role', 'pi',
                 '--launch-file', nav2_launch,
                 '--state-topic', '/mission/state/pi'],
            name='mission_supervisor_pi', output='screen'),
    ])
