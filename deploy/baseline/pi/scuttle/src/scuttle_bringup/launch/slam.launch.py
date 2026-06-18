#!/usr/bin/env python3
"""
SCUTTLE SLAM Launch — Layer 2
Launch on top of standby when you want to map.
Kill when map is saved.
"""

import os
from launch import LaunchDescription
from launch.actions import EmitEvent
from launch_ros.actions import LifecycleNode
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_nav = get_package_share_directory('scuttle_nav')
    slam_config = os.path.join(pkg_nav, 'config', 'mapper_params_online_async.yaml')

    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': False}
        ],
        remappings=[('/scan', '/scan')]
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    activate_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_ACTIVATE
        )
    )

    return LaunchDescription([
        slam_node,
        configure_event,
        activate_event,
    ])