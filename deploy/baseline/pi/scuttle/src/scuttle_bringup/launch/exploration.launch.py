#!/usr/bin/env python3
"""
SCUTTLE Exploration Launch
Launch ON TOP of standby.launch.py in a new terminal.
Starts: SLAM -> (delay) Nav2 -> (delay) explore_lite + control manager
Kill this when exploration is complete.

REQUIRES RMW_IMPLEMENTATION=rmw_cyclonedds_cpp (set in ~/.bashrc).
The Jazzy default (Fast DDS) intermittently drops lifecycle change_state
service responses under load, which hangs Nav2's autostart bringup
(see ros-navigation/navigation2#3033). CycloneDDS does not have this bug,
so the stock autostart=true bringup below works reliably.
Every ROS terminal (standby + exploration) MUST use the same RMW.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, EmitEvent, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_nav          = get_package_share_directory('scuttle_nav')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    slam_config    = os.path.join(pkg_nav, 'config', 'mapper_params_online_async.yaml')
    nav2_params    = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    explore_params = os.path.join(pkg_nav, 'config', 'explore_params.yaml')

    # ---- SLAM Toolbox ----
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

    # ---- Nav2 — delayed 10s after SLAM starts ----
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params,
            'autostart': 'true'
        }.items()
    )

    nav2_delayed = TimerAction(
        period=10.0,
        actions=[nav2_launch]
    )

    # ---- explore_lite — delayed 30s after SLAM starts ----
    # Gives Nav2 ~20s to fully activate before explore starts sending goals.
    # (explore also retries connecting to the nav2 action server on its own.)
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[
            explore_params,
            {'use_sim_time': False}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    explore_delayed = TimerAction(
        period=30.0,
        actions=[explore_node]
    )

    # ---- Control manager — delayed 30s same as explore ----
    control_manager = Node(
        package='scuttle_base',
        executable='scuttle_control_manager',
        name='scuttle_control_manager',
        output='screen',
        parameters=[{
            'teleop_timeout': 0.5,
            'cancel_on_teleop': True,
        }]
    )

    control_manager_delayed = TimerAction(
        period=30.0,
        actions=[control_manager]
    )

    return LaunchDescription([
        # SLAM starts immediately
        slam_node,
        configure_event,
        activate_event,
        # Nav2 starts 10s later (autostart=true — reliable under CycloneDDS)
        nav2_delayed,
        # explore_lite + control manager start 30s later
        explore_delayed,
        control_manager_delayed,
    ])
