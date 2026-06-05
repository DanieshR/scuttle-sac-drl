#!/usr/bin/env python3
"""
SCUTTLE turtlebot3_drlnav Training Launch — Gazebo Harmonic
Supports stages 1-9 via the 'stage' argument.

Usage (4 terminals):
  T1: ros2 launch scuttle_bringup drl_tomasvr.launch.py stage:=1
  T2: ros2 run turtlebot3_drl gazebo_goals
  T3: ros2 run turtlebot3_drl environment
  T4: ros2 run turtlebot3_drl train_agent sac
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    OpaqueFunction,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    stage = int(LaunchConfiguration('stage').perform(context))

    pkg_desc   = get_package_share_directory('scuttle_description')
    pkg_gazebo = get_package_share_directory('scuttle_gazebo')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')

    # Write stage file so all training nodes pick it up consistently
    with open('/tmp/drlnav_current_stage.txt', 'w') as f:
        f.write(f'{stage}\n')
    print(f'[drl_tomasvr] Stage {stage} — writing /tmp/drlnav_current_stage.txt')

    xacro_file = os.path.join(pkg_desc, 'urdf', 'scuttle.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    world_file = os.path.join(pkg_gazebo, 'worlds', f'drl_stage{stage}.sdf')
    if not os.path.exists(world_file):
        raise FileNotFoundError(f'World SDF not found: {world_file}')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'{world_file} -r --headless-rendering'}.items(),
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', '/robot_description', '-name', 'scuttle', '-z', '0.04175'],
            output='screen',
        )],
    )

    bridge = TimerAction(
        period=5.0,
        actions=[Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
            output='screen',
        )],
    )

    odom_to_tf = TimerAction(
        period=6.0,
        actions=[Node(
            package='scuttle_gazebo',
            executable='odom_to_tf.py',
            name='odom_to_tf',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )],
    )

    return [robot_state_publisher, gazebo, spawn_robot, bridge, odom_to_tf]


def generate_launch_description():
    pkg_desc = get_package_share_directory('scuttle_description')
    drlnav_models = os.path.join(
        os.path.expanduser('~'), 'ros2_ws', 'src', 'turtlebot3_drlnav',
        'src', 'turtlebot3_simulations', 'turtlebot3_gazebo', 'models'
    )

    gz_resource_paths = [
        pkg_desc,
        drlnav_models,
        os.path.join(os.path.expanduser('~'), '.gz', 'models'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('stage', default_value='1',
                              description='DRL training stage (1-9)'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(gz_resource_paths)),
        OpaqueFunction(function=launch_setup),
    ])
