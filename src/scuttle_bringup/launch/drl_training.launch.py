#!/usr/bin/env python3
"""
SCUTTLE DRL Training Launch File
- Fixed world: drl_training.sdf (10x10m room, 8 named obstacle models)
- Spawns SCUTTLE with explicit name "scuttle" for gz set_pose teleportation
- Mirrors sim.launch.py structure (known working) with DRL-specific overrides
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    pkg_desc = get_package_share_directory('scuttle_description')
    pkg_gazebo = get_package_share_directory('scuttle_gazebo')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg_desc, 'urdf', 'scuttle.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    world_file = os.path.join(pkg_gazebo, 'worlds', 'drl_training.sdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'{world_file} -r --headless-rendering'}.items()
    )

    # Explicit -name scuttle so gz service /world/drl_training/set_pose can target it
    spawn_robot = TimerAction(
        period=3.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', '/robot_description',
                '-name', 'scuttle',
                '-z', '0.04175',
            ],
            output='screen'
        )]
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
            output='screen'
        )]
    )

    odom_to_tf = TimerAction(
        period=6.0,
        actions=[Node(
            package='scuttle_gazebo',
            executable='odom_to_tf.py',
            name='odom_to_tf',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )]
    )

    return [
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        odom_to_tf,
    ]


def generate_launch_description():
    pkg_desc = get_package_share_directory('scuttle_description')

    gz_resource_paths = [pkg_desc, os.path.join(os.path.expanduser('~'), '.gz', 'models')]

    set_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join(gz_resource_paths)
    )

    return LaunchDescription([
        set_model_path,
        OpaqueFunction(function=launch_setup),
    ])
