#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_desc = get_package_share_directory('scuttle_description')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'scuttle.xacro')
    world_file = os.path.join(pkg_desc, 'worlds', 'drl_training.world')
    models_dir = os.path.join(pkg_desc, 'models')

    existing = os.environ.get('GAZEBO_MODEL_PATH', '')
    model_path = f"{models_dir}:{existing}" if existing else models_dir

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
            'use_sim_time': True,
        }]
    )

    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_file,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        additional_env={
            'GAZEBO_MODEL_PATH': model_path,
            'GAZEBO_MODEL_DATABASE_URI': '',
        },
        output='screen'
    )

    return LaunchDescription([robot_state_publisher, gzserver])
