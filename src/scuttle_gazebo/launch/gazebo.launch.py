#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get package directories
    pkg_scuttle = get_package_share_directory('scuttle_gazebo')
    pkg_desc    = get_package_share_directory('scuttle_description')
    pkg_ros_gz  = get_package_share_directory('ros_gz_sim')

    # Set Gazebo resource paths
    gz_resource_paths = [
        pkg_desc,
        os.path.join(os.path.expanduser('~'), '.gz', 'models')
    ]
    
    models_dir = os.path.join(pkg_scuttle, 'models')
    if os.path.exists(models_dir):
        gz_resource_paths.append(models_dir)
    
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing_gz_path:
        for path in existing_gz_path.split(os.pathsep):
            if path and path not in gz_resource_paths:
                gz_resource_paths.append(path)
    
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join(gz_resource_paths)
    
    set_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join(gz_resource_paths)
    )

    # Robot description (XACRO → URDF)
    xacro_file = os.path.join(pkg_desc, 'urdf', 'scuttle.urdf.xacro')
    if not os.path.exists(xacro_file):
        xacro_file = os.path.join(pkg_desc, 'urdf', 'scuttle.xacro')
    
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), 
        value_type=str
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # Launch Gazebo (start paused to allow proper initialization)
    world_file = os.path.join(pkg_scuttle, 'worlds', 'empty.sdf')
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'{world_file} -r',  # -r means start running (not paused)
            'on_exit_shutdown': 'True'
        }.items()
    )

    # Spawn robot after Gazebo is ready
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_scuttle',
                output='screen',
                arguments=[
                    '-topic', 'robot_description',
                    '-name', 'scuttle',
                    '-x', '0', '-y', '0', '-z', '0.05',
                    '-allow-renaming', 'true'
                ],
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # ROS-Gazebo bridge (including clock)
    bridge_config = os.path.join(pkg_desc, 'config', 'bridge_config.yaml')
    ros_gz_bridge = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge',
                output='screen',
                arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # Load joint state broadcaster (after spawn)
    load_joint_state_broadcaster = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                name='spawner_joint_state_broadcaster',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # Load diff drive controller (after joint state broadcaster succeeds)
    load_diff_drive_controller = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                name='spawner_diff_drive_controller',
                arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        set_model_path,
        robot_state_publisher,
        gz_sim_launch,
        spawn_robot,
        ros_gz_bridge,
        load_joint_state_broadcaster,
        load_diff_drive_controller
    ])
