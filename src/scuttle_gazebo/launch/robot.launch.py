#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_desc = get_package_share_directory('scuttle_description')
    pkg_gazebo = get_package_share_directory('scuttle_gazebo')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')
    
    # Set Gazebo resource path
    gz_resource_paths = [pkg_desc, os.path.join(os.path.expanduser('~'), '.gz', 'models')]
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join(gz_resource_paths)
    
    set_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join(gz_resource_paths)
    )
    
    # Robot description
    xacro_file = os.path.join(pkg_desc, 'urdf', 'scuttle.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )
    
    # Joint State Publisher (publishes joint states from Gazebo)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )
    
    # Launch Gazebo
    world_file = os.path.join(pkg_gazebo, 'worlds', 'empty.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'{world_file} -r'}.items()
    )
    
    # Spawn robot (removed -name to avoid namespace prefix)
    spawn_robot = TimerAction(
        period=2.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-z', '0.04175'],
            output='screen'
        )]
    )
    
    # Bridge for Gazebo topics to ROS (scan_raw instead of scan)
    bridge = TimerAction(
        period=3.0,
        actions=[Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/scan_raw@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/odom_raw@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            ],
            output='screen',
            remappings=[
                ('/cmd_vel', '/cmd_vel'),
            ]
        )]
    )
    
    # Frame fixer node to republish scan with correct frame_id
    frame_fixer = TimerAction(
        period=4.0,
        actions=[Node(
            package='scuttle_gazebo',
            executable='fix_scan_frame.py',
            name='scan_frame_fixer',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )]
    )
    
    # Odom frame fixer node
    odom_fixer = TimerAction(
        period=4.0,
        actions=[Node(
            package='scuttle_gazebo',
            executable='fix_odom_frame.py',
            name='odom_frame_fixer',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )]
    )

    return LaunchDescription([
        set_model_path,
        robot_state_publisher,
        joint_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        frame_fixer,
        odom_fixer,
    ])
