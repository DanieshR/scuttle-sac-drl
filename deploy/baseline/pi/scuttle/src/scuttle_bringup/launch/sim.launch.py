#!/usr/bin/env python3
"""
Clean SCUTTLE Simulation Launch File
- Uses DiffDrive plugin only (no ros2_control)
- No namespace prefixes
- Simple bridge configuration
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os


def launch_setup(context, *args, **kwargs):
    pkg_desc = get_package_share_directory('scuttle_description')
    pkg_gazebo = get_package_share_directory('scuttle_gazebo')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')
    
    world_name = LaunchConfiguration('world').perform(context)
    
    # Robot description
    xacro_file = os.path.join(pkg_desc, 'urdf', 'scuttle.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    
    # Robot state publisher
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
    
    # Determine world file
    local_world = os.path.join(pkg_gazebo, 'worlds', f'{world_name}.sdf')
    system_world = f'/usr/share/gz/gz-sim8/worlds/{world_name}.sdf'
    
    if os.path.isfile(world_name):
        world_file = world_name
    elif os.path.isfile(local_world):
        world_file = local_world
    elif os.path.isfile(system_world):
        world_file = system_world
    else:
        world_file = os.path.join(pkg_gazebo, 'worlds', 'empty.sdf')
        print(f"[WARNING] World '{world_name}' not found, using empty.sdf")
    
    print(f"[sim.launch.py] Loading world: {world_file}")
    
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'{world_file} -r'}.items()
    )
    
    # Spawn robot WITHOUT name to avoid namespace
    spawn_robot = TimerAction(
        period=2.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', '/robot_description', '-z', '0.04175'],
            output='screen'
        )]
    )
    
    # Bridge - simple and clean
    bridge = TimerAction(
        period=3.0,
        actions=[Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/gas_sensor@std_msgs/msg/Float32[gz.msgs.Float',
            ],
            output='screen'
        )]
    )

    # Odom to TF publisher
    odom_tf_publisher = TimerAction(
        period=4.0,
        actions=[Node(
            package='scuttle_gazebo',
            executable='odom_to_tf.py',
            name='odom_to_tf',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )]
    )

    # Gas sensor bridge: gz Float → olfaction_msgs/GasSensor on /gdm/gas_1
    gas_sensor_bridge = TimerAction(
        period=4.0,
        actions=[Node(
            package='scuttle_gazebo',
            executable='gas_sensor_bridge.py',
            name='gas_sensor_bridge',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )]
    )

    return [
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        odom_tf_publisher,
        gas_sensor_bridge,
    ]


def generate_launch_description():
    pkg_desc = get_package_share_directory('scuttle_description')

    # Gazebo resource path
    gz_resource_paths = [pkg_desc, os.path.join(os.path.expanduser('~'), '.gz', 'models')]

    set_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join(gz_resource_paths)
    )

    # Gazebo plugin path (needed for libGasSourcePlugin.so)
    set_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=os.path.join(get_package_prefix('scuttle_gazebo'), 'lib', 'scuttle_gazebo')
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='drl_training',
        description='World name or full path to .sdf file'
    )
    
    return LaunchDescription([
        set_model_path,
        set_plugin_path,
        world_arg,
        OpaqueFunction(function=launch_setup)
    ])