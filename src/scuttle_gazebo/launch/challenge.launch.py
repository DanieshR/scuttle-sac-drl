#!/usr/bin/env python3
"""
SCUTTLE Challenge World Launch
- Loads 'gas_maze.sdf' by default
- Perfect for DRL Obstacle/Coverage testing
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    pkg_desc = get_package_share_directory('scuttle_description')
    pkg_gazebo = get_package_share_directory('scuttle_gazebo')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')
    
    # --- CHANGE 1: DEFAULT TO GAS MAZE ---
    # We grab the world name, defaulting to gas_maze if not specified
    world_name = LaunchConfiguration('world').perform(context)
    
    # Construct the full path to the world file
    # Priority: 1. Full Path Provided -> 2. Local Package World -> 3. System World
    if os.path.exists(world_name):
        world_file = world_name
    else:
        # Check inside scuttle_gazebo/worlds/
        local_world_path = os.path.join(pkg_gazebo, 'worlds', f'{world_name}.sdf')
        if os.path.exists(local_world_path):
            world_file = local_world_path
        else:
            # Fallback to empty if file missing
            print(f"[WARNING] World '{world_name}' not found. Loading empty.")
            world_file = os.path.join(pkg_gazebo, 'worlds', 'empty.sdf')

    print(f"[challenge.launch.py] Target World: {world_file}")

    # Robot Description (URDF/Xacro)
    xacro_file = os.path.join(pkg_desc, 'urdf', 'scuttle.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # 1. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # 2. Launch Gazebo with the World
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # 3. Spawn Robot (Offset to avoid spawning inside a wall)
    spawn_robot = TimerAction(
        period=2.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', '/robot_description', 
                '-z', '0.05', 
                '-x', '-1.5', # Start slightly back to face the maze
                '-y', '-1.5'
            ],
            output='screen'
        )]
    )

    # 4. Bridge (Topics + Reset Control)
    bridge = TimerAction(
        period=3.0,
        actions=[Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                # Sensors & Actuators
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                
                # IMPORTANT: World Control for DRL Resetting
                '/model/scuttle/pose@geometry_msgs/msg/Pose]gz.msgs.Pose',
                '/world/gas_maze/control@ros_gz_interfaces/msg/WorldControl]gz.msgs.WorldControl',
                '/world/gas_maze/set_pose@ros_gz_interfaces/msg/WorldControl]gz.msgs.WorldControl' # Sometimes needed
            ],
            output='screen'
        )]
    )

    # 5. Odom to TF
    odom_to_tf = TimerAction(
        period=4.0,
        actions=[Node(
            package='scuttle_gazebo',
            executable='odom_to_tf.py',
            name='odom_to_tf',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )]
    )

    return [robot_state_publisher, gazebo, spawn_robot, bridge, odom_to_tf]


def generate_launch_description():
    pkg_desc = get_package_share_directory('scuttle_description')
    pkg_gazebo = get_package_share_directory('scuttle_gazebo')

    # Ensure Gazebo can find our models
    gz_resource_paths = [pkg_desc, pkg_gazebo, os.path.join(os.path.expanduser('~'), '.gz', 'models')]
    
    set_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join(gz_resource_paths)
    )

    # Argument to allow overriding the world name
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='gas_maze',
        description='Name of the world file (without .sdf) in scuttle_gazebo/worlds'
    )

    return LaunchDescription([
        set_model_path,
        world_arg,
        OpaqueFunction(function=launch_setup)
    ])