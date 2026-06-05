import os
from launch import LaunchDescription, LaunchContext
from launch.actions import EmitEvent, SetLaunchConfiguration
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from launch_ros.actions import LifecycleNode, Node
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory

lc = LaunchContext()

def generate_launch_description():

    pkg_scuttle_nav = get_package_share_directory('scuttle_nav')

    # SLAM config file
    config_file = os.path.join(
        pkg_scuttle_nav,
        'config',
        'mapper_params_online_async.yaml'
    )

    # RViz config file (create this)
    rviz_config = os.path.join(
        pkg_scuttle_nav,
        'rviz',
        'slam_config.rviz'
    )

    # SLAM Toolbox Lifecycle Node
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',   # <--- FIXED: Jazzy requires this
        output='screen',
        parameters=[
            config_file,
            {"use_sim_time": True}
        ],
        remappings=[("/scan", "/scan")],
    )

    # Configure event
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    # Activate event
    activate_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_ACTIVATE
        )
    )

    # RViz Node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace='',  # optional but recommended
        output='screen',
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}]
    )

    return LaunchDescription([
        SetLaunchConfiguration('use_sim_time', 'true'),
        slam_node,
        configure_event,
        activate_event,
        #rviz,
    ])
