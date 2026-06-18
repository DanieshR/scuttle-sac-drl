from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scuttle_base',
            executable='scuttle_base_node',
            name='scuttle_base',
            parameters=[{
                'port': '/dev/ttyESP32',
                'baud': 115200,
                'wheel_radius': 0.04175,
                'wheel_separation': 0.4247,
                'counts_per_rev': 16384,
                'publish_tf': True,
            }],
            output='screen',
        )
    ])
