"""
Live GDM (Gas Distribution Mapping) on the real robot.

Pipeline: fake_gas_sensor (/odom -> /gdm/gas_1) -> gmrf_node, which builds
its grid from a throttled copy of SLAM's /map (map_throttle) so it doesn't
discard the accumulated gas estimate on every SLAM map republish.

Run alongside standby.launch.py / exploration.launch.py and slam.launch.py.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scuttle_base',
            executable='fake_gas_sensor',
            name='fake_gas_sensor',
            output='screen',
            parameters=[{
                'source_x': 2.0,
                'source_y': 2.0,
                'peak_ppm': 100.0,
                'baseline_ppm': 5.0,
                'sigma': 1.5,
                'noise_std': 1.0,
                'update_rate': 1.0,
            }],
        ),
        Node(
            package='scuttle_base',
            executable='map_throttle',
            name='map_throttle',
            output='screen',
            parameters=[{
                'input_topic': '/map',
                'output_topic': '/map_throttled',
                'period_sec': 30.0,
            }],
        ),
        Node(
            package='gmrf_gas_mapping',
            executable='gmrf_node',
            name='gmrf_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'frame_id': 'map',
                'occupancy_map_topic': '/map_throttled',
                'sensor_topic': '/gdm/gas_1',
                'observation_topic': '/gdm/obs_unused',
                'output_csv_folder': '',
                'exec_freq': 2.0,
                'cell_size': 0.5,
                'min_sensor_val': 0.0,
                'max_sensor_val': 110.0,
                'GMRF_lambdaPrior': 5.0,
                'GMRF_lambdaObs': 0.5,
                'GMRF_lambdaObsLoss': 0.01,
            }],
        ),
    ])
