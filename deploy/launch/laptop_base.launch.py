#!/usr/bin/env python3
"""
laptop_base.launch.py — the laptop's always-on base. Auto-started by scuttle-laptop-base.service.

SLAM + GDM (map_throttle + gmrf_node writing CSV + gmrf_to_grid bridge) + the dashboard
(rosbridge+web) + mission supervisor (brings up laptop_frontier on frontier mode) + map_saver.
The zenoh router runs as its own systemd service, started before this launch.
"""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

_HERE = os.path.dirname(os.path.abspath(__file__))
_DEPLOY = os.path.dirname(_HERE)
_GMRF_CSV_DIR = os.path.expanduser('~/.scuttle/gmrf')


def generate_launch_description():
    pkg_dashboard = get_package_share_directory('scuttle_dashboard')
    supervisor = os.path.join(_DEPLOY, 'nodes', 'mission_supervisor.py')
    gmrf_to_grid = os.path.join(_DEPLOY, 'nodes', 'gmrf_to_grid.py')
    map_saver = os.path.join(_DEPLOY, 'nodes', 'map_saver.py')
    frontier_launch = os.path.join(_HERE, 'laptop_frontier.launch.py')

    os.makedirs(_GMRF_CSV_DIR, exist_ok=True)

    return LaunchDescription([
        # SLAM (gated on /scan)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(_HERE, 'laptop_slam.launch.py'))),

        # GDM — always on. map_throttle feeds gmrf_node; gmrf_node writes CSV; bridge republishes.
        Node(package='scuttle_base', executable='map_throttle', name='map_throttle',
             output='screen',
             parameters=[{'input_topic': '/map', 'output_topic': '/map_throttled',
                          'period_sec': 30.0, 'use_sim_time': False}]),
        Node(package='gmrf_gas_mapping', executable='gmrf_node', name='gmrf_node',
             output='screen',
             parameters=[{'use_sim_time': False, 'frame_id': 'map',
                          'occupancy_map_topic': '/map_throttled', 'sensor_topic': '/gdm/gas_1',
                          'observation_topic': '/gdm/obs_unused',
                          'output_csv_folder': _GMRF_CSV_DIR,
                          'exec_freq': 2.0, 'cell_size': 0.5,
                          'min_sensor_val': 0.0, 'max_sensor_val': 110.0,
                          'GMRF_lambdaPrior': 5.0, 'GMRF_lambdaObs': 0.5,
                          'GMRF_lambdaObsLoss': 0.01}]),
        ExecuteProcess(
            cmd=['python3', gmrf_to_grid, '--ros-args',
                 '-p', f'csv_path:={os.path.join(_GMRF_CSV_DIR, "GMRF_mean.csv")}',
                 '-p', 'cell_size:=0.5'],
            name='gmrf_to_grid', output='screen'),

        # Dashboard (rosbridge + static web)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dashboard, 'launch', 'dashboard.launch.py'))),

        # Mission supervisor + map saver
        ExecuteProcess(
            cmd=['python3', supervisor,
                 '--role', 'laptop',
                 '--launch-file', frontier_launch,
                 '--state-topic', '/mission/state/laptop'],
            name='mission_supervisor_laptop', output='screen'),
        ExecuteProcess(cmd=['python3', map_saver], name='map_saver', output='screen'),
    ])
