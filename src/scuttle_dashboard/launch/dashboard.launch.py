#!/usr/bin/env python3
"""
dashboard.launch.py — the operator console, run on the BASE STATION (laptop).

Starts two things:
  1. rosbridge_websocket (+ rosapi) on ws://localhost:9090 — the ROS 2 ↔ browser bridge.
     Because the laptop is zenoh-sourced, this bridge also sees the Pi's topics over the link.
  2. A static HTTP server for the web app, so the browser can load index.html + ros-bindings.js
     + vendored roslib over http:// (file:// breaks the websocket origin + module loading).

Then point a browser at  http://localhost:8080  (printed below).

  source /opt/ros/jazzy/setup.bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch scuttle_dashboard dashboard.launch.py
  # optional: ros2 launch scuttle_dashboard dashboard.launch.py http_port:=9000 bridge_port:=9091

Prereq:  sudo apt install ros-jazzy-rosbridge-suite
The app falls back to SIM mode (with a badge) if the bridge isn't reachable, so it still opens
for review without a robot.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    web_dir = os.path.join(get_package_share_directory('scuttle_dashboard'), 'web')

    bridge_port = LaunchConfiguration('bridge_port')
    http_port = LaunchConfiguration('http_port')

    return LaunchDescription([
        DeclareLaunchArgument('bridge_port', default_value='9090',
                              description='rosbridge websocket port (the app connects to ws://localhost:<bridge_port>)'),
        DeclareLaunchArgument('http_port', default_value='8080',
                              description='static HTTP port that serves the web app'),

        # 1) ROS 2 ↔ browser websocket bridge
        Node(package='rosbridge_server', executable='rosbridge_websocket',
             name='rosbridge_websocket', output='screen',
             parameters=[{'port': bridge_port}]),
        Node(package='rosapi', executable='rosapi_node', name='rosapi', output='screen'),

        # 2) Static web server for the SPA (serves the installed share/web directory)
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server',
                 PythonExpression(["str(", http_port, ")"]),
                 '--directory', web_dir],
            output='screen'),

        LogInfo(msg=['SCUTTLE dashboard → open  http://localhost:',
                     http_port, '   (rosbridge on ws://localhost:', bridge_port, ')']),
    ])
