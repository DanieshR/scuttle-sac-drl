"""Launch files must import and produce a LaunchDescription.

Run in a ROS-sourced shell (needs launch/launch_ros/ament_index):
    source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
    python3 -m pytest deploy/tests/test_launch_files.py -v
"""
import importlib.util
import os

from launch import LaunchDescription

LAUNCH_DIR = os.path.join(os.path.dirname(__file__), '..', 'launch')


def _load(name):
    path = os.path.join(LAUNCH_DIR, name)
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_laptop_frontier_builds():
    ld = _load('laptop_frontier.launch.py').generate_launch_description()
    assert isinstance(ld, LaunchDescription)


def test_pi_base_builds():
    ld = _load('pi_base.launch.py').generate_launch_description()
    assert isinstance(ld, LaunchDescription)
    assert len(ld.entities) >= 2          # hardware include + supervisor


def test_laptop_base_builds():
    ld = _load('laptop_base.launch.py').generate_launch_description()
    assert isinstance(ld, LaunchDescription)
    assert len(ld.entities) >= 6          # slam + 3 gdm + dashboard + supervisor + map_saver


def test_exploration_gdm_removed():
    assert not os.path.exists(os.path.join(LAUNCH_DIR, 'laptop_exploration_gdm.launch.py'))
