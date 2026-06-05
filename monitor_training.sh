#!/usr/bin/env bash
# Quick training status monitor
# Usage: ./monitor_training.sh [--watch]   # --watch refreshes every 10s
source /opt/ros/jazzy/setup.bash 2>/dev/null || true

REPO_DIR="$HOME/ros2_ws/src/DRL-Robot-Navigation-ROS2"
PYTHON="$HOME/drl_env/bin/python3"

if [ "${1:-}" = "--watch" ]; then
    while true; do
        clear
        cd "$REPO_DIR"
        PYTHONPATH=src/drl_navigation_ros2:${PYTHONPATH:-} "$PYTHON" src/drl_navigation_ros2/monitor.py 2>/dev/null
        echo ""
        echo "  (refreshing every 10s — Ctrl+C to stop)"
        sleep 10
    done
else
    cd "$REPO_DIR"
    PYTHONPATH=src/drl_navigation_ros2:${PYTHONPATH:-} "$PYTHON" src/drl_navigation_ros2/monitor.py 2>/dev/null
fi
