#!/usr/bin/env bash
# Quick stage-1 smoke test — runs for TIMEOUT_SEC then reports.
# Run inside the container with workspace already sourced.
set -e

TIMEOUT_SEC=300   # 5 minutes total
LOG=/tmp/verify_stage1.log
AGENT_LOG=/tmp/agent_stdout.log

source /opt/ros/humble/setup.bash
source /home/turtlebot3_drlnav/install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/turtlebot3_drlnav/src/turtlebot3_simulations/turtlebot3_gazebo/models
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/turtlebot3_drlnav/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_drl_world/obstacle_plugin/lib
export ROS_DOMAIN_ID=1

ts() { date '+%H:%M:%S'; }

cleanup() {
    echo "[$(ts)] Cleaning up..." | tee -a $LOG
    pkill -f "gzserver|gzclient|gazebo_goals|drl_environment|train_agent|robot_state_publisher" 2>/dev/null || true
    sleep 3
    pkill -9 -f "gzserver|gzclient|gazebo_goals|drl_environment|train_agent|robot_state_publisher" 2>/dev/null || true
}
trap cleanup EXIT

> $LOG
> $AGENT_LOG

echo "[$(ts)] Starting Gazebo stage1..." | tee -a $LOG
ros2 launch turtlebot3_gazebo turtlebot3_drl_stage1.launch.py pause:=false \
    >> /tmp/gazebo.log 2>&1 &
sleep 50
echo "[$(ts)] Gazebo started" | tee -a $LOG

echo "[$(ts)] Starting gazebo_goals..." | tee -a $LOG
ros2 run turtlebot3_drl gazebo_goals >> /tmp/goals.log 2>&1 &
sleep 8
echo "[$(ts)] Goals started" | tee -a $LOG

echo "[$(ts)] Starting environment..." | tee -a $LOG
ros2 run turtlebot3_drl environment >> /tmp/env.log 2>&1 &
sleep 8
echo "[$(ts)] Environment started" | tee -a $LOG

echo "[$(ts)] Starting SAC agent..." | tee -a $LOG
ros2 run turtlebot3_drl train_agent sac >> $AGENT_LOG 2>&1 &
AGENT_PID=$!

# Monitor for remaining time
ELAPSED=66  # time already spent on startup (50+8+8)
EPISODES_SEEN=0

while [ $ELAPSED -lt $TIMEOUT_SEC ]; do
    sleep 10
    ELAPSED=$((ELAPSED + 10))

    # Count episode lines in agent log
    EPISODES_SEEN=$(grep -c "^Observe phase:\|^steps:" $AGENT_LOG 2>/dev/null || echo 0)
    EP_LINES=$(grep -E "^steps:|outcome|episode" $AGENT_LOG 2>/dev/null | tail -3)

    echo "[$(ts)] t=${ELAPSED}s  episodes_lines=${EPISODES_SEEN}" | tee -a $LOG
    if [ -n "$EP_LINES" ]; then
        echo "  AGENT: $EP_LINES" | tee -a $LOG
    fi

    # Check if agent died
    if ! kill -0 $AGENT_PID 2>/dev/null; then
        echo "[$(ts)] AGENT DIED unexpectedly" | tee -a $LOG
        echo "=== AGENT STDOUT ===" | tee -a $LOG
        cat $AGENT_LOG | tee -a $LOG
        break
    fi
done

echo "" | tee -a $LOG
echo "=== FINAL AGENT LOG (last 40 lines) ===" | tee -a $LOG
tail -40 $AGENT_LOG | tee -a $LOG

echo "" | tee -a $LOG
if [ "$EPISODES_SEEN" -gt 0 ]; then
    echo "RESULT: PASS — saw $EPISODES_SEEN episode/step lines from agent" | tee -a $LOG
else
    echo "RESULT: FAIL — no episode output from agent in ${TIMEOUT_SEC}s" | tee -a $LOG
    echo "=== ENV LOG ===" | tee -a $LOG
    tail -20 /tmp/env.log | tee -a $LOG
    echo "=== GAZEBO LOG ===" | tee -a $LOG
    tail -20 /tmp/gazebo.log | tee -a $LOG
fi
