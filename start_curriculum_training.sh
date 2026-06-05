#!/usr/bin/env bash
# Curriculum training helper for SCUTTLE SAC/TD3 — stages 1-9
#
# Usage:
#   ./start_curriculum_training.sh [stage]       # launch Gazebo + write stage file
#   ./start_curriculum_training.sh status         # show current stage + episode count
#   ./start_curriculum_training.sh advance        # advance to next stage (manual trigger)
#
# 4-terminal workflow (after running this script in T1):
#   T2: source ~/drl_env/bin/activate && ros2 run turtlebot3_drl gazebo_goals
#   T3: source ~/drl_env/bin/activate && ros2 run turtlebot3_drl environment
#   T4: source ~/drl_env/bin/activate && ros2 run turtlebot3_drl train_agent sac
#
# Odom monitor (optional 5th terminal — no venv needed):
#   ros2 run scuttle_gazebo odom_reset_monitor.py

set -e

STAGE_FILE=/tmp/drlnav_current_stage.txt
export DRLNAV_BASE_PATH=/home/daniesh/ros2_ws/src/turtlebot3_drlnav

# ── Goal-rate thresholds to advance stages (adjust as training progresses) ─
# These are suggested minimums — advance manually when confident.
declare -A ADVANCE_THRESHOLD=(
  [1]=70   # 70% goal rate before moving to stage 2
  [2]=70
  [3]=65
  [4]=60
  [5]=60
  [6]=55
  [7]=55
  [8]=50
  [9]=50   # final stage — no advance
)

current_stage() {
  if [[ -f "$STAGE_FILE" ]]; then cat "$STAGE_FILE" | tr -d '[:space:]'; else echo "1"; fi
}

case "${1:-}" in

  status)
    STAGE=$(current_stage)
    MODEL_DIR=/home/daniesh/ros2_ws/src/turtlebot3_drlnav/src/turtlebot3_drl/model
    echo "Current stage : $STAGE"
    echo "Stage file    : $STAGE_FILE"
    if [[ -d "$MODEL_DIR" ]]; then
      LATEST=$(ls -t "$MODEL_DIR" 2>/dev/null | head -1)
      echo "Latest model  : ${LATEST:-none}"
    fi
    echo "Advance threshold for stage $STAGE: ${ADVANCE_THRESHOLD[$STAGE]:-?}% goal rate"
    ;;

  advance)
    STAGE=$(current_stage)
    if (( STAGE >= 9 )); then
      echo "Already at final stage 9 — nothing to advance."
      exit 0
    fi
    NEXT=$(( STAGE + 1 ))
    echo "Advancing stage $STAGE → $NEXT"
    echo "$NEXT" > "$STAGE_FILE"
    echo "Stage file updated. Restart Gazebo with:"
    echo "  ros2 launch scuttle_bringup drl_tomasvr.launch.py stage:=$NEXT"
    ;;

  [1-9])
    STAGE=$1
    echo "Setting stage $STAGE and launching Gazebo..."
    echo "$STAGE" > "$STAGE_FILE"
    source /opt/ros/jazzy/setup.bash
    source /home/daniesh/ros2_ws/install/setup.bash
    ros2 launch scuttle_bringup drl_tomasvr.launch.py stage:="$STAGE"
    ;;

  *)
    # Default: launch at current stage (or stage 1 if no stage file)
    STAGE=$(current_stage)
    echo "Launching at current stage: $STAGE"
    echo "$STAGE" > "$STAGE_FILE"
    source /opt/ros/jazzy/setup.bash
    source /home/daniesh/ros2_ws/install/setup.bash
    ros2 launch scuttle_bringup drl_tomasvr.launch.py stage:="$STAGE"
    ;;

esac
