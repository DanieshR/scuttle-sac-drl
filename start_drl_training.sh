#!/usr/bin/env bash
# ============================================================
# SCUTTLE DRL SAC Training Launcher
# Usage:
#   ./start_drl_training.sh              # fresh training
#   ./start_drl_training.sh --resume     # load saved weights
# ============================================================
set -euo pipefail

REPO_DIR="$HOME/ros2_ws/src/DRL-Robot-Navigation-ROS2"
TRAIN_SCRIPT="src/drl_navigation_ros2/train.py"
PYTHON="$HOME/drl_env/bin/python3"
LOG_DIR="$REPO_DIR/log"
LOG_FILE="$LOG_DIR/training_$(date +%Y%m%d_%H%M%S).log"
GAZEBO_WAIT_TIMEOUT=60   # seconds to wait for Gazebo topics

RESUME_FLAG=""
for arg in "$@"; do
    case "$arg" in
        --resume) RESUME_FLAG="--resume" ;;
    esac
done

# ── Source ROS2 environment ───────────────────────────────────
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

mkdir -p "$LOG_DIR"

echo "=============================================="
echo "  SCUTTLE SAC DRL Training Launcher"
echo "  $(date)"
echo "  Resume: ${RESUME_FLAG:-no}"
echo "  Log: $LOG_FILE"
echo "=============================================="

# ── Kill any stale Gazebo / training processes ────────────────
if pgrep -f "gz sim" > /dev/null 2>&1; then
    echo "[WARN] Gazebo already running. Stopping it first..."
    pkill -f "gz sim" 2>/dev/null || true
    sleep 3
fi
if pgrep -f "train.py" > /dev/null 2>&1; then
    echo "[WARN] Stale train.py found. Stopping it..."
    pkill -f "train.py" 2>/dev/null || true
    sleep 1
fi

# ── Launch Gazebo DRL training world ─────────────────────────
echo "[1/3] Launching Gazebo DRL training world..."
ros2 launch scuttle_bringup drl_training.launch.py \
    > "$LOG_DIR/gazebo.log" 2>&1 &
GAZEBO_PID=$!
echo "  Gazebo PID: $GAZEBO_PID"

# ── Wait for /scan topic (bridge is up + robot spawned) ───────
echo "[2/3] Waiting for /scan and /odom topics (max ${GAZEBO_WAIT_TIMEOUT}s)..."
WAIT_START=$(date +%s)
while true; do
    SCAN_OK=$(ros2 topic list 2>/dev/null | grep -c "^/scan$" || true)
    ODOM_OK=$(ros2 topic list 2>/dev/null | grep -c "^/odom$" || true)
    if [ "$SCAN_OK" -ge 1 ] && [ "$ODOM_OK" -ge 1 ]; then
        echo "  /scan and /odom are live."
        break
    fi
    NOW=$(date +%s)
    ELAPSED=$((NOW - WAIT_START))
    if [ "$ELAPSED" -ge "$GAZEBO_WAIT_TIMEOUT" ]; then
        echo "[ERROR] Topics not available after ${GAZEBO_WAIT_TIMEOUT}s."
        echo "        Check $LOG_DIR/gazebo.log for Gazebo errors."
        kill "$GAZEBO_PID" 2>/dev/null || true
        exit 1
    fi
    echo "  ...waiting ($ELAPSED s elapsed)"
    sleep 3
done

# Extra buffer for LiDAR warmup
sleep 3

# ── Start training ─────────────────────────────────────────────
echo "[3/3] Starting SAC training${RESUME_FLAG:+ (resuming from checkpoint)}..."
echo "  Output → $LOG_FILE"
echo "  TensorBoard: tensorboard --logdir $REPO_DIR/runs"
echo ""

cd "$REPO_DIR"
PYTHONPATH=src/drl_navigation_ros2:${PYTHONPATH:-} \
    "$PYTHON" "$TRAIN_SCRIPT" $RESUME_FLAG 2>&1 | tee "$LOG_FILE"

echo ""
echo "Training finished. Log saved to: $LOG_FILE"
