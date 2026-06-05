#!/usr/bin/env bash
# ============================================================
# DRL Training Watchdog — auto-restarts training if it crashes
# Usage:
#   ./watchdog_drl_training.sh             # fresh on first start
#   ./watchdog_drl_training.sh --resume    # load weights on every restart
#   nohup ./watchdog_drl_training.sh --resume > ~/watchdog.log 2>&1 &
# ============================================================
LAUNCHER="$HOME/ros2_ws/start_drl_training.sh"
LOG_DIR="$HOME/ros2_ws/src/DRL-Robot-Navigation-ROS2/log"
MAX_RESTARTS=99
RESTART_DELAY=15   # seconds between restarts

RESUME_FLAG=""
for arg in "$@"; do
    [ "$arg" = "--resume" ] && RESUME_FLAG="--resume"
done

mkdir -p "$LOG_DIR"

echo "========================================"
echo "  DRL Training Watchdog started"
echo "  $(date)"
echo "  Resume mode: ${RESUME_FLAG:-off}"
echo "  PID: $$"
echo "========================================"

attempt=0
while [ "$attempt" -lt "$MAX_RESTARTS" ]; do
    attempt=$((attempt + 1))
    echo ""
    echo "--- Attempt $attempt / $MAX_RESTARTS  [$(date)] ---"

    # After first attempt, always resume from saved weights
    if [ "$attempt" -gt 1 ]; then
        RESUME_FLAG="--resume"
    fi

    bash "$LAUNCHER" $RESUME_FLAG
    EXIT_CODE=$?

    if [ "$EXIT_CODE" -eq 0 ]; then
        echo "Training completed successfully (epoch limit reached)."
        exit 0
    fi

    echo "[WARN] Training exited with code $EXIT_CODE. Restarting in ${RESTART_DELAY}s..."
    sleep "$RESTART_DELAY"
done

echo "[ERROR] Reached maximum restart limit ($MAX_RESTARTS). Manual intervention needed."
exit 1
