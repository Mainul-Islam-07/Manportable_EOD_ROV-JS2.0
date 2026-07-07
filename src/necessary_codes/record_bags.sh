#!/usr/bin/env bash
set -euo pipefail

# ---------------- Config ----------------
BAG_DIR="${BAG_DIR:-$HOME/rosbags}"        # storage folder
SEGMENT_SEC="${SEGMENT_SEC:-300}"          # 5 minutes per file
MAX_SIZE_GB="${MAX_SIZE_GB:-30}"           # remove oldest if folder exceeds this
CLEAN_INTERVAL_SEC="${CLEAN_INTERVAL_SEC:-300}"  # cleanup check every 5 min
# ----------------------------------------

TOPICS=(
  /arm_active_motors
  /arm_fault_stop
  /arm_joint_commands
  /coordinator/diagnostics
  /drive_active_motors
  /drive_fault_stop
  /drive_flipper_states
  /drive_motor_commands
  /fire_mode
  /flipper_joint_commands
  /joint_states
  /motor_commands
  /motor_diagnostics
  /sbus/control
  /sbus/joint_states
)

mkdir -p "$BAG_DIR"

cleanup() {
  # While folder > MAX_SIZE_GB, delete the oldest bag dir
  local max_kb=$(( MAX_SIZE_GB * 1024 * 1024 ))
  while :; do
    local used_kb
    used_kb=$(du -s "$BAG_DIR" | awk '{print $1}')
    [ "$used_kb" -le "$max_kb" ] && break

    local oldest
    oldest=$(find "$BAG_DIR" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' \
             | sort -n | head -n1 | cut -d' ' -f2-)
    [ -z "$oldest" ] && break
    echo "[cleanup] size limit exceeded, removing $oldest"
    rm -rf "$oldest"
  done
}

# Background cleanup loop
( while true; do cleanup; sleep "$CLEAN_INTERVAL_SEC"; done ) &
CLEAN_PID=$!
trap 'kill "$CLEAN_PID" 2>/dev/null; kill 0 2>/dev/null' EXIT INT TERM

cleanup  # run once at startup

cd "$BAG_DIR"
exec ros2 bag record "${TOPICS[@]}" \
  --max-bag-duration "$SEGMENT_SEC" \
  -o "session_$(date +%Y%m%d_%H%M%S)"
