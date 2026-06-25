#!/usr/bin/env bash
#
# robot_startup.sh
# Launches the full robot software stack in sequence.
# First command waits 10s, subsequent ones wait 3s each.
#
# Edit these paths/sources if your workspace layout changes.
# ---------------------------------------------------------------------------

# NOTE: do NOT use `set -u` here — ROS setup.bash files reference unset
# variables and will abort the script the moment they're sourced.

echo "[$(date '+%F %T')] robot_startup.sh: begin"

# ---- User / environment ---------------------------------------------------
RUN_USER="jontro_soinik_2_0-2"
HOME_DIR="/home/${RUN_USER}"
WS_SETUP="${HOME_DIR}/ros2_ws/install/setup.bash"   # <-- adjust to your workspace
RECORD_DIR="${HOME_DIR}"                            # dir containing record_bags.sh
CAN_BRINGUP="${HOME_DIR}/can_bringup.sh"

LOG_DIR="${HOME_DIR}/robot_startup_logs"
mkdir -p "${LOG_DIR}"

# ---- Source ROS -----------------------------------------------------------
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "[$(date '+%F %T')] sourced /opt/ros/humble"
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "[$(date '+%F %T')] sourced /opt/ros/jazzy"
else
    echo "[$(date '+%F %T')] ERROR: no ROS distro found in /opt/ros" >&2
fi

if [ -f "${WS_SETUP}" ]; then
    source "${WS_SETUP}"
    echo "[$(date '+%F %T')] sourced workspace ${WS_SETUP}"
else
    echo "[$(date '+%F %T')] WARNING: workspace setup not found at ${WS_SETUP}" >&2
fi

# ---- Helper: launch a command in the background, logged --------------------
PIDS=()
run_bg () {
    local name="$1"; shift
    echo "[$(date '+%F %T')] starting: ${name}"
    "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
    PIDS+=("$!")
}

# Clean shutdown: kill all children if the service stops
cleanup () {
    echo "[$(date '+%F %T')] stopping stack..."
    for pid in "${PIDS[@]}"; do
        kill "${pid}" 2>/dev/null
    done
    wait 2>/dev/null
    exit 0
}
trap cleanup SIGTERM SIGINT

# ---- Sequence -------------------------------------------------------------

# 1) MoveIt demo launch (no rviz) — 10s gap before next
run_bg "01_moveit_demo" ros2 launch part_assembly_for_urdf_moveit_config demo.launch.py use_rviz:=false
sleep 10

# 2) SBUS publisher
run_bg "02_sbus_publisher" ros2 run sbus_driver sbus_publisher
sleep 3

# 3) CAN bring-up (needs root; see notes on sudoers)
echo "[$(date '+%F %T')] starting: 03_can_bringup"
sudo "${CAN_BRINGUP}" >"${LOG_DIR}/03_can_bringup.log" 2>&1
sleep 3

# 4) CAN bus robot node
run_bg "04_canbus_robot" ros2 run ros2_canbus robot
sleep 3

# 5) flipper controller spawner, then coordinator launch
run_bg "05_flipper_and_coordinator" bash -c \
  'ros2 run controller_manager spawner flipper_controller && \
   ros2 launch part_assembly_for_urdf_coordinator coordinator.launch.py'
sleep 3

# 6) diagnostics
run_bg "06_diagnostics" ros2 run ros2_canbus diagnostics
sleep 3

# 7) light
run_bg "07_light" ros2 run ros2_canbus light
sleep 3

# 8) fire
run_bg "08_fire" ros2 run ros2_canbus fire
sleep 3

# 9) bag recording
cd "${RECORD_DIR}" || echo "[$(date '+%F %T')] WARNING: cannot cd to ${RECORD_DIR}" >&2
run_bg "09_record_bags" ./record_bags.sh

echo "[$(date '+%F %T')] full stack launched."

# Keep the script (and therefore the service) alive while children run
wait
