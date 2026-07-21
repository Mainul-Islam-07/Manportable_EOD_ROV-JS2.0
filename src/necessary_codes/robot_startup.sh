#!/usr/bin/env bash
#
# robot_startup.sh
# Launches the robot software stack in sequence.
#
# Sequence:
#   1) can_bringup.sh             (sudo, blocking; needs NOPASSWD sudoers)
#   2) bringup_sequence.launch.py (ros2 launch; internal staggering)
#   3) record_bags.sh             (rosbag; started after bringup topics are up)
#
# (audio + fake_memory_battery were removed.)
# Edit these paths/sources if your workspace layout changes.
# ---------------------------------------------------------------------------

# NOTE: do NOT use `set -u` here — ROS setup.bash files reference unset
# variables and will abort the script the moment they're sourced.

echo "[$(date '+%F %T')] robot_startup.sh: begin"

# ---- User / environment ---------------------------------------------------
RUN_USER="jontro_soinik_2_0-2"
HOME_DIR="/home/${RUN_USER}"
WS_SETUP="${HOME_DIR}/ros2_ws/install/setup.bash"   # <-- adjust to your workspace

NECESSARY_CODES_DIR="${HOME_DIR}/ros2_ws/src/necessary_codes"
CAN_BRINGUP="${NECESSARY_CODES_DIR}/can_bringup.sh"
RECORD_BAGS_SCRIPT="${NECESSARY_CODES_DIR}/record_bags.sh"

# Seconds to wait after launching bringup before starting rosbag, so the
# staggered nodes (~8 x 3 s) have created their topics first.
BRINGUP_SETTLE_SEC="${BRINGUP_SETTLE_SEC:-25}"

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

# 1) CAN bring-up (needs root). Under systemd there is no TTY to type a password,
#    so this requires a passwordless sudoers rule for can_bringup.sh
#    (see pi5_setup_commands.txt). Blocking; the script itself waits/retries.
echo "[$(date '+%F %T')] starting: 01_can_bringup"
if ! sudo -n true 2>/dev/null; then
    echo "[$(date '+%F %T')] ERROR: passwordless sudo unavailable -> CAN bring-up will fail. " \
         "Add a NOPASSWD sudoers rule for ${CAN_BRINGUP} (see pi5_setup_commands.txt)." >&2
fi
sudo "${CAN_BRINGUP}" >"${LOG_DIR}/01_can_bringup.log" 2>&1
can_rc=$?
if [ "${can_rc}" -ne 0 ]; then
    echo "[$(date '+%F %T')] WARNING: can_bringup exited ${can_rc} (see 01_can_bringup.log)" >&2
fi
# Log resulting link state to the journal for quick diagnosis.
ip -brief link show can_drive 2>&1 | head -1
ip -brief link show can_arm   2>&1 | head -1
sleep 2

# 2) Main bringup sequence launch (handles its own internal staggering for
#    sim_headless/sbus/robot/light/fire/mode/diagnostics/bms/coordinator).
run_bg "02_bringup_launch" ros2 launch ros2_canbus bringup_sequence.launch.py

# 3) rosbag recording — start only after the bringup topics are up.
echo "[$(date '+%F %T')] waiting ${BRINGUP_SETTLE_SEC}s for bringup topics before rosbag..."
sleep "${BRINGUP_SETTLE_SEC}"
run_bg "03_record_bags" "${RECORD_BAGS_SCRIPT}"

echo "[$(date '+%F %T')] full stack launched."

# Keep the script (and therefore the service) alive while children run
wait
