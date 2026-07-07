#!/usr/bin/env bash
#
# robot_startup.sh
# Launches the full robot software stack in sequence.
#
# Sequence:
#   1) can_bringup.sh        (sudo, blocking)               -> 3s gap
#   2) fake_memory_battery.py (system python3, no venv)     -> 3s gap
#   3) bringup_sequence.launch.py (ros2 launch; handles its
#      own internal staggering for sim_headless/sbus/robot/
#      light/fire/mode/diagnostics/coordinator)             -> 3s gap
#   4) audio_mic_duplex.py   (venv python3)                 -> 20s gap
#   5) record_bags.sh        (cd into recording dir first)
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

# Scripts that moved under necessary_codes/
NECESSARY_CODES_DIR="${HOME_DIR}/ros2_ws/src/necessary_codes"
CAN_BRINGUP="${NECESSARY_CODES_DIR}/can_bringup.sh"
BATTERY_SCRIPT="${NECESSARY_CODES_DIR}/fake_memory_battery.py"
AUDIO_SCRIPT="${NECESSARY_CODES_DIR}/audio/audio_mic_duplex.py"
AUDIO_VENV_PY="${HOME_DIR}/manportable_audio_venv/venv/bin/python3"
RECORD_BAGS_SCRIPT="${NECESSARY_CODES_DIR}/record_bags.sh"

# Recording directory (unchanged — still the old home-based location, since
# record_bags.sh needs to run/save from here even though the script file
# itself now lives under necessary_codes/)
RECORD_DIR="${HOME_DIR}"

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

# 1) CAN bring-up (needs root; see notes on sudoers) — blocking, as before
echo "[$(date '+%F %T')] starting: 01_can_bringup"
sudo "${CAN_BRINGUP}" >"${LOG_DIR}/01_can_bringup.log" 2>&1
sleep 3

# 2) Fake memory battery script (system python3, no venv)
run_bg "02_fake_memory_battery" python3 "${BATTERY_SCRIPT}"
sleep 3

# 3) Main bringup sequence launch file (handles its own internal timing for
#    sim_headless/sbus/robot/light/fire/mode/diagnostics/coordinator)
run_bg "03_bringup_sequence_launch" ros2 launch ros2_canbus bringup_sequence.launch.py
sleep 3

# 4) Audio mic duplex (must run inside its dedicated venv)
run_bg "04_audio_mic_duplex" "${AUDIO_VENV_PY}" "${AUDIO_SCRIPT}"
sleep 20

# 5) Bag recording
cd "${RECORD_DIR}" || echo "[$(date '+%F %T')] WARNING: cannot cd to ${RECORD_DIR}" >&2
run_bg "05_record_bags" "${RECORD_BAGS_SCRIPT}"

echo "[$(date '+%F %T')] full stack launched."

# Keep the script (and therefore the service) alive while children run
wait
