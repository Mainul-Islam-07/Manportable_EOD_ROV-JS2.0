#!/usr/bin/env bash
#
# robot_startup.sh
# Launches the robot software stack in sequence.
#
# Sequence:
#   0) boot grace                 (BOOT_GRACE_SEC; adapters + motor controllers)
#   1) can_bringup.sh             (sudo, blocking; needs NOPASSWD sudoers)
#                                 -> CAN_SETTLE_SEC
#   2) bringup_sequence.launch.py (ros2 launch; internal staggering)
#                                 -> ROSBAG_DELAY_SEC
#   3) record_bags.sh             (rosbag; started after bringup topics are up)
#
# Audio is NOT here: audio_mic_duplex.py runs from its own audio-startup.service
# (see audio_startup.sh) so an audio failure can't affect the robot stack.
# (fake_memory_battery was removed.)
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

# Seconds to wait BEFORE touching any hardware. At boot the Pi is ready long
# before the rest of the robot is: the USB CAN adapters are still enumerating and
# the motor controllers are not powered/heartbeating yet, which produced
# "Cannot find device can_arm" and "[BRINGUP-FAULT] Drive fault (Left_Drive|
# heartbeat lost)" -> "[BRINGUP-FATAL] Cannot operate safely. Terminating."
# A manual restart always worked because by then everything had settled.
# This used to be 25 s of blind guessing; can_bringup.sh now POLLS for the CAN
# interfaces to actually appear (CAN_WAIT_SEC), so this only needs to cover the
# short gap before the USB subsystem starts enumerating at all.
BOOT_GRACE_SEC="${BOOT_GRACE_SEC:-5}"

# Seconds to wait after CAN is up before launching bringup, so the buses are
# settled and the motor controllers are answering before any node talks to them.
# Unlike BOOT_GRACE_SEC this is a real settle window (controllers need a moment
# to start heartbeating once the bus is up), so it shrinks rather than vanishes.
# Raise this if a drive/arm heartbeat fault shows up on a cold boot.
CAN_SETTLE_SEC="${CAN_SETTLE_SEC:-10}"

# How many times to retry can_bringup.sh before giving up on the whole stack.
CAN_BRINGUP_TRIES="${CAN_BRINGUP_TRIES:-3}"

# Seconds to wait after the bringup launch before starting rosbag, so the
# staggered bringup nodes (~8 x 3 s) have all created their topics first.
ROSBAG_DELAY_SEC="${ROSBAG_DELAY_SEC:-40}"

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

# 0) Short head start before touching hardware. The real waiting for the CAN
#    adapters is done by can_bringup.sh, which polls for them. See BOOT_GRACE_SEC.
echo "[$(date '+%F %T')] boot grace: waiting ${BOOT_GRACE_SEC}s before touching hardware..."
sleep "${BOOT_GRACE_SEC}"

# 1) CAN bring-up (needs root). Under systemd there is no TTY to type a password,
#    so this requires a passwordless sudoers rule for can_bringup.sh
#    (see pi5_setup_commands.txt). Blocking; the script itself waits/retries.
echo "[$(date '+%F %T')] starting: 01_can_bringup"
if ! sudo -n true 2>/dev/null; then
    echo "[$(date '+%F %T')] ERROR: passwordless sudo unavailable -> CAN bring-up will fail. " \
         "Add a NOPASSWD sudoers rule for ${CAN_BRINGUP} (see pi5_setup_commands.txt)." >&2
fi
can_rc=1
: >"${LOG_DIR}/01_can_bringup.log"          # truncate once, then append per attempt
for try in $(seq 1 "${CAN_BRINGUP_TRIES}"); do
    echo "--- attempt ${try}/${CAN_BRINGUP_TRIES} at $(date '+%F %T') ---" \
        >>"${LOG_DIR}/01_can_bringup.log"
    sudo "${CAN_BRINGUP}" >>"${LOG_DIR}/01_can_bringup.log" 2>&1
    can_rc=$?
    [ "${can_rc}" -eq 0 ] && break
    echo "[$(date '+%F %T')] WARNING: can_bringup attempt ${try}/${CAN_BRINGUP_TRIES}" \
         "exited ${can_rc} (see 01_can_bringup.log)" >&2
done

# Launching the stack against a dead CAN bus produces "[BRINGUP-FATAL] Cannot
# operate safely" from the nodes anyway, so bail out instead. Exiting non-zero
# lets robot-startup.service's Restart=on-failure retry the whole stack cleanly.
if [ "${can_rc}" -ne 0 ]; then
    echo "[$(date '+%F %T')] FATAL: CAN bring-up failed after ${CAN_BRINGUP_TRIES}" \
         "attempts. Not launching the stack; systemd will retry." >&2
    exit 1
fi
# Log resulting link state to the journal for quick diagnosis.
ip -brief link show can_drive 2>&1 | head -1
ip -brief link show can_arm   2>&1 | head -1
echo "[$(date '+%F %T')] waiting ${CAN_SETTLE_SEC}s for the CAN buses to settle before bringup..."
sleep "${CAN_SETTLE_SEC}"

# 2) Main bringup sequence launch (handles its own internal staggering for
#    sim_headless/sbus/robot/light/fire/mode/diagnostics/bms/coordinator).
run_bg "02_bringup_launch" ros2 launch ros2_canbus bringup_sequence.launch.py

# 3) rosbag recording — start only after the bringup topics are up.
echo "[$(date '+%F %T')] waiting ${ROSBAG_DELAY_SEC}s for bringup topics before rosbag..."
sleep "${ROSBAG_DELAY_SEC}"
run_bg "03_record_bags" "${RECORD_BAGS_SCRIPT}"

echo "[$(date '+%F %T')] full stack launched."

# Keep the script (and therefore the service) alive while children run
wait
