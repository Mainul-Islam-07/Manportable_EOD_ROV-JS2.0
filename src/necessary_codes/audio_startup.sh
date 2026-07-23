#!/usr/bin/env bash
#
# audio_startup.sh
# Starts the full-duplex mic/speaker link with the MK32 (audio_mic_duplex.py).
#
# Run by audio-startup.service, deliberately SEPARATE from robot-startup.service:
# audio has nothing to do with CAN/MoveIt/rosbag, and a failure here must not be
# able to disturb the robot stack (or vice versa).
#
# That unit is a systemd USER service (~/.config/systemd/user/). As a system
# service PortAudio fails with "Invalid sample rate [PaErrorCode -9997]": there
# is no user audio session, so ALSA's "default" has no PulseAudio to resample
# the device's native rate down to the 16 kHz the script asks for.
#
# The audio code lives in the venv directory itself, so this just reproduces the
# manual flow that is known to work:
#     cd ~/manportable_audio_venv && source venv/bin/activate && python audio_mic_duplex.py
#
# Output goes to the journal (the unit sets StandardOutput/Error=journal):
#     journalctl -u audio-startup -f
# ---------------------------------------------------------------------------

# NOTE: no `set -u` — the venv's activate script references unset variables.

# Under the user manager HOME is already correct; the fallback keeps this script
# runnable standalone.
RUN_USER="jontro_soinik_2_0-2"
HOME_DIR="${HOME:-/home/${RUN_USER}}"

# Directory holding BOTH the venv and audio_mic_duplex.py.
AUDIO_DIR="${AUDIO_DIR:-${HOME_DIR}/manportable_audio_venv}"

# Give USB/ALSA time to enumerate the audio device before PortAudio opens it.
AUDIO_START_DELAY_SEC="${AUDIO_START_DELAY_SEC:-10}"

echo "[$(date '+%F %T')] audio_startup.sh: begin (dir=${AUDIO_DIR})"

echo "[$(date '+%F %T')] waiting ${AUDIO_START_DELAY_SEC}s for the audio device to enumerate..."
sleep "${AUDIO_START_DELAY_SEC}"

if [ ! -d "${AUDIO_DIR}" ]; then
    echo "[$(date '+%F %T')] ERROR: ${AUDIO_DIR} does not exist." >&2
    exit 1
fi

# ---- Locate the venv ------------------------------------------------------
# Match by activate script rather than guessing an interpreter name; the glob
# lets the environment folder be called anything.
ACTIVATE=""
for cand in \
    "${AUDIO_DIR}/venv/bin/activate" \
    "${AUDIO_DIR}/bin/activate" \
    "${AUDIO_DIR}"/*/bin/activate ; do
    if [ -f "${cand}" ]; then ACTIVATE="${cand}"; break; fi
done

if [ -z "${ACTIVATE}" ]; then
    echo "[$(date '+%F %T')] ERROR: no venv activate under ${AUDIO_DIR}" \
         "(tried venv/bin, bin, */bin). Contents:" >&2
    ls -1 "${AUDIO_DIR}" 2>&1 | head -20 >&2
    exit 1
fi

# ---- Locate the audio script ----------------------------------------------
AUDIO_SCRIPT="${AUDIO_DIR}/audio_mic_duplex.py"
if [ ! -f "${AUDIO_SCRIPT}" ]; then
    # Fall back to the only .py in the directory, if there is exactly one.
    for cand in "${AUDIO_DIR}"/*.py; do
        if [ -f "${cand}" ]; then AUDIO_SCRIPT="${cand}"; break; fi
    done
fi

if [ ! -f "${AUDIO_SCRIPT}" ]; then
    echo "[$(date '+%F %T')] ERROR: no audio script in ${AUDIO_DIR}" \
         "(expected audio_mic_duplex.py). Contents:" >&2
    ls -1 "${AUDIO_DIR}" 2>&1 | head -20 >&2
    exit 1
fi

echo "[$(date '+%F %T')] venv:   ${ACTIVATE}"
echo "[$(date '+%F %T')] script: ${AUDIO_SCRIPT}"

# ---- Run ------------------------------------------------------------------
cd "${AUDIO_DIR}" || exit 1
# shellcheck disable=SC1090
. "${ACTIVATE}"

# `python` (not python3): after activation a venv always provides `python`.
# `exec` so systemd supervises the python process directly — Restart=always then
# reacts to the real exit rather than to this wrapper's.
exec python "${AUDIO_SCRIPT}"
