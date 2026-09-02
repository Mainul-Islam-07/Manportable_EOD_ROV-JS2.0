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

# Short head start before probing. The real waiting is done by the polling
# below; this used to be a blind 10 s guess that was the whole readiness story.
AUDIO_START_DELAY_SEC="${AUDIO_START_DELAY_SEC:-2}"

# Mic and speaker are two SEPARATE USB cards (see audio_mic_hardaware_info.txt):
# card "UM02" captures, card "UACDemo" plays back. Matched by substring so a
# shifting card index between boots doesn't matter. Override if you swap either.
MIC_MATCH="${MIC_MATCH:-UM02}"
SPK_MATCH="${SPK_MATCH:-UACDemo}"

# How long to wait for those cards, and for the sound server, to show up.
AUDIO_WAIT_SEC="${AUDIO_WAIT_SEC:-60}"

log () { echo "[$(date '+%F %T')] $*"; }

echo "[$(date '+%F %T')] audio_startup.sh: begin (dir=${AUDIO_DIR})"

log "settling ${AUDIO_START_DELAY_SEC}s before probing audio..."
sleep "${AUDIO_START_DELAY_SEC}"

# ---- Wait for BOTH USB sound cards to enumerate ---------------------------
# audio_mic_duplex.py opens PulseAudio's default sink/source. If it starts
# before the cards are present, Pulse's default is some other device (HDMI,
# auto_null) — the stream opens fine and then plays into it silently forever,
# with no error and nothing to trigger the unit's Restart=always. So gate on
# the cards actually existing rather than guessing with a sleep.
wait_for_cards () {
    local waited=0 missing=""
    while : ; do
        missing=""
        grep -q "${MIC_MATCH}" /proc/asound/cards 2>/dev/null || missing="${MIC_MATCH} (mic)"
        if ! grep -q "${SPK_MATCH}" /proc/asound/cards 2>/dev/null; then
            [ -n "${missing}" ] && missing="${missing}, "
            missing="${missing}${SPK_MATCH} (speaker)"
        fi
        [ -z "${missing}" ] && break
        if [ "${waited}" -ge "${AUDIO_WAIT_SEC}" ]; then
            log "ERROR: audio card(s) never appeared within ${AUDIO_WAIT_SEC}s: ${missing}" >&2
            log "Present cards:" >&2
            cat /proc/asound/cards >&2 2>/dev/null
            return 1
        fi
        [ "${waited}" -eq 0 ] && log "waiting for USB sound cards (${missing})..."
        sleep 1
        waited=$((waited + 1))
    done
    [ "${waited}" -gt 0 ] && log "both sound cards present after ${waited}s."
    return 0
}

wait_for_cards || exit 1

# ---- Wait for the sound server --------------------------------------------
# THIS is what removes the multi-minute lag: previously nothing waited for the
# server, so the script raced Pulse's card probing and default promotion.
# `pactl` talks to PulseAudio and to PipeWire's pulse-compat layer alike.
if command -v pactl >/dev/null 2>&1; then
    waited=0
    while ! pactl info >/dev/null 2>&1; do
        if [ "${waited}" -ge "${AUDIO_WAIT_SEC}" ]; then
            log "WARNING: sound server not responding after ${AUDIO_WAIT_SEC}s;" \
                "starting anyway (device binding may be wrong)." >&2
            break
        fi
        [ "${waited}" -eq 0 ] && log "waiting for the sound server (pactl)..."
        sleep 1
        waited=$((waited + 1))
    done
    if pactl info >/dev/null 2>&1; then
        [ "${waited}" -gt 0 ] && log "sound server ready after ${waited}s."
        log "sound server: $(pactl info | sed -n 's/^Server Name: //p')"

        # ---- Pin the defaults to our two cards ----------------------------
        # Without this the default is whatever the server happened to promote,
        # which is the actual root cause of the intermittent failure.
        SRC="$(pactl list short sources 2>/dev/null \
               | grep -i "${MIC_MATCH}" | grep -vi '\.monitor' \
               | head -1 | cut -f2)"
        SNK="$(pactl list short sinks 2>/dev/null \
               | grep -i "${SPK_MATCH}" | head -1 | cut -f2)"

        if [ -n "${SRC}" ]; then
            pactl set-default-source "${SRC}" 2>/dev/null \
                && log "default source pinned: ${SRC}" \
                || log "WARNING: could not set default source ${SRC}" >&2
            # Move anything already recording onto it.
            pactl list short source-outputs 2>/dev/null | cut -f1 | while read -r i; do
                [ -n "${i}" ] && pactl move-source-output "${i}" "${SRC}" 2>/dev/null || true
            done
        else
            log "WARNING: no source matching '${MIC_MATCH}' — mic may be wrong." >&2
        fi

        if [ -n "${SNK}" ]; then
            pactl set-default-sink "${SNK}" 2>/dev/null \
                && log "default sink pinned: ${SNK}" \
                || log "WARNING: could not set default sink ${SNK}" >&2
            pactl list short sink-inputs 2>/dev/null | cut -f1 | while read -r i; do
                [ -n "${i}" ] && pactl move-sink-input "${i}" "${SNK}" 2>/dev/null || true
            done
        else
            log "WARNING: no sink matching '${SPK_MATCH}' — speaker may be wrong." >&2
        fi
    fi
else
    log "WARNING: pactl not found; cannot pin default sink/source." \
        "Install it with:  sudo apt install -y pulseaudio-utils" \
        "(it drives PipeWire too, via pipewire-pulse.)" >&2
fi

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

log "venv:   ${ACTIVATE}"
# Print the script's mtime too: this file is a COPY that lives outside the git
# repo (the repo's own copy is src/necessary_codes/audio/audio_mic_duplex.py),
# so a stale copy here is otherwise invisible. Compare against the repo file
# after editing it — see the deploy note in pi5_setup_commands.txt.
log "script: ${AUDIO_SCRIPT} (modified $(date -r "${AUDIO_SCRIPT}" '+%F %T' 2>/dev/null || echo unknown))"

# ---- Run ------------------------------------------------------------------
cd "${AUDIO_DIR}" || exit 1
# shellcheck disable=SC1090
. "${ACTIVATE}"

# `python` (not python3): after activation a venv always provides `python`.
# `exec` so systemd supervises the python process directly — Restart=always then
# reacts to the real exit rather than to this wrapper's.
exec python "${AUDIO_SCRIPT}"
