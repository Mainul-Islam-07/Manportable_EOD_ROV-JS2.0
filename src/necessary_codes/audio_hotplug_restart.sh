#!/bin/bash
#
# audio_hotplug_restart.sh
# Restart the audio-startup USER service when a USB sound card is plugged in.
#
# Called by /etc/udev/rules.d/99-audio-hotplug.rules (see pi5_setup_commands.txt).
# udev runs as root, but audio-startup is a *user* service (it needs the user
# session's PulseAudio to resample the device's native rate to 16 kHz). So we
# can't `systemctl restart` it directly — we have to enter the owning user's
# manager. This finds every lingering user (each has a /run/user/<uid> runtime
# dir) and restarts the unit for whichever one has it. That keeps it working
# regardless of the exact account name.
#
# Why restart at all: a process that started before the card was present opens
# PulseAudio's `default`, and only recovers once Pulse promotes the new card to
# default — which is the ~1 min lag. A fresh process started AFTER the card is
# up grabs it in a few seconds.
#
# Runs with --no-block so it returns immediately (udev kills slow RUN helpers).

# Fallback account, used only if no /run/user/* exists yet (see below).
RUN_USER="${RUN_USER:-jontro_soinik_2_0-2}"

restarted=0
for rt in /run/user/*; do
    [ -d "$rt" ] || continue
    uid="$(basename "$rt")"
    user="$(id -nu "$uid" 2>/dev/null)" || continue
    # Harmless for users that don't have the unit (restart just fails, ignored).
    runuser -u "$user" -- env XDG_RUNTIME_DIR="$rt" \
        systemctl --user --no-block restart audio-startup.service 2>/dev/null || true
    restarted=$((restarted + 1))
done

# At BOOT this rule fires during coldplug, which can be before the lingering
# user's /run/user/<uid> exists — the loop above then runs zero times and, with
# every path being `|| true`, used to fail completely silently. Log it, and try
# the known account's runtime dir as a fallback. (The boot case no longer
# depends on this: audio_startup.sh now polls for the cards itself.)
if [ "$restarted" -eq 0 ]; then
    rt="/run/user/$(id -u "$RUN_USER" 2>/dev/null)"
    if [ -d "$rt" ]; then
        runuser -u "$RUN_USER" -- env XDG_RUNTIME_DIR="$rt" \
            systemctl --user --no-block restart audio-startup.service 2>/dev/null || true
        logger -t audio-hotplug "no /run/user/* found; restarted via fallback $RUN_USER"
    else
        logger -t audio-hotplug \
            "USB sound card added but no user runtime dir exists yet (coldplug?) — no restart done"
    fi
fi
