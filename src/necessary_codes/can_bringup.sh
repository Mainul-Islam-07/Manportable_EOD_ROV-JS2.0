#!/bin/bash
# can_bringup.sh — bring up both CAN buses using persistent names
# ─────────────────────────────────────────────────────────────────
# Requires: 90-can-persistent.rules installed in /etc/udev/rules.d/
#
# Usage:  sudo ./can_bringup.sh
#
# NOTE: deliberately NOT `set -e`. Each bus is brought up independently and we
# first WAIT for udev to create the persistent interface, so a slow USB
# enumeration at boot (e.g. an extra device drawing the port-power budget) can't
# leave one bus down or abort the other. See pi5_setup_commands.txt step 11b for
# the USB current-limit fix (usb_max_current_enable=1) that is the real cure when
# CAN adapters go red at boot with the BMS FTDI TTL attached.

BITRATE=1000000
WAIT_S=15          # max seconds to wait for a persistent interface to appear
RETRIES=3          # attempts to bring an interface up once it exists

echo "[CAN] Loading kernel modules..."
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev
sudo modprobe gs_usb
sudo modprobe peak_usb

# bring_up <iface>: wait (<= WAIT_S) for the udev-named interface to exist, then
# bring it up (retried). Never aborts the caller; returns non-zero on failure.
bring_up () {
    local iface="$1" i=0 r=0
    while ! ip link show "$iface" >/dev/null 2>&1; do
        if [ "$i" -ge "$WAIT_S" ]; then
            echo "[CAN] ERROR: '$iface' never appeared after ${WAIT_S}s " \
                 "(adapter not enumerated / udev rule not matched?)" >&2
            return 1
        fi
        [ "$i" -eq 0 ] && echo "[CAN] waiting for '$iface' to enumerate..."
        sleep 1; i=$((i + 1))
    done

    while [ "$r" -lt "$RETRIES" ]; do
        sudo ip link set "$iface" down 2>/dev/null || true
        if sudo ip link set "$iface" up type can bitrate "$BITRATE"; then
            echo "[CAN] $iface UP:"
            ip -details link show "$iface"
            return 0
        fi
        r=$((r + 1))
        echo "[CAN] '$iface' up failed (attempt ${r}/${RETRIES}), retrying..." >&2
        sleep 1
    done
    echo "[CAN] ERROR: could not bring up '$iface'" >&2
    return 1
}

rc=0
echo "[CAN] Bringing up can_drive..."
bring_up can_drive || rc=1
echo "[CAN] Bringing up can_arm..."
bring_up can_arm || rc=1

echo ""
if [ "$rc" -eq 0 ]; then
    echo "[CAN] Both buses ready."
else
    echo "[CAN] WARNING: one or more buses did NOT come up (see errors above)." >&2
fi
echo "  can_drive  →  USB port 1-1 (Drive: Left_Drive, Right_Drive, Front_Flipper, Rear_Flipper)"
echo "  can_arm    →  USB port 3-1 (Arm: Turret, Differentials, Telescopic, Wrist, Gripper)"
exit "$rc"
