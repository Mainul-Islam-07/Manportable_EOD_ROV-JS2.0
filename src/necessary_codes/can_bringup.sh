#!/bin/bash
# can_bringup.sh — bring up both CAN buses using persistent names
# ─────────────────────────────────────────────────────────────────
# Requires: 90-can-persistent.rules installed in /etc/udev/rules.d/
#
# Usage:  sudo ./can_bringup.sh

set -e

BITRATE=1000000

# Seconds to wait for a USB CAN adapter to enumerate as its persistent name.
# At boot the Pi is ready before the adapters have finished enumerating, so
# bringing the link up immediately fails with "Cannot find device can_arm".
# Polling here (instead of a fixed sleep in robot_startup.sh) means we wait
# exactly as long as the hardware actually needs, and no longer.
CAN_WAIT_SEC="${CAN_WAIT_SEC:-60}"

# Poll until the named interface exists, or give up after CAN_WAIT_SEC.
wait_for_iface () {
    local name="$1"
    local waited=0
    while ! ip link show "${name}" >/dev/null 2>&1; do
        if [ "${waited}" -ge "${CAN_WAIT_SEC}" ]; then
            echo "[CAN] ERROR: ${name} did not appear within ${CAN_WAIT_SEC}s." >&2
            echo "[CAN]        Check the USB CAN adapter is plugged into the expected" >&2
            echo "[CAN]        port and that 90-can-persistent.rules is installed." >&2
            return 1
        fi
        if [ "${waited}" -eq 0 ]; then
            echo "[CAN] waiting for ${name} to enumerate (up to ${CAN_WAIT_SEC}s)..."
        fi
        sleep 1
        waited=$((waited + 1))
    done
    [ "${waited}" -gt 0 ] && echo "[CAN] ${name} appeared after ${waited}s."
    return 0
}

echo "[CAN] Loading kernel modules..."
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev
sudo modprobe gs_usb
sudo modprobe peak_usb

# ── Drive bus (USB port 1-1) ──────────────────────────────────────
wait_for_iface can_drive
echo "[CAN] Bringing up can_drive..."
sudo ip link set can_drive down 2>/dev/null || true
sudo ip link set can_drive up type can bitrate $BITRATE
echo "[CAN] can_drive UP:"
ip -details link show can_drive

# ── Arm bus (USB port 3-1) ────────────────────────────────────────
wait_for_iface can_arm
echo "[CAN] Bringing up can_arm..."
sudo ip link set can_arm down 2>/dev/null || true
sudo ip link set can_arm up type can bitrate $BITRATE
echo "[CAN] can_arm UP:"
ip -details link show can_arm

echo ""
echo "[CAN] Both buses ready."
echo "  can_drive  →  USB port 1-1 (Drive: Left_Drive, Right_Drive, Front_Flipper, Rear_Flipper)"
echo "  can_arm    →  USB port 3-1 (Arm: Turret, Differentials, Telescopic, Wrist, Gripper)"
