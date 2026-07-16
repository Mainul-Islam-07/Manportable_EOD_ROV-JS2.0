#!/usr/bin/env python3
"""
robot_bringup.py
================
Single-process entry point that composes:

1. **HeartbeatNode**  — owns both CAN buses, heartbeat at 1 Hz,
   diagnostics at 10 Hz.
2. **MotorController** — arm-bus command bridge.
3. **DriveController** — drive-bus command bridge.

All three nodes share the same executor so Motor_CANopen_Lib objects are
passed by reference — no inter-process serialisation needed.

Startup policy
--------------
- Any **drive** motor missing at startup → full terminate (unsafe to move).
- Any **arm** motor missing at startup → arm bus disabled, drive continues.

Runtime policy
--------------
- ``/drive_fault_stop`` received → graceful shutdown of entire process.
- ``/arm_fault_stop`` received → arm controller disarms; drive keeps running.

Usage::

    python3 robot_bringup.py
"""

import json
import os
import signal
import threading
import time
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

from ros2_canbus.can_utils import safe_disarm_all, safe_disconnect
from ros2_canbus.motor_heartbeat_node import (
    HeartbeatNode, bring_up_all, ARM_FAILSAFE_FULL_LOOPBACK)
from ros2_canbus.arm_controller import MotorController
from ros2_canbus.drive_controller import DriveController
from ros2_canbus.battery_monitor import BatteryMonitor


# =========================================================================
# Config
# =========================================================================

CONFIG_FILE = str(Path(__file__).resolve().parent / "controller_config.json")

def _load_config():
    with open(CONFIG_FILE, 'r') as f:
        return json.load(f)

CFG           = _load_config()
SETTINGS_FILE = CFG['settings_file']
IDLE_TICK_S   = float(CFG['timing']['idle_tick_s'])
GRACE_S       = float(CFG['timing']['shutdown_disarm_grace_s'])
STARTUP_BATTERY_WAIT_S = float(CFG['timing']['startup_battery_wait_s'])

TOPIC_DRIVE_FAULT_STOP = CFG['heartbeat_node']['topics']['drive_fault_stop']


# =========================================================================
# Signal handling
# =========================================================================

_shutdown_requested = False

def _request_shutdown(signum=None, frame=None):
    global _shutdown_requested
    if not _shutdown_requested:
        print("\n[BRINGUP-SIGNAL] Ctrl+C / SIGTERM received — requesting graceful shutdown...")
    _shutdown_requested = True


# =========================================================================
# main
# =========================================================================

def main():
    global _shutdown_requested

    nets = None
    arm_motors = {}
    drive_motors = {}
    executor = None
    hb_node = None
    arm_node = None
    drive_node = None
    batt_node = None
    spin_thread = None

    try:
        # -- 1. Verify xlsx exists ───────────────────────────────────────
        xlsx_path = str(Path(__file__).parent / SETTINGS_FILE)
        if not os.path.isfile(xlsx_path):
            print(f"[BRINGUP-ERROR] motor_settings.xlsx not found: {xlsx_path}")
            return

        # -- 2. ROS init (needed before any Node; moved up so the battery
        #       gate below can run and Ctrl+C stays responsive) ───────────
        print("[BRINGUP] Initializing rclpy...")
        rclpy.init()
        signal.signal(signal.SIGINT, _request_shutdown)
        signal.signal(signal.SIGTERM, _request_shutdown)

        # -- 3. Battery gate ────────────────────────────────────────────
        # The encoder memory-battery voltage MUST be known before we touch the
        # hardware.  Construct the monitor (its UDP rx thread starts at once)
        # and wait up to STARTUP_BATTERY_WAIT_S for the first datagram.  If none
        # arrives, refuse to initialize — bring_up_all() is never called, so no
        # CAN scan / motor init happens.
        batt_node = BatteryMonitor()
        print(f"[BRINGUP] Waiting up to {STARTUP_BATTERY_WAIT_S:.0f}s for the "
              f"encoder memory-battery reading (UDP :9000)...")
        deadline = time.monotonic() + STARTUP_BATTERY_WAIT_S
        while (not batt_node.has_reading()
               and not _shutdown_requested
               and time.monotonic() < deadline):
            time.sleep(0.2)

        if _shutdown_requested:
            return
        if not batt_node.has_reading():
            print(f"[BRINGUP-FATAL] No encoder memory-battery reading within "
                  f"{STARTUP_BATTERY_WAIT_S:.0f}s — refusing to initialize.")
            return
        b1_mv, b2_mv = batt_node.latest_mv()
        print(f"[BRINGUP] Memory battery OK to proceed: "
              f"BAT1(drive)={b1_mv} mV, BAT2(arm)={b2_mv} mV")

        # -- 4. Hardware bring-up (heartbeat scan + motor init) ──────────
        (nets, arm_motors, drive_motors,
         arm_cfgs, drive_cfgs,
         unavail_arm, unavail_drive) = bring_up_all(xlsx_path)

        if _shutdown_requested:
            return

        # ── STARTUP POLICY ─────────────────────────────────────────────
        # Rule 1: Any drive motor missing → full terminate
        if unavail_drive:
            print(f"[BRINGUP-FATAL] Drive motor(s) unavailable at startup: "
                  f"{list(unavail_drive.keys())}")
            print(f"[BRINGUP-FATAL] Cannot operate safely. Terminating.")
            safe_disarm_all(arm_motors, tag="ARM")
            safe_disarm_all(drive_motors, tag="DRIVE")
            time.sleep(GRACE_S)
            safe_disconnect(nets, tag="ALL")
            return

        # Rule 2: Arm motor(s) missing at startup. Behavior depends on
        # ARM_FAILSAFE_FULL_LOOPBACK (defined in motor_heartbeat_node.py):
        #   True  → FAILSAFE: ENTIRE arm bus loopback + latched, alive disarmed.
        #   False → PARTIAL: run the present arm motors, loopback the missing.
        # Drive continues normally either way.
        if unavail_arm and ARM_FAILSAFE_FULL_LOOPBACK:
            arm_bus_latched_at_startup = True
            print(f"[BRINGUP-WARN] Arm motor(s) unavailable at startup: "
                  f"{sorted(unavail_arm.keys())}")
            print(f"[BRINGUP-WARN] FAILSAFE: ENTIRE arm bus will start in loopback "
                  f"(latched until restart). Disarming any alive arm motors. "
                  f"Drive bus operates normally.")
            # Disarm every alive arm motor — none should hold torque when the
            # arm chain is incomplete.
            safe_disarm_all(arm_motors, tag="ARM-LATCH")
        elif unavail_arm:
            arm_bus_latched_at_startup = False
            print(f"[BRINGUP-WARN] Arm motor(s) unavailable at startup: "
                  f"{sorted(unavail_arm.keys())}")
            print(f"[BRINGUP-WARN] PARTIAL: running present arm motors "
                  f"{sorted(arm_motors.keys())}; loopback for missing "
                  f"{sorted(unavail_arm.keys())}. Drive bus operates normally.")
        else:
            arm_bus_latched_at_startup = False

        # -- 5. Create nodes ────────────────────────────────────────────
        #   arm_motors = only alive motors (CAN commands go here)
        #   arm_cfgs   = ALL arm motor configs (enables loopback for missing)
        hb_node    = HeartbeatNode(arm_motors=arm_motors,
                                   drive_motors=drive_motors)

        # If the arm bus is latched at startup, the controller gets NO real
        # arm motors — every arm joint runs in loopback from the first tick.
        # (The heartbeat node still holds the alive arm motors so their
        # telemetry/diagnostics keep flowing.)
        arm_real_motors = {} if arm_bus_latched_at_startup else arm_motors
        arm_node   = MotorController(real_motors=arm_real_motors,
                                     cfgs=arm_cfgs,
                                     start_latched=arm_bus_latched_at_startup)

        drive_node = DriveController(real_motors=drive_motors,
                                     cfgs=drive_cfgs)

        # batt_node was already constructed in step 3 (the startup gate) and is
        # added to the executor below alongside the other nodes.

        # -- 6. Subscribe to drive_fault_stop for process shutdown ──────
        qos = QoSProfile(depth=int(CFG['ros_queue_depth']),
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)

        def _on_drive_fault(msg):
            global _shutdown_requested
            if not _shutdown_requested:
                print(f"\n[BRINGUP-FAULT] Drive fault received ({msg.data}). "
                      f"Requesting process shutdown...")
            _shutdown_requested = True

        hb_node.create_subscription(String, TOPIC_DRIVE_FAULT_STOP,
                                    _on_drive_fault, qos)

        # -- 7. Compose in single executor ──────────────────────────────
        executor = SingleThreadedExecutor()
        executor.add_node(hb_node)
        executor.add_node(arm_node)
        executor.add_node(drive_node)
        executor.add_node(batt_node)

        spin_thread = threading.Thread(target=executor.spin,
                                       name="ros-spin-all", daemon=True)
        spin_thread.start()

        if arm_bus_latched_at_startup:
            status_arm = (f"LATCHED (entire arm loopback; "
                          f"missing at boot: {sorted(unavail_arm.keys())})")
        elif unavail_arm:
            status_arm = (f"PARTIAL ({len(arm_motors)} active: "
                          f"{sorted(arm_motors.keys())}; loopback: "
                          f"{sorted(unavail_arm.keys())})")
        else:
            status_arm = f"OK ({len(arm_motors)} active)"
        print(f"[BRINGUP] All nodes spinning.  arm={status_arm}  drive=OK")
        print(f"[BRINGUP] Ctrl+C to shut down.\n")

        # -- 8. Idle loop ───────────────────────────────────────────────
        while not _shutdown_requested:
            time.sleep(IDLE_TICK_S)

    except Exception as e:
        print(f"[BRINGUP-ERROR] Unhandled exception: {e}")

    finally:
        print("\n[BRINGUP-SHUTDOWN] Beginning graceful shutdown...")

        if executor:
            try:
                executor.shutdown()
            except Exception as e:
                print(f"[BRINGUP-SHUTDOWN] executor error: {e}")

        for node, label in [(hb_node, "heartbeat"),
                            (arm_node, "arm"), (drive_node, "drive"),
                            (batt_node, "battery")]:
            if node:
                try:
                    node.destroy_node()
                except Exception as e:
                    print(f"[BRINGUP-SHUTDOWN] {label} destroy error: {e}")

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"[BRINGUP-SHUTDOWN] rclpy error: {e}")

        if spin_thread and spin_thread.is_alive():
            spin_thread.join(timeout=1.0)

        safe_disarm_all(arm_motors, tag="ARM")
        safe_disarm_all(drive_motors, tag="DRIVE")
        time.sleep(GRACE_S)
        safe_disconnect(nets, tag="ALL")
        print("[BRINGUP-SHUTDOWN] Done. Bye.")


if __name__ == "__main__":
    main()