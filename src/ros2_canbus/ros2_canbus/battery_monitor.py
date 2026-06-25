#!/usr/bin/env python3
"""
battery_monitor.py
==================
ROS 2 node that watches the two **encoder memory-battery** voltages and turns
them into a safety signal for the motor controllers plus telemetry for the app.

An STM32 device (``192.168.144.51``) sends an ASCII datagram to UDP port 9000
every ~30 s::

    BAT1=3950 mV, BAT2=4012 mV\\r\\n

  * **BAT1** is the backup cell that preserves the **Drive_CAN** motors' encoder
    memory.
  * **BAT2** is the backup cell that preserves the **Arm_CAN** motors' encoder
    memory.

If either cell drops below ``LOW_VOLTAGE_THRESHOLD_MV`` the corresponding bus's
absolute-encoder positions can no longer be trusted, so that bus must be
disarmed.  This node only *reports* the situation; the arm/drive controllers
subscribe to the ``*_ok`` topics and latch their bus off when it goes low.

Published topics (all latched / TRANSIENT_LOCAL so a controller that starts
*after* the last UDP packet still receives the most recent reading)::

    /drive_memory_battery_ok        std_msgs/Bool     True  = above threshold
    /arm_memory_battery_ok          std_msgs/Bool     False = below  threshold
    /drive_memory_battery_voltage   std_msgs/Float32  volts
    /arm_memory_battery_voltage     std_msgs/Float32  volts

Nothing is published until the first valid datagram arrives, so consumers treat
"no reading yet" as OK (motors operate normally) and only latch off once a real
low reading is seen.

Run:
    python3 battery_monitor.py
or:
    ros2 run ros2_canbus battery
"""

import json
import re
import socket
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from std_msgs.msg import Bool, Float32


# ============================ CONFIG (defaults) ============================

# Tunable low-voltage cutoff.  A memory cell at or below this is "low" and its
# bus gets disarmed.  ~3.3 V on a 3.6 V encoder backup cell.  Edit here.
LOW_VOLTAGE_THRESHOLD_MV = 3300

# UDP socket the STM32 sends battery readings to.
LISTEN_HOST = "0.0.0.0"
LISTEN_PORT = 9000

# How often the latched values are (re)published, in Hz.
PUBLISH_RATE_HZ = 1.0

# Matches "BAT1=3950 mV, BAT2=4012 mV" (whitespace/case tolerant).
_BATT_RE = re.compile(r"BAT1\s*=\s*(\d+)\s*mV.*?BAT2\s*=\s*(\d+)\s*mV",
                      re.IGNORECASE | re.DOTALL)

CONFIG_FILE = str(Path(__file__).resolve().parent / "controller_config.json")


def _load_topics():
    """Read the battery_monitor topic names from controller_config.json,
    falling back to the documented defaults if the block is missing."""
    defaults = {
        "drive_battery_ok":      "/drive_memory_battery_ok",
        "arm_battery_ok":        "/arm_memory_battery_ok",
        "drive_battery_voltage": "/drive_memory_battery_voltage",
        "arm_battery_voltage":   "/arm_memory_battery_voltage",
    }
    try:
        with open(CONFIG_FILE, 'r') as f:
            cfg = json.load(f)
        topics = cfg.get("battery_monitor", {}).get("topics", {})
        defaults.update({k: v for k, v in topics.items() if k in defaults})
    except Exception:
        pass
    return defaults


TOPICS = _load_topics()
# =========================================================================


class BatteryMonitor(Node):

    def __init__(self):
        super().__init__("battery_monitor")

        # -- parameters (overridable via --ros-args -p ...) --
        self.threshold_mv = int(
            self.declare_parameter("low_voltage_threshold_mv",
                                   LOW_VOLTAGE_THRESHOLD_MV).value)
        self.listen_port = int(
            self.declare_parameter("listen_port", LISTEN_PORT).value)
        self.rate_hz = float(
            self.declare_parameter("rate_hz", PUBLISH_RATE_HZ).value)

        # -- latest reading (guarded by _lock) --
        self._lock = threading.Lock()
        self._bat1_mv = None        # Drive_CAN memory cell, millivolts
        self._bat2_mv = None        # Arm_CAN  memory cell, millivolts
        self._last_rx = None        # monotonic time of last valid datagram

        # -- latched publishers: a late subscriber still gets the last value --
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._pub_drive_ok  = self.create_publisher(Bool,    TOPICS["drive_battery_ok"], qos)
        self._pub_arm_ok    = self.create_publisher(Bool,    TOPICS["arm_battery_ok"], qos)
        self._pub_drive_v   = self.create_publisher(Float32, TOPICS["drive_battery_voltage"], qos)
        self._pub_arm_v     = self.create_publisher(Float32, TOPICS["arm_battery_voltage"], qos)

        # -- UDP receive socket + reader thread --
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((LISTEN_HOST, self.listen_port))
        self._sock.settimeout(1.0)        # so the thread can exit cleanly
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop,
                                            name="battery-udp-rx", daemon=True)
        self._rx_thread.start()

        # -- (re)publish latched values at a slow steady rate --
        self.create_timer(1.0 / self.rate_hz, self._publish_tick)

        self.get_logger().info(
            f"battery_monitor listening on UDP :{self.listen_port}  "
            f"threshold={self.threshold_mv} mV  "
            f"(BAT1->drive, BAT2->arm)")

    # ----------------------------------------------------------------- UDP rx

    def _rx_loop(self):
        while self._running:
            try:
                data, _addr = self._sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break    # socket closed during shutdown
            text = data.decode(errors="replace")
            m = _BATT_RE.search(text)
            if not m:
                self.get_logger().warn(
                    f"unparseable battery datagram: {text.strip()!r}",
                    throttle_duration_sec=30.0)
                continue
            bat1_mv, bat2_mv = int(m.group(1)), int(m.group(2))
            with self._lock:
                self._bat1_mv = bat1_mv
                self._bat2_mv = bat2_mv
                self._last_rx = time.monotonic()
            self.get_logger().info(
                f"BAT1(drive)={bat1_mv} mV  BAT2(arm)={bat2_mv} mV")

    # ------------------------------------------------------- startup-gate API

    def has_reading(self):
        """True once at least one valid datagram (both BAT1 and BAT2) has been
        received.  Used by robot_bringup to gate motor initialization."""
        with self._lock:
            return self._bat1_mv is not None and self._bat2_mv is not None

    def latest_mv(self):
        """Latest (bat1_mv, bat2_mv) reading, each None until first received."""
        with self._lock:
            return self._bat1_mv, self._bat2_mv

    # ------------------------------------------------------------- publishing

    def _publish_tick(self):
        with self._lock:
            bat1_mv, bat2_mv = self._bat1_mv, self._bat2_mv
        # No reading yet -> stay silent so consumers treat the bus as OK.
        if bat1_mv is None or bat2_mv is None:
            return
        self._pub_drive_ok.publish(Bool(data=bool(bat1_mv >= self.threshold_mv)))
        self._pub_arm_ok.publish(Bool(data=bool(bat2_mv >= self.threshold_mv)))
        self._pub_drive_v.publish(Float32(data=bat1_mv / 1000.0))
        self._pub_arm_v.publish(Float32(data=bat2_mv / 1000.0))

    def destroy_node(self):
        self._running = False
        try:
            self._sock.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
