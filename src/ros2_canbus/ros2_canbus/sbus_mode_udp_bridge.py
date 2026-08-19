#!/usr/bin/env python3
"""
sbus_mode_udp_bridge.py
=======================
ROS 2 node that watches the operator's mode switches on ``/sbus/control`` and
UDP-publishes the single *current mode* as a compact JSON datagram for an app
(e.g. the MK32 dashboard) to display.

Mode logic (see sbus_interfaces/msg/SbusControl.msg)
---------------------------------------------------
``operation_mode``  ARMED=0  STAIR=1  DISARMED=2
``control_mode``    ARM=0    HOME=1   DRIVE=2

  * **DISARMED**  -> mode = ``"DISARM"``  (overrides everything, fail-safe)
  * **STAIR**     -> mode = ``"STAIR"``   (operator selected Stair Mode on CH2)
  * **ARMED**     -> mode = the current ``control_mode``: ``"HOME"`` / ``"ARM"``
                     / ``"DRIVE"``

.. note::
   This node only *reports* the label.  The stair behaviour itself (stair pose,
   flipper angles, arm latch-lock) lives in ``coordinator_node``.  The
   coordinator's separate FIRING feature is unrelated to this field and is
   driven by ``/fire_mode``.

Until the first message arrives the mode defaults to ``DISARM`` (the safe
assumption).

Output (UDP JSON, sent at ``rate_hz`` to ``dest_ip:dest_port``)::

    {"seq":12,"stamp":1719500000.12,"mode":"DRIVE",
     "control_mode":2,"operation_mode":0,"rx_age_ms":40}

  * ``mode``           : "DISARM" | "STAIR" | "HOME" | "ARM" | "DRIVE"
  * ``control_mode``   : last raw control_mode int (or null before first msg)
  * ``operation_mode`` : last raw operation_mode int (or null before first msg)
  * ``rx_age_ms``      : ms since the last /sbus/control message (-1 if none yet)

Run:
    ros2 run ros2_canbus mode
    ros2 run ros2_canbus mode --ros-args -p dest_ip:=192.168.144.20 -p dest_port:=9871
"""

import json
import socket
import time

import rclpy
from rclpy.node import Node

from sbus_interfaces.msg import SbusControl


# ============================ CONFIG (defaults) ============================
DEFAULT_DEST_IP   = "192.168.144.20"   # MK32 tablet (same default as telemetry bridge)
DEFAULT_DEST_PORT = 9871               # distinct from telemetry_udp_bridge's 9870
DEFAULT_RATE_HZ   = 5.0
DEFAULT_SBUS_TOPIC = "/sbus/control"

# control_mode int -> reported mode string (used only while ARMED).
_CONTROL_TO_MODE = {
    SbusControl.CONTROL_MODE_ARM:   "ARM",
    SbusControl.CONTROL_MODE_HOME:  "HOME",
    SbusControl.CONTROL_MODE_DRIVE: "DRIVE",
}
# =========================================================================


class SbusModeUdpBridge(Node):

    def __init__(self):
        super().__init__("sbus_mode_udp_bridge")

        # -- parameters (overridable via --ros-args -p ...) --
        self.dest_ip    = self.declare_parameter("dest_ip", DEFAULT_DEST_IP).value
        self.dest_port  = int(self.declare_parameter("dest_port", DEFAULT_DEST_PORT).value)
        self.rate_hz    = float(self.declare_parameter("rate_hz", DEFAULT_RATE_HZ).value)
        self.sbus_topic = self.declare_parameter("sbus_topic", DEFAULT_SBUS_TOPIC).value

        # -- state --
        # Fail-safe default: report DISARM until a real ARMED/DISARMED is seen.
        self._mode = "DISARM"
        self._control_mode = None      # last raw control_mode int
        self._operation_mode = None    # last raw operation_mode int
        self._last_rx = None           # monotonic time of last /sbus/control msg
        self._seq = 0

        # -- UDP send socket --
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._addr = (self.dest_ip, self.dest_port)

        # -- subscribe to the operator's mode switches --
        self.create_subscription(SbusControl, self.sbus_topic, self._on_sbus, 10)

        # -- transmit the latest mode at a steady rate --
        self.create_timer(1.0 / self.rate_hz, self._send_tick)

        self.get_logger().info(
            f"sbus_mode_udp_bridge: {self.sbus_topic} -> "
            f"{self.dest_ip}:{self.dest_port} @ {self.rate_hz} Hz")

    # ----------------------------------------------------------------- sbus rx

    def _on_sbus(self, msg: SbusControl):
        """Update the held mode. DISARMED wins; STAIR reports itself; ARMED
        maps control_mode."""
        self._control_mode = int(msg.control_mode)
        self._operation_mode = int(msg.operation_mode)
        self._last_rx = time.monotonic()

        if msg.operation_mode == SbusControl.OPERATION_MODE_DISARMED:
            self._mode = "DISARM"
        elif msg.operation_mode == SbusControl.OPERATION_MODE_STAIR:
            # Label only — coordinator_node owns the actual stair behaviour.
            self._mode = "STAIR"
        elif msg.operation_mode == SbusControl.OPERATION_MODE_ARMED:
            # Unknown control_mode value -> hold last (defensive; shouldn't happen).
            self._mode = _CONTROL_TO_MODE.get(msg.control_mode, self._mode)
        # Any other value: leave self._mode unchanged.

    # ------------------------------------------------------------- publishing

    def _send_tick(self):
        if self._last_rx is None:
            rx_age_ms = -1
        else:
            rx_age_ms = int((time.monotonic() - self._last_rx) * 1000.0)

        packet = {
            "seq": self._seq,
            "stamp": time.time(),
            "mode": self._mode,
            "control_mode": self._control_mode,
            "operation_mode": self._operation_mode,
            "rx_age_ms": rx_age_ms,
        }
        self._seq += 1
        try:
            data = json.dumps(packet, separators=(",", ":")).encode("utf-8")
            self._sock.sendto(data, self._addr)
        except Exception as e:
            # Never let a send error kill the node.
            self.get_logger().warn(f"udp send failed: {e}",
                                   throttle_duration_sec=5.0)

    def destroy_node(self):
        try:
            self._sock.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SbusModeUdpBridge()
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
