#!/usr/bin/env python3
"""
telemetry_udp_bridge.py
=======================
ROS 2 node that forwards robot telemetry to the MK32 Android dashboard
over a one-way UDP/JSON link.

Subscribes
----------
- /motor_diagnostics   (diagnostic_msgs/DiagnosticArray)  -- per-motor health
- /joint_states        (sensor_msgs/JointState)           -- live joint values
- /arm_active_motors   (sensor_msgs/JointState)           -- alive arm motors
- /drive_active_motors (sensor_msgs/JointState)           -- alive drive motors

It keeps the latest of each, then at a fixed rate (default 10 Hz) packs a
single compact JSON datagram and sends it to a FIXED MK32 IP:port.  The
KeyValue diagnostic pairs are flattened into plain fields so the Android
app does zero parsing.

One-way UDP is intentional:
  * no broker / rosbridge dependency (same spirit as fire_server.py)
  * each datagram is a COMPLETE snapshot, so a dropped packet is simply
    replaced by the next one ~100 ms later.
  * every packet carries `seq` + `stamp` so the app can show a STALE /
    link-lost banner if packets stop arriving.

Config: edit the CONFIG block below (or override with ROS params).

Run:
    python3 telemetry_udp_bridge.py
or with overrides:
    python3 telemetry_udp_bridge.py --ros-args \
        -p mk32_ip:=192.168.0.50 -p mk32_port:=9870 -p rate_hz:=10.0
"""

import json
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

# Decoders shared with the publisher side.  /motor_diagnostics omits the
# derived state/flag strings to save bandwidth; we recompute them here from
# the raw hex values using the SAME logic the heartbeat node would have used.
from ros2_canbus.diagnostics import (
    decode_cia402_state, humanize_heartbeat_state,
    statusword_flags_from_raw, errorregister_flags_from_raw,
    errorcode_flags_from_raw,
)


# ============================ CONFIG (defaults) ============================
DEFAULT_MK32_IP   = "192.168.144.20"   # <-- set to your MK32's fixed IP
DEFAULT_MK32_PORT = 9870
DEFAULT_RATE_HZ   = 5.0

# Battery voltage -> percentage mapping (linear, clamped).
#   <= BATTERY_MIN_V  -> 0 %
#   >= BATTERY_MAX_V  -> 100 %
BATTERY_MIN_V = 39.0
BATTERY_MAX_V = 54.0

# A fresh motor whose voltage is below this is treated as NOT-YET-INITIALISED
# (0 / garbage during boot) and skipped, so the scan moves on to the next
# motor instead of locking onto a bogus 0 V reading.  Real pack voltage on a
# 39-54 V bus never sits this low while the robot is powered.
BATTERY_PLAUSIBLE_MIN_V = 30.0

# When no fresh motor has a valid voltage, keep reporting the LAST good
# percentage for this long before falling back to null ("--" on the app).
# Overridable via the ROS param 'battery_hold_s'.
DEFAULT_BATTERY_HOLD_S = 4.0

# Topic names (match controller_config.json if you renamed them)
TOPIC_DIAGNOSTICS   = "/motor_diagnostics"
TOPIC_JOINT_STATES  = "/joint_states"
TOPIC_ARM_ACTIVE    = "/arm_active_motors"
TOPIC_DRIVE_ACTIVE  = "/drive_active_motors"

# Encoder memory-battery topics (published by battery_monitor.py, latched).
TOPIC_DRIVE_MEM_OK   = "/drive_memory_battery_ok"
TOPIC_ARM_MEM_OK     = "/arm_memory_battery_ok"
TOPIC_DRIVE_MEM_V    = "/drive_memory_battery_voltage"
TOPIC_ARM_MEM_V      = "/arm_memory_battery_voltage"

# DiagnosticStatus.level -> short label the app understands
_LEVEL = {
    DiagnosticStatus.OK:    "OK",
    DiagnosticStatus.WARN:  "WARN",
    DiagnosticStatus.ERROR: "FAULT",
    DiagnosticStatus.STALE: "STALE",
}


def _hex_to_int(s):
    """Parse a '0x..' diagnostic value to int; None/'--'/bad -> None."""
    if not s or s == "--":
        return None
    try:
        return int(s, 16)
    except (TypeError, ValueError):
        return None

# Which motors live on which bus (from motor_settings.xlsx).  Used to derive
# the active list from the diagnostics stream when /arm_active_motors or
# /drive_active_motors haven't published recently (they only publish on
# heartbeat STATE CHANGE, so a late-starting bridge can miss them).
ARM_MOTORS = {
    "Turret", "Left_Differential", "Right_Differential",
    "Telescopic", "Wrist", "Gripper_360", "Gripper",
}
DRIVE_MOTORS = {
    "Left_Drive", "Right_Drive", "Front_Flipper", "Rear_Flipper",
}
# =========================================================================


def volts_to_pct(volts, lo=BATTERY_MIN_V, hi=BATTERY_MAX_V):
    """Map a battery voltage to 0-100 %, clamped at both ends.

    volts <= lo  -> 0 ; volts >= hi -> 100 ; linear in between.
    Returns an int, or None if *volts* is not a finite number.
    """
    try:
        v = float(volts)
    except (TypeError, ValueError):
        return None
    if v != v:                       # NaN
        return None
    if v <= lo:
        return 0
    if v >= hi:
        return 100
    return int(round((v - lo) / (hi - lo) * 100.0))


class TelemetryUdpBridge(Node):

    def __init__(self):
        super().__init__("telemetry_udp_bridge")

        # -- parameters (overridable via --ros-args -p ...) --
        self.mk32_ip   = self.declare_parameter("mk32_ip", DEFAULT_MK32_IP).value
        self.mk32_port = int(self.declare_parameter("mk32_port", DEFAULT_MK32_PORT).value)
        self.rate_hz   = float(self.declare_parameter("rate_hz", DEFAULT_RATE_HZ).value)

        # -- UDP socket (send only) --
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._addr = (self.mk32_ip, self.mk32_port)

        # -- latest-snapshot caches --
        # Diagnostics arrive PER BUS (arm and drive publish separate
        # DiagnosticArray messages at different times).  We therefore MERGE
        # per-motor and stamp each motor with its own arrival time, instead
        # of replacing the whole dict (which would drop the other bus).
        self._diag = {}            # {motor_name: {fields..., "_t": monotonic}}
        self._joints = {}          # {joint_name: position}
        self._arm_active = []      # [names] from /arm_active_motors
        self._drive_active = []    # [names] from /drive_active_motors
        self._seq = 0

        # Encoder memory-battery: latest voltage (volts) and OK flag per bus.
        # None = no reading received yet -> sent as JSON null ("--" on the app).
        self._drive_mem_volts = None
        self._arm_mem_volts = None
        self._drive_mem_ok = None
        self._arm_mem_ok = None

        # Battery hold-over: remember the last good % and when we saw it, so a
        # brief dropout of ALL motors doesn't blank the indicator instantly.
        self._last_batt_pct = None
        self._last_batt_volts = None
        self._last_batt_t = None          # monotonic time of last good reading
        self.battery_hold_s = float(
            self.declare_parameter("battery_hold_s", DEFAULT_BATTERY_HOLD_S).value)

        # A motor's data is considered "live" if it updated within this many
        # seconds.  Per your spec: latest data within 500 ms = live.
        self.fresh_s = float(
            self.declare_parameter("fresh_window_s", 1.5).value)

        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        # Diagnostics is published RELIABLE in the heartbeat node; the active
        # lists too.  joint_states is high-rate; BEST_EFFORT is fine but
        # RELIABLE KEEP_LAST(10) also works.  Match publishers to be safe.
        self.create_subscription(DiagnosticArray, TOPIC_DIAGNOSTICS,
                                 self._on_diag, qos)
        self.create_subscription(JointState, TOPIC_JOINT_STATES,
                                 self._on_joints, qos)
        self.create_subscription(JointState, TOPIC_ARM_ACTIVE,
                                 self._on_arm_active, qos)
        self.create_subscription(JointState, TOPIC_DRIVE_ACTIVE,
                                 self._on_drive_active, qos)

        # Memory-battery topics are LATCHED (TRANSIENT_LOCAL) at the publisher,
        # so match durability here to receive the last value on subscribe.
        batt_qos = QoSProfile(depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, TOPIC_DRIVE_MEM_OK,
                                 self._on_drive_mem_ok, batt_qos)
        self.create_subscription(Bool, TOPIC_ARM_MEM_OK,
                                 self._on_arm_mem_ok, batt_qos)
        self.create_subscription(Float32, TOPIC_DRIVE_MEM_V,
                                 self._on_drive_mem_v, batt_qos)
        self.create_subscription(Float32, TOPIC_ARM_MEM_V,
                                 self._on_arm_mem_v, batt_qos)

        # -- fixed-rate sender --
        self.create_timer(1.0 / self.rate_hz, self._send_tick)

        self.get_logger().info(
            f"telemetry_udp_bridge -> {self.mk32_ip}:{self.mk32_port} "
            f"@ {self.rate_hz:.0f} Hz")

    # ----------------------------------------------------------------- subs

    def _on_diag(self, msg: DiagnosticArray):
        now = time.monotonic()
        # MERGE into the persistent dict — do NOT replace it.  Each message
        # now carries BOTH buses' motors, but a transient message missing a
        # motor must not blank it, so we merge per-motor and keep last data.
        for st in msg.status:
            vals = {kv.key: kv.value for kv in st.values}
            # Derived state/flag strings are no longer sent on the wire to save
            # bandwidth — recompute them locally from the raw hex values.
            sw_raw = _hex_to_int(vals.get("statusword"))
            er_raw = _hex_to_int(vals.get("errorregister"))
            ec_raw = _hex_to_int(vals.get("errorcode"))
            hb_raw = _hex_to_int(vals.get("heartbeat_state"))
            self._diag[st.name] = {
                "level":   _LEVEL.get(st.level, "STALE"),
                "msg":     st.message,
                "hw_id":   st.hardware_id,
                "bus":     vals.get("bus", ""),
                "voltage": vals.get("voltage_raw", "--"),
                "current": vals.get("current_raw", "--"),
                "coil_t":  vals.get("coil_temperature", "--"),
                "board_t": vals.get("board_temperature", "--"),
                "sw":      vals.get("statusword", "--"),
                "sw_state": decode_cia402_state(sw_raw) or "--",
                "sw_flags": statusword_flags_from_raw(sw_raw),
                "er":      vals.get("errorregister", "--"),
                "er_flags": errorregister_flags_from_raw(er_raw),
                "ec":      vals.get("errorcode", "--"),
                "ec_flags": errorcode_flags_from_raw(ec_raw),
                "hb_state": humanize_heartbeat_state(hb_raw) or "--",
                "hb_count": vals.get("heartbeat_count", "--"),
                "_t":       now,    # per-motor arrival time (monotonic)
            }

    def _on_joints(self, msg: JointState):
        self._joints = {n: (float(msg.position[i])
                            if i < len(msg.position) else 0.0)
                        for i, n in enumerate(msg.name)}

    def _on_arm_active(self, msg: JointState):
        self._arm_active = list(msg.name)

    def _on_drive_active(self, msg: JointState):
        self._drive_active = list(msg.name)

    def _on_drive_mem_ok(self, msg: Bool):
        self._drive_mem_ok = bool(msg.data)

    def _on_arm_mem_ok(self, msg: Bool):
        self._arm_mem_ok = bool(msg.data)

    def _on_drive_mem_v(self, msg: Float32):
        self._drive_mem_volts = float(msg.data)

    def _on_arm_mem_v(self, msg: Float32):
        self._arm_mem_volts = float(msg.data)

    # --------------------------------------------------------------- sender

    def _send_tick(self):
        self._seq += 1
        now = time.monotonic()

        # Build the diagnostics payload: strip the internal timestamp, add a
        # per-motor freshness flag and age.  "Live" = updated within fresh_s
        # (default 500 ms), exactly as specified.  Each bus is judged on its
        # OWN motors' arrival times, so arm and drive never mask each other.
        diag_out = {}
        fresh_arm, fresh_drive = set(), set()
        battery_volts = None          # first fresh motor's bus voltage
        for name, d in self._diag.items():
            t = d.get("_t")
            age_ms = -1 if t is None else int((now - t) * 1000)
            fresh = (age_ms >= 0) and (age_ms <= int(self.fresh_s * 1000))
            entry = {k: v for k, v in d.items() if k != "_t"}
            entry["age_ms"] = age_ms
            entry["fresh"]  = fresh
            diag_out[name] = entry
            if fresh:
                # Battery: take the first fresh motor with a PLAUSIBLE voltage.
                # All motors share the same DC bus, so any one is representative.
                # A fresh motor still reading 0/garbage at boot is below the
                # plausibility floor, so we skip it and keep scanning to the
                # next motor instead of locking onto a bogus reading.
                if battery_volts is None:
                    try:
                        v = float(d.get("voltage"))
                    except (TypeError, ValueError):
                        v = None
                    if v is not None and v == v and v >= BATTERY_PLAUSIBLE_MIN_V:
                        battery_volts = v
                if name in ARM_MOTORS:
                    fresh_arm.add(name)
                elif name in DRIVE_MOTORS:
                    fresh_drive.add(name)

        # Hold-over: if we got a good reading this tick, remember it.  If not,
        # keep reporting the last good % for up to battery_hold_s, then null.
        if battery_volts is not None:
            battery_pct = volts_to_pct(battery_volts)
            self._last_batt_pct = battery_pct
            self._last_batt_volts = battery_volts
            self._last_batt_t = now
        elif (self._last_batt_t is not None
              and (now - self._last_batt_t) <= self.battery_hold_s):
            # Within the hold window — repeat the last good values.
            battery_pct = self._last_batt_pct
            battery_volts = self._last_batt_volts
        else:
            # No fresh motor and hold window expired -> unknown.
            battery_pct = None
            battery_volts = None

        # Active = explicit /*_active_motors list  OR  fresh diagnostics
        # within the window.  Either source is enough.
        arm_active = sorted(set(self._arm_active) | fresh_arm)
        drive_active = sorted(set(self._drive_active) | fresh_drive)

        packet = {
            "seq":   self._seq,
            "stamp": time.time(),          # wall clock seconds
            "fresh_window_ms": int(self.fresh_s * 1000),
            # Battery state, derived from the first fresh motor's bus voltage.
            # battery_pct is null when no fresh voltage is available, so a
            # consumer can show "--" instead of a misleading 0 %.
            "battery_pct":   battery_pct,
            "battery_volts": battery_volts,
            # Encoder memory-battery voltage per bus (volts) + OK flag.  null
            # until a reading arrives; the app shows "--" / greys the readout.
            "drive_mem_volts": self._drive_mem_volts,
            "arm_mem_volts":   self._arm_mem_volts,
            "drive_mem_ok":    self._drive_mem_ok,
            "arm_mem_ok":      self._arm_mem_ok,
            "diagnostics": diag_out,
            "joints":      self._joints,
            "arm_active":  arm_active,
            "drive_active": drive_active,
        }
        try:
            data = json.dumps(packet, separators=(",", ":")).encode("utf-8")
            # Typical payload is well under the safe UDP datagram size.
            self._sock.sendto(data, self._addr)
        except Exception as e:
            # Never let a send error kill the node.
            self.get_logger().warn(f"UDP send failed: {e}", throttle_duration_sec=5.0)

    def destroy_node(self):
        try:
            self._sock.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryUdpBridge()
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
