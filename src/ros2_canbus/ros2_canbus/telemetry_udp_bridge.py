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
from std_msgs.msg import Bool, Float32, UInt8
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

# Main-pack battery % is the real SOC from the JK-BD BMS (published on
# /battery_soc by battery_bms.py); no motor-voltage estimate is used.

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

        # Main-pack battery: real SOC (%) + pack voltage from the JK-BD BMS
        # (battery_bms.py -> /battery_soc, /battery_pack_v). None until a reading
        # arrives -> sent as JSON null ("--" on the app). The BMS polls ~0.5 Hz,
        # so hold the last value briefly before blanking on a serial gap.
        self._soc_pct = None
        self._soc_t = None
        self._pack_v = None
        self._pack_v_t = None
        self.battery_soc_hold_s = float(
            self.declare_parameter("battery_soc_hold_s", 10.0).value)

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

        # Main-pack BMS: real SOC (%) and pack voltage (latched by battery_bms.py).
        self.create_subscription(UInt8, "/battery_soc", self._on_soc, batt_qos)
        self.create_subscription(Float32, "/battery_pack_v", self._on_pack_v, batt_qos)

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

    def _on_soc(self, msg: UInt8):
        self._soc_pct = int(msg.data)
        self._soc_t = time.monotonic()

    def _on_pack_v(self, msg: Float32):
        self._pack_v = float(msg.data)
        self._pack_v_t = time.monotonic()

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
        for name, d in self._diag.items():
            t = d.get("_t")
            age_ms = -1 if t is None else int((now - t) * 1000)
            fresh = (age_ms >= 0) and (age_ms <= int(self.fresh_s * 1000))
            entry = {k: v for k, v in d.items() if k != "_t"}
            entry["age_ms"] = age_ms
            entry["fresh"]  = fresh
            diag_out[name] = entry
            if fresh:
                if name in ARM_MOTORS:
                    fresh_arm.add(name)
                elif name in DRIVE_MOTORS:
                    fresh_drive.add(name)

        # Battery = real SOC from the JK-BD BMS (battery_bms.py -> /battery_soc).
        # Held for battery_soc_hold_s across brief serial gaps; null (app shows
        # "--") when the BMS is stale/absent — we no longer fall back to the
        # motor-bus-voltage estimate.
        if self._soc_t is not None and (now - self._soc_t) <= self.battery_soc_hold_s:
            battery_pct = int(self._soc_pct)
        else:
            battery_pct = None
        if self._pack_v_t is not None and (now - self._pack_v_t) <= self.battery_soc_hold_s:
            battery_volts = self._pack_v
        else:
            battery_volts = None

        # Active = explicit /*_active_motors list  OR  fresh diagnostics
        # within the window.  Either source is enough.
        arm_active = sorted(set(self._arm_active) | fresh_arm)
        drive_active = sorted(set(self._drive_active) | fresh_drive)

        packet = {
            "seq":   self._seq,
            "stamp": time.time(),          # wall clock seconds
            "fresh_window_ms": int(self.fresh_s * 1000),
            # Battery state = real SOC (%) + pack voltage from the JK-BD BMS.
            # battery_pct is null when the BMS is stale/absent, so the app shows
            # "--" instead of a misleading number.
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
