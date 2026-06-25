#!/usr/bin/env python3
"""
Heartbeat monitor node (standalone diagnostic, tuned for 1 Hz heartbeats).

Each motor emits a CANopen heartbeat at 1 Hz. A joint is ACTIVE if its
last heartbeat arrived within the liveness timeout, otherwise INACTIVE.
When a missing heartbeat reappears, the joint goes ACTIVE again.

This is a STANDALONE diagnostic tool.  The same liveness logic now also
lives inside the integrated ``motor_heartbeat_node.HeartbeatNode`` (which
is what ``robot_bringup.py`` runs in production).  Use this node on its
own — e.g. for bench-checking which joints are alive without bringing up
the full controller stack.

⚠️  DO NOT run this node at the same time as ``robot_bringup.py`` (or any
    other process that owns the Arm/Drive CAN buses).  This node creates
    its OWN CANopen_Network and Motor_Heartbeat objects; running two
    owners on one physical bus causes node-ID collisions and duplicated
    heartbeat subscriptions.  One bus, one owner.

Configuration is read from ``controller_config.json`` (same file the rest
of the stack uses), so bus split, CAN config, settings file, timeout and
rate stay consistent.  ROS params still override if provided.

Bus assignment (from controller_config.json arm_bus_filter / drive_bus_filter):
  Drive bus (Drive_CAN_Config.json):
      Left_Drive, Right_Drive, Front_Flipper, Rear_Flipper
  Arm bus (Arm_CAN_Config.json):
      Turret, Left_Differential, Right_Differential, Telescopic,
      Wrist, Gripper_360, Gripper

Publishes (every tick):
  /active_motors     std_msgs/String  - comma-separated names of active joints
  /inactive_motors   std_msgs/String  - comma-separated names of inactive joints

Tuning notes for a 1 Hz heartbeat:
  - hb_liveness_timeout_s 2.5 tolerates one fully missed beat (~1 s) plus
    jitter. Do not go below ~1.2 s or normal jitter causes false flaps.
  - hb_poll_rate_hz 2.0 gives ~0.5 s detection granularity; polling faster
    just republishes the same state against a 1 Hz source.
  - Keep heartbeat_timeout_ms in motor_settings.xlsx around 2500 so the
    library's internal watchdog agrees with this monitor.

NOTE: Arm_CAN_Config.json and Drive_CAN_Config.json must be placed in
      .../JS2_Motor_CANOpen_Lib_V_1_0/Network/  (the library resolves them
      via file_navigator("Network", ...)).
"""

import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.CANopen_Network.Network_Lib import CANopen_Network
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.Motor_Lib_Heartbeat import Motor_Heartbeat


_CONFIG_FILE = str(Path(__file__).resolve().parent / "controller_config.json")


def _load_config():
    with open(_CONFIG_FILE, 'r') as f:
        return json.load(f)


class HeartbeatMonitorNode(Node):

    def __init__(self):
        super().__init__("heartbeat_monitor")

        cfg     = _load_config()
        hb_cfg  = cfg['heartbeat_node']
        timing  = cfg['timing']

        # ---- parameters (config defaults, ROS-param overridable) ----
        self.declare_parameter("timeout_seconds",
                               float(timing.get('hb_liveness_timeout_s', 2.5)))
        self.declare_parameter("publish_rate_hz",
                               float(timing.get('hb_poll_rate_hz', 2.0)))
        self.declare_parameter("settings_file", cfg['settings_file'])

        self.timeout_s = float(self.get_parameter("timeout_seconds").value)
        rate_hz        = float(self.get_parameter("publish_rate_hz").value)
        settings_file  = self.get_parameter("settings_file").value

        arm_can   = hb_cfg['arm_can']
        drive_can = hb_cfg['drive_can']
        drive_names = list(hb_cfg['drive_bus_filter'])
        arm_names   = list(hb_cfg['arm_bus_filter'])

        # ---- CAN networks (config-driven role / node id) ----
        self.net = {
            "Drive": CANopen_Network(drive_can['config_name'],
                                     drive_can['master_role'],
                                     int(drive_can['master_node_id'])),
            "Arm":   CANopen_Network(arm_can['config_name'],
                                     arm_can['master_role'],
                                     int(arm_can['master_node_id'])),
        }
        for name in self.net:
            self.net[name].network_reset()
            self.net[name].network_preoperational()

        # ---- one Motor_Heartbeat per joint, on its correct bus ----
        self.heartbeat = {}
        for jn in drive_names:
            self.heartbeat[jn] = Motor_Heartbeat(jn, self.net["Drive"], settings_file)
        for jn in arm_names:
            self.heartbeat[jn] = Motor_Heartbeat(jn, self.net["Arm"], settings_file)

        # ---- publishers ----
        self.active_pub = self.create_publisher(String, "/active_motors", 10)
        self.inactive_pub = self.create_publisher(String, "/inactive_motors", 10)

        # ---- timer ----
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)

        self.get_logger().info(
            f"Heartbeat monitor up: {len(self.heartbeat)} joints, "
            f"timeout={self.timeout_s:.2f}s, rate={rate_hz:.1f}Hz")

    # ------------------------------------------------------------------
    def _is_alive(self, name):
        """Alive if the last heartbeat arrived within the timeout window."""
        last = self.heartbeat[name].heartbeat.get_status()["last_heartbeat_time"]
        if last is None:
            return False
        return (time.monotonic() - last) <= self.timeout_s

    def tick(self):
        active, inactive = [], []
        for name in self.heartbeat:
            (active if self._is_alive(name) else inactive).append(name)

        self.active_pub.publish(String(data=",".join(active)))
        self.inactive_pub.publish(String(data=",".join(inactive)))

    # ------------------------------------------------------------------
    def destroy_node(self):
        for name, hb in self.heartbeat.items():
            try:
                hb.heartbeat.stop()
            except Exception as e:
                self.get_logger().error(f"stop() failed for {name}: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatMonitorNode()

    # Let devices power up and emit a couple of heartbeats before first publish.
    # Scales with the configured timeout so retuning stays correct.
    startup_wait_s = 2.5 * node.timeout_s
    node.get_logger().info(f"Waiting {startup_wait_s:.1f}s for heartbeats to settle...")
    time.sleep(startup_wait_s)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()