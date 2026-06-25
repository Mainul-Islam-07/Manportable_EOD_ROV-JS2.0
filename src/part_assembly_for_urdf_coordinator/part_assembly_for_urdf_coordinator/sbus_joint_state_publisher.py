#!/usr/bin/env python3
"""
sbus_joint_state_publisher.py

Publishes a standard sensor_msgs/JointState for three joints whose state
is taken directly from /sbus/control rather than from ros2_control:

  - left_drive_Joint   (velocity mode — drive wheel speed command)
  - right_drive_Joint  (velocity mode — drive wheel speed command)
  - gripper_Joint      (torque mode    — claw effort command)

The output topic defaults to /sbus/joint_states (separate from /joint_states
so it doesn't fight the joint_state_broadcaster output).  Anything that
wants combined state can subscribe to both topics or merge via
joint_state_publisher.

Field conventions
-----------------
  drive joints
    position : 0.0 (no encoder; we don't fabricate a position)
    velocity : passthrough of SBus drive_{left,right} * drive_velocity_scale
               default scale = 1.0 so the field carries the raw SBus value
               ("signed % of drive_speed" per SbusControl.msg).
               Set drive_velocity_scale to (max_rad_per_s / 100.0) if you
               want proper rad/s.
    effort   : 0.0

  gripper joint
    position : 0.0 (no encoder)
    velocity : 0.0
    effort   : passthrough of SBus claw_cmd * gripper_effort_scale
               default scale = 1.0 so the field carries -100..+100 directly.
               Set gripper_effort_scale to (max_torque_Nm / 100.0) for Nm.

Watchdog
--------
If no SBus message arrives within `watchdog_timeout` seconds, the
publisher emits a JointState with all-zero velocity/effort.  Downstream
sees safe-stop state instead of stale demand.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from sbus_interfaces.msg import SbusControl


JOINT_NAMES = ['left_drive_Joint', 'right_drive_Joint', 'gripper_Joint']

# Index conventions within the JointState arrays (kept fixed so consumers
# can rely on positional access if they prefer it to name lookup).
IDX_LEFT_DRIVE  = 0
IDX_RIGHT_DRIVE = 1
IDX_GRIPPER     = 2


class SbusJointStatePublisher(Node):
    """Echoes SBus drive/gripper commands as a JointState message."""

    def __init__(self):
        super().__init__('sbus_joint_state_publisher')

        # ── parameters ────────────────────────────────────────────────────
        self.declare_parameter('publish_rate',          50.0)   # Hz
        self.declare_parameter('watchdog_timeout',      0.5)    # seconds
        self.declare_parameter('topic',                 '/sbus/joint_states')
        self.declare_parameter('sbus_topic',            '/sbus/control')
        self.declare_parameter('drive_velocity_scale',  1.0)
        self.declare_parameter('gripper_effort_scale',  1.0)

        publish_rate           = self.get_parameter('publish_rate').value
        self._watchdog_timeout = self.get_parameter('watchdog_timeout').value
        topic                  = self.get_parameter('topic').value
        sbus_topic             = self.get_parameter('sbus_topic').value
        self._drive_scale      = self.get_parameter('drive_velocity_scale').value
        self._grip_scale       = self.get_parameter('gripper_effort_scale').value

        if publish_rate <= 0.0:
            raise ValueError(f'publish_rate must be > 0 (got {publish_rate})')

        # ── latched SBus state (last received values) ────────────────────
        self._latest_drive_left  = 0.0
        self._latest_drive_right = 0.0
        self._latest_claw        = 0.0
        self._last_sbus_time     = None   # rclpy Time of most recent SBus msg
        self._warned_no_sbus     = False  # one-shot WARN when stale

        # ── ROS interfaces ────────────────────────────────────────────────
        # Best-effort QoS for SBus (matches sbus_publisher / coordinator
        # convention) — losing a single tick is fine when we publish at
        # 50 Hz and SBus arrives at 200 Hz.
        sbus_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._sub = self.create_subscription(
            SbusControl, sbus_topic, self._on_sbus, sbus_qos)

        # JointState is consumed by RViz, MoveIt, and the coordinator;
        # they expect reliable delivery.
        self._pub = self.create_publisher(JointState, topic, 10)

        # Steady-rate timer (decoupled from SBus rate) so consumers see a
        # predictable cadence even if SBus jitter is high.
        self._timer = self.create_timer(1.0 / publish_rate, self._publish)

        self.get_logger().info(
            f'sbus_joint_state_publisher started: '
            f'joints={JOINT_NAMES} '
            f'topic={topic} rate={publish_rate} Hz '
            f'drive_scale={self._drive_scale} '
            f'gripper_scale={self._grip_scale} '
            f'watchdog={self._watchdog_timeout}s')

    # ──────────────────────────────────────────────────────────────────────
    def _on_sbus(self, msg: SbusControl) -> None:
        """Latch most recent SBus values; timer reads from these."""
        self._latest_drive_left  = float(msg.drive_left)
        self._latest_drive_right = float(msg.drive_right)
        self._latest_claw        = float(msg.claw_cmd)
        self._last_sbus_time     = self.get_clock().now()
        if self._warned_no_sbus:
            self.get_logger().info('SBus resumed; publishing live values.')
            self._warned_no_sbus = False

    # ──────────────────────────────────────────────────────────────────────
    def _publish(self) -> None:
        """Emit a JointState with the most recent SBus values, or zeros
        if SBus has gone silent for longer than watchdog_timeout."""
        now = self.get_clock().now()
        stale = self._is_stale(now)

        if stale:
            drive_l = drive_r = claw = 0.0
            if not self._warned_no_sbus:
                self.get_logger().warning(
                    'No SBus message for > %.2fs — publishing zero '
                    'velocity / effort.' % self._watchdog_timeout)
                self._warned_no_sbus = True
        else:
            drive_l = self._latest_drive_left  * self._drive_scale
            drive_r = self._latest_drive_right * self._drive_scale
            claw    = self._latest_claw        * self._grip_scale

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = list(JOINT_NAMES)

        # Position field is reserved (zeros) — these joints have no encoder.
        msg.position = [0.0] * 3

        # Drives are velocity-mode → velocity carries the command.
        # Gripper is torque-mode → effort carries the command.
        msg.velocity = [0.0] * 3
        msg.velocity[IDX_LEFT_DRIVE]  = drive_l
        msg.velocity[IDX_RIGHT_DRIVE] = drive_r

        msg.effort   = [0.0] * 3
        msg.effort[IDX_GRIPPER]       = claw

        self._pub.publish(msg)

    # ──────────────────────────────────────────────────────────────────────
    def _is_stale(self, now) -> bool:
        """True if no SBus message in the last watchdog_timeout seconds."""
        if self._last_sbus_time is None:
            return True
        age_s = (now - self._last_sbus_time).nanoseconds * 1e-9
        return age_s > self._watchdog_timeout


def main(args=None):
    rclpy.init(args=args)
    node = SbusJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
