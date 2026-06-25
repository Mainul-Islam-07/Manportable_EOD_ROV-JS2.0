#!/usr/bin/env python3
"""
teleop_input_node.py  —  sbus_driver / arm_ik_control

Keyboard teleop that publishes SbusControl messages on /sbus/control,
bypassing the physical SBUS receiver so the arm simulation can be driven
from the keyboard with exactly the same IK pipeline as the real robot.

Controls
--------
  W / S      — Z up / down
  A / D      — Y left / right
  Q / E      — X forward / backward
  R / F      — Wrist pitch up / down
  Z / X      — Wrist roll left / right
  O / C      — Gripper open / close  (claw_cmd -1 / +1 each tick held)
  H          — Return to home position  (operation_mode = HOME for one cycle)
  ESC / ^C   — Quit

All arm values start at the home position.  Hold a key to move continuously;
release to hold position.  Status line is printed each tick.
"""

import math
import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from sbus_interfaces.msg import SbusControl

from arm_ik_control.workspace_limits import clamp_to_workspace

# ── Home position — must match ik_solver_node.py / sbus_publisher.py ─────────
HOME_X     =  0.0
HOME_Y     = -0.100
HOME_Z     =  0.050
HOME_PITCH =  0.0
HOME_ROLL  =  0.0

# ── Joint limits (from URDF) used to clamp accumulator values ─────────────────
PITCH_MIN  = -2.35   # Wrist_Flex_Joint lower limit
PITCH_MAX  =  0.79   # Wrist_Flex_Joint upper limit
ROLL_MIN   = -math.pi
ROLL_MAX   =  math.pi
GRIP_MIN   =  0.0
GRIP_MAX   =  1.57   # Gripper_Joint upper limit

# ── Key-to-action map — printed in the help banner ───────────────────────────
_HELP = """\
┌─────────────────────────────────────────────────┐
│           ARM KEYBOARD TELEOP                   │
│                                                 │
│  W / S   Z up / down         0.005 m / tick    │
│  A / D   Y left / right      0.005 m / tick    │
│  Q / E   X fwd  / back       0.005 m / tick    │
│  R / F   Pitch  up / down    0.05  rad / tick  │
│  Z / X   Roll   left / right 0.05  rad / tick  │
│  O / C   Gripper open/close  (incremental)     │
│  H       Go HOME                               │
│  ESC     Quit                                  │
└─────────────────────────────────────────────────┘
"""


class TeleopInputNode(Node):

    def __init__(self):
        super().__init__('teleop_input_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('pos_step',   0.005)
        self.declare_parameter('angle_step', 0.05)
        self.declare_parameter('publish_rate', 50.0)

        self._pos_step   = self.get_parameter('pos_step').value
        self._angle_step = self.get_parameter('angle_step').value
        rate             = self.get_parameter('publish_rate').value

        # ── Publisher ─────────────────────────────────────────────────────────
        self._pub = self.create_publisher(SbusControl, '/sbus/control', 10)

        # ── Arm state (absolute, metres / radians) ────────────────────────────
        self._x     = HOME_X
        self._y     = HOME_Y
        self._z     = HOME_Z
        self._pitch = HOME_PITCH
        self._roll  = HOME_ROLL

        # claw_cmd is published as a one-shot per tick: -1 open, 0 hold, +1 close
        self._claw_cmd: int = 0

        # When True, publish one HOME tick then revert to NORMAL
        self._go_home_next: bool = False

        # ── Terminal setup ────────────────────────────────────────────────────
        self._old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # ── Tick timer ────────────────────────────────────────────────────────
        self._timer = self.create_timer(1.0 / rate, self._tick)

        print(_HELP)
        self.get_logger().info(
            f'Teleop ready — publishing /sbus/control at {rate:.0f} Hz  '
            f'home=[{HOME_X}, {HOME_Y}, {HOME_Z}]')

    # ── Non-blocking key read ─────────────────────────────────────────────────

    def _get_key(self) -> str | None:
        if select.select([sys.stdin], [], [], 0.0)[0]:
            return sys.stdin.read(1)
        return None

    # ── Main tick ─────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        key = self._get_key()

        # Reset claw to HOLD each tick — only active when key is pressed
        self._claw_cmd = 0

        if key:
            k = key.lower()

            if key == '\x1b':                          # ESC
                self.get_logger().info('ESC — shutting down teleop')
                self._restore_terminal()
                raise SystemExit

            elif k == 'w': self._z     += self._pos_step
            elif k == 's': self._z     -= self._pos_step
            elif k == 'a': self._y     += self._pos_step
            elif k == 'd': self._y     -= self._pos_step
            elif k == 'q': self._x     += self._pos_step
            elif k == 'e': self._x     -= self._pos_step
            elif k == 'r': self._pitch += self._angle_step
            elif k == 'f': self._pitch -= self._angle_step
            elif k == 'z': self._roll  -= self._angle_step
            elif k == 'x': self._roll  += self._angle_step
            elif k == 'o': self._claw_cmd = -1         # open
            elif k == 'c': self._claw_cmd =  1         # close
            elif k == 'h':
                # Snap state to home and request one HOME-mode publish
                self._x     = HOME_X
                self._y     = HOME_Y
                self._z     = HOME_Z
                self._pitch = HOME_PITCH
                self._roll  = HOME_ROLL
                self._claw_cmd = 0
                self._go_home_next = True
                self.get_logger().info('H — returning to home')

        # Clamp all accumulated values to URDF limits
        self._pitch = float(max(PITCH_MIN, min(PITCH_MAX, self._pitch)))
        self._roll  = float(max(ROLL_MIN,  min(ROLL_MAX,  self._roll)))

        # Clamp position to workspace half-sphere
        self._x, self._y, self._z = clamp_to_workspace(
            self._x, self._y, self._z,
        )

        # Build and publish the SbusControl message
        msg = SbusControl()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.control_mode  = SbusControl.CONTROL_MODE_ARM

        if self._go_home_next:
            msg.operation_mode = SbusControl.OPERATION_MODE_HOME
            self._go_home_next = False
        else:
            msg.operation_mode = SbusControl.OPERATION_MODE_NORMAL

        msg.arm_x        = round(self._x,     4)
        msg.arm_y        = round(self._y,     4)
        msg.arm_z        = round(self._z,     4)
        msg.wrist_pitch  = round(self._pitch, 4)
        msg.wrist_roll   = round(self._roll,  4)
        msg.claw_cmd     = int(self._claw_cmd)

        # All drive / flipper / aux fields stay at zero (simulation only)
        msg.drive_left  = 0.0
        msg.drive_right = 0.0
        msg.drive_speed = 0

        self._pub.publish(msg)

        # Live status — overwrite the same terminal line
        op = 'HOME  ' if msg.operation_mode == SbusControl.OPERATION_MODE_HOME else 'NORMAL'
        claw_str = {-1: 'OPEN ', 0: 'HOLD ', 1: 'CLOSE'}[self._claw_cmd]
        sys.stdout.write(
            f'\r  [{op}]  '
            f'x={self._x:+.3f}  y={self._y:+.3f}  z={self._z:+.3f}  '
            f'pitch={math.degrees(self._pitch):+6.1f}°  '
            f'roll={math.degrees(self._roll):+6.1f}°  '
            f'claw={claw_str}   '
        )
        sys.stdout.flush()

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def _restore_terminal(self) -> None:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
        print()  # newline after the overwritten status line

    def destroy_node(self) -> None:
        self._restore_terminal()
        super().destroy_node()


# =============================================================================
#  Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = TeleopInputNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
