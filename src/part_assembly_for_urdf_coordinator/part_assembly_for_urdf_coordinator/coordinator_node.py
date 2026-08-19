"""
Coordinator Node for 7-DOF Hybrid Robotic Arm.

Bridges operator SBUS RC commands to MoveIt 2 motion planning and trajectory
execution.  Implements an event-driven state machine with a Single-Consumer /
Multiple-Producer (SCMP) concurrency model.

State machine:  IDLE → VALIDATE → PLANNING → EXECUTING → IDLE

The coordinator owns the integrated arm x/y/z goal.  It receives -1/0/+1
joystick deltas from the SBUS publisher and integrates them at a fixed
tick rate.  On IK or planning failure the current goal is rolled back to
the last successfully-planned goal and a "re-arm" flag blocks further
accumulation until all sticks return to center — preventing thrash when
the operator holds a stick against an unreachable position.
"""

import json
import math
import threading
import time
from datetime import datetime
from enum import Enum, auto
from queue import Empty, SimpleQueue

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, PoseStamped
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionIKRequest,
    RobotState,
    RobotTrajectory,
)
from moveit_msgs.srv import GetMotionPlan, GetStateValidity
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty as EmptyMsg
from std_msgs.msg import String, UInt8

from sbus_interfaces.msg import SbusControl
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from . import arm_ik       # closed-form 3-DOF IK for the arm
from . import fk as fk_mod # forward kinematics — used to anchor the goal
                           # accumulator to the wrist's actual achieved
                           # position after each IK execution.


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Arm joints in kinematic chain order
ARM_JOINTS = [
    'turret_Joint',
    'shoulder_Joint',
    'elbow_Joint',
    'telescope_Joint',
    'wrist_pan_Joint',
    'wrist_roll_Joint',
]

# URDF joint limits — used for random IK seed generation
JOINT_LIMITS = {
    'turret_Joint':    (-3.14,  3.14),
    'shoulder_Joint':  (-3.14,  0.0),
    'elbow_Joint':     ( 0.0,   3.14),
    'telescope_Joint': (-0.33982, 0.0),
    'wrist_pan_Joint': (-3.14,  3.14),
    'wrist_roll_Joint':(-3.14,  3.14),  # locked at 0 operationally
}

# Home position: all joints zero
HOME_JOINTS = {j: 0.0 for j in ARM_JOINTS}

# Flipper joints
FLIPPER_JOINTS = ['front_flipper_Joint', 'rear_flipper_Joint']
FLIPPER_LIMITS = {
    'front_flipper_Joint': (0.0,  6.28),
    'rear_flipper_Joint':  (-6.28, 0.0),
}

# Flipper home pose: both flipper joints at URDF zero — the same all-zeros
# reference HOME_JOINTS uses for the arm, and the pose the arm HOME fallback
# already relies on being collision-free.
HOME_FLIPPERS = {j: 0.0 for j in FLIPPER_JOINTS}

# Accumulator attribute backing each flipper joint, so the sequence runner
# can drive either joint generically.
FLIPPER_POS_ATTR = {
    'front_flipper_Joint': '_front_flipper_pos',
    'rear_flipper_Joint':  '_rear_flipper_pos',
}


class State(Enum):
    IDLE = auto()
    VALIDATE = auto()
    PLANNING = auto()
    EXECUTING = auto()
    FAULT = auto()


class EventType(Enum):
    GOAL = auto()
    RESET = auto()


# ---------------------------------------------------------------------------
# Coordinator Node
# ---------------------------------------------------------------------------

class CoordinatorNode(Node):

    def __init__(self):
        super().__init__('coordinator')

        # --- Parameters (all values from config — no defaults) ------------
        self.declare_parameter('planning_group')
        self.declare_parameter('planning_timeout')
        self.declare_parameter('goal_deadband')
        self.declare_parameter('settle_velocity_threshold')
        self.declare_parameter('settle_timeout')
        self.declare_parameter('monitoring_rate')
        self.declare_parameter('diagnostics_rate')
        self.declare_parameter('sbus_topic')
        self.declare_parameter('fire_mode_topic')
        self.declare_parameter('joint_state_topic')
        self.declare_parameter('home_position')
        self.declare_parameter('end_effector_link')
        self.declare_parameter('flipper_step')
        self.declare_parameter('flipper_rate')
        self.declare_parameter('flipper_collision_check')
        self.declare_parameter('flipper_home_enable')
        self.declare_parameter('flipper_home_step')
        self.declare_parameter('flipper_home_tolerance')
        self.declare_parameter('flipper_home_timeout')
        self.declare_parameter('stair_flipper_front')
        self.declare_parameter('stair_flipper_rear')
        self.declare_parameter('stair_arm_joints')
        self.declare_parameter('stair_arm_timeout')
        self.declare_parameter('stair_freeze_gripper')
        self.declare_parameter('arm_step')
        self.declare_parameter('arm_tick_rate')
        self.declare_parameter('pitch_deadband')
        self.declare_parameter('pitch_step')
        self.declare_parameter('pitch_limit')
        self.declare_parameter('roll_deadband')
        self.declare_parameter('roll_step')
        self.declare_parameter('roll_limit')
        self.declare_parameter('telescope_step')
        self.declare_parameter('telescope_deadband')
        self.declare_parameter('max_ahead_factor')
        self.declare_parameter('max_joint_delta')
        self.declare_parameter('watchdog_timeout')
        self.declare_parameter('sbus_joint_state_topic')
        self.declare_parameter('sbus_joint_state_rate')
        self.declare_parameter('joint_state_timeout')
        self.declare_parameter('flipper_ahead_limit')
        self.declare_parameter('roll_ahead_limit')
        self.declare_parameter('telescope_ahead_limit')
        self.declare_parameter('goal_arrival_tolerance')
        self.declare_parameter('drive_velocity_scale')
        self.declare_parameter('gripper_effort_scale')
        self.declare_parameter('arm_joint_commands_topic')
        self.declare_parameter('arm_joint_commands_rate')
        self.declare_parameter('flipper_joint_commands_topic')
        self.declare_parameter('flipper_joint_commands_rate')
        self.declare_parameter('turret_scale_r')

        self._planning_group = self.get_parameter('planning_group').value
        self._ee_link = self.get_parameter('end_effector_link').value
        self._planning_timeout = self.get_parameter('planning_timeout').value
        self._goal_deadband = self.get_parameter('goal_deadband').value
        self._diagnostics_rate = self.get_parameter('diagnostics_rate').value
        sbus_topic = self.get_parameter('sbus_topic').value
        fire_mode_topic = self.get_parameter('fire_mode_topic').value
        joint_state_topic = self.get_parameter('joint_state_topic').value
        self._flipper_step = self.get_parameter('flipper_step').value
        flipper_rate = self.get_parameter('flipper_rate').value
        self._flipper_collision_check = (
            self.get_parameter('flipper_collision_check').value)
        self._flipper_home_enable = (
            self.get_parameter('flipper_home_enable').value)
        self._flipper_home_step = self.get_parameter('flipper_home_step').value
        self._flipper_home_tol = (
            self.get_parameter('flipper_home_tolerance').value)
        self._flipper_home_timeout = (
            self.get_parameter('flipper_home_timeout').value)
        # --- STAIR mode ---------------------------------------------------
        self._stair_front = self.get_parameter('stair_flipper_front').value
        self._stair_rear = self.get_parameter('stair_flipper_rear').value
        self._stair_arm_timeout = self.get_parameter('stair_arm_timeout').value
        self._stair_freeze_gripper = (
            self.get_parameter('stair_freeze_gripper').value)
        _saj = list(self.get_parameter('stair_arm_joints').value)
        if len(_saj) != len(ARM_JOINTS):
            raise ValueError(
                f'stair_arm_joints must have {len(ARM_JOINTS)} entries in '
                f'ARM_JOINTS order {ARM_JOINTS}, got {len(_saj)}')
        self._stair_arm_joints = {j: float(v) for j, v in zip(ARM_JOINTS, _saj)}
        self._arm_step = self.get_parameter('arm_step').value
        arm_tick_rate = self.get_parameter('arm_tick_rate').value
        self._pitch_deadband = self.get_parameter('pitch_deadband').value
        self._pitch_step = self.get_parameter('pitch_step').value
        self._pitch_limit = self.get_parameter('pitch_limit').value
        self._roll_deadband = self.get_parameter('roll_deadband').value
        self._roll_step = self.get_parameter('roll_step').value
        self._roll_limit = self.get_parameter('roll_limit').value
        self._telescope_step = self.get_parameter('telescope_step').value
        self._telescope_deadband = self.get_parameter('telescope_deadband').value
        maf = self.get_parameter('max_ahead_factor').value
        # max_ahead must exceed the respective deadband, otherwise
        # the cap prevents goals from ever dispatching.
        self._pos_max_ahead = max(self._arm_step * maf,
                                  self._goal_deadband * 1.5)
        self._pitch_max_ahead = max(self._pitch_step * maf,
                                    self._pitch_deadband * 1.5)
        self._roll_max_ahead = max(self._roll_step * maf,
                                   self._roll_deadband * 1.5)
        self._tele_max_ahead = max(self._telescope_step * maf,
                                   self._telescope_deadband * 1.5)
        # Safety
        self._max_joint_delta = self.get_parameter('max_joint_delta').value
        self._turret_scale_r = self.get_parameter('turret_scale_r').value
        self._watchdog_timeout = self.get_parameter('watchdog_timeout').value
        self._js_timeout = self.get_parameter('joint_state_timeout').value
        # Physical-position caps — max distance accumulator may run ahead
        # of the physical joint position from /joint_states.
        self._flipper_ahead_limit = self.get_parameter('flipper_ahead_limit').value
        self._roll_ahead_limit = self.get_parameter('roll_ahead_limit').value
        self._telescope_ahead_limit = self.get_parameter('telescope_ahead_limit').value
        # Hardware arrival tolerance: max joint delta (rad) between the
        # physical joint state and the PREVIOUS IK target.  _last_valid
        # only advances if the hardware has reached (within this tolerance)
        # the position it was previously commanded to.  Prevents the
        # accumulator from drifting ahead of frozen or lagging motors.
        self._goal_arrival_tolerance = self.get_parameter('goal_arrival_tolerance').value
        # SBus joint state passthrough config
        self._sjs_topic         = self.get_parameter('sbus_joint_state_topic').value
        self._sjs_rate          = self.get_parameter('sbus_joint_state_rate').value
        self._sjs_drive_scale   = self.get_parameter('drive_velocity_scale').value
        self._sjs_grip_scale    = self.get_parameter('gripper_effort_scale').value
        self._ajc_topic         = self.get_parameter('arm_joint_commands_topic').value
        self._ajc_rate          = self.get_parameter('arm_joint_commands_rate').value
        self._fjc_topic         = self.get_parameter('flipper_joint_commands_topic').value
        self._fjc_rate          = self.get_parameter('flipper_joint_commands_rate').value
        hp = self.get_parameter('home_position').value
        self._home_pos = (hp[0], hp[1], hp[2])

        # --- Shared state (protected by locks) ----------------------------
        self._state = State.IDLE
        self._state_lock = threading.Lock()

        self._last_joint_state = None          # latest JointState msg
        self._js_lock = threading.Lock()
        # Finite-difference velocity tracking.  The hardware state interface
        # advertises 'velocity' but actually publishes zeros, so we compute
        # joint velocity from the position stream ourselves.  Used to set
        # p0.velocities in the trajectory builder so the controller doesn't
        # reset commanded velocity to zero on every preempt.
        self._joint_velocities = {}            # jname -> float (m/s or rad/s)
        self._js_prev_pos = {}                 # jname -> previous position
        self._js_prev_time = None              # previous timestamp (seconds)

        self._pending_goal = None              # latest (x, y, z) or 'HOME'
        self._goal_lock = threading.Lock()

        self._last_accepted_goal = None        # for deadband filter
        self._at_home = False                  # don't assume — first HOME will verify
        self._last_error = ''
        self._planning_attempts = 0
        self._last_goal_display = None         # for diagnostics
        self._last_goal_time = ''

        # --- Arm integration state ----------------------------------------
        # The coordinator owns the integrated Cartesian goal.  SBUS callbacks
        # just store the latest -1/0/+1 deltas atomically; the arm tick timer
        # reads them, accumulates into _current_goal, deadband-filters, and
        # enqueues a GOAL event.  On IK/plan failure, _current_goal is rolled
        # back to _last_valid_goal and _rearm_required blocks further
        # accumulation until all sticks return to center.
        #
        # NOTE: these are placeholder values used until the first joint state
        # arrives.  _on_joint_state runs a one-shot sync that re-anchors all
        # of last_valid_goal / current_goal / last_valid_pitch / roll /
        # telescope to the FK of the actual joint state — so if the arm was
        # left at some non-home pose between coordinator restarts, the first
        # user command no longer rejects with "delta exceeded" against a
        # stale home assumption.  See _sync_state_from_joints below.
        self._current_goal = self._home_pos
        self._last_valid_goal = self._home_pos
        self._initial_state_synced = False
        self._arm_x_cmd = 0
        self._arm_y_cmd = 0
        self._arm_z_cmd = 0
        self._ee_pitch_cmd = 0            # latest -1/0/+1 delta from SBUS
        self._ee_roll_cmd = 0             # latest -1/0/+1 delta from SBUS
        self._telescope_cmd = 0           # latest -1/0/+1 delta from SBUS
        self._arm_cmd_lock = threading.Lock()
        self._rearm_required = True            # block accumulation at startup
        self._prev_active_inputs = set()       # per-input tracking for stop-on-release
        self._pipeline_stale = False           # set on release, blocks execution of in-flight goals
        self._last_rollback_reason = ''
        self._last_rollback_time = ''

        # Pitch accumulation (like position, coordinator owns it)
        self._current_pitch = 0.0          # accumulated ee_pitch (radians)
        self._last_valid_pitch = 0.0       # rollback target
        self._last_dispatched_pitch = 0.0  # for pitch deadband
        self._pending_pitch = 0.0          # captured with pending_goal

        # Roll accumulation (same pattern as pitch)
        self._current_roll = 0.0           # accumulated ee_roll (radians)
        self._last_valid_roll = 0.0        # rollback target
        self._last_dispatched_roll = 0.0   # for roll deadband
        self._pending_roll = 0.0           # captured with pending_goal

        # Telescope accumulation (metres, URDF limits: -0.33982 to 0)
        self._current_telescope = 0.0      # accumulated telescope (metres)
        self._last_valid_telescope = 0.0   # rollback target
        self._last_dispatched_telescope = 0.0
        self._pending_telescope = 0.0

        # FIRING mode state (triggered by /fire_mode topic)
        self._firing_active = False        # True while in firing control loop
        self._firing_pending = False       # True between FIRING HOME and activation
        self._firing_wrist_pan = 0.0       # accumulated wrist_pan target
        self._firing_tick_count = 0        # rate limiter
        self._firing_send_interval = 5     # send every Nth tick (4 Hz at 20 Hz)

        # --- Watchdog (RC link loss detection) ----------------------------
        self._last_sbus_time = time.monotonic()
        self._watchdog_triggered = False

        # --- Joint-state feedback watchdog --------------------------------
        # Tracks wall-clock time of the last /joint_states message.  If the
        # topic goes silent (broadcaster crash, hardware disconnect), all
        # safety systems that depend on joint feedback degrade silently.
        # The watchdog blocks arm and flipper motion until feedback returns.
        self._last_js_wall_time = None   # None = no message received yet
        self._js_watchdog_triggered = False

        # --- SBus joint state passthrough state ---------------------------
        # Latest values captured in _on_sbus; published by _publish_sbus_joint_state.
        # No lock needed — single floats, GIL-atomic, telemetry-only.
        # NOTE: wrist_roll is read directly from _ee_roll_cmd (already a
        # latched -1/0/+1 SBus value used by the arm pipeline) so we don't
        # need a separate _sjs_roll_cmd field.
        self._sjs_drive_left  = 0.0
        self._sjs_drive_right = 0.0
        self._sjs_claw_cmd    = 0.0

        # --- Arm joint command stream state --------------------------------
        # Latest 6-arm-joint target as a dict {joint_name -> position}.
        # Updated in two places:
        #   (1) initial state sync (sets target = measured joints, so the
        #       publisher emits "hold current position" from message 1).
        #   (2) every successful IK execution (sets target = IK solution).
        # The timer in _publish_arm_joint_commands emits this at the
        # configured rate, repeating the last value between IK ticks.  The
        # last-known target is held forever — never zeroed on watchdog
        # (zero positions would collapse the arm).  Hardware should look
        # at the message stamp to detect coordinator silence.
        self._latest_arm_joint_target = None

        # --- Flipper joint command stream state ----------------------------
        # Same pattern as the arm topic: a dict {joint_name -> position}
        # holding the latest commanded flipper targets.  Updated on initial
        # state sync (from measured joints) and on every flipper trajectory
        # dispatch.  Held forever between dispatches.  Hardware bridge
        # subscribes if it wants to drive flippers without implementing the
        # flipper_controller action server.
        self._latest_flipper_joint_target = None

        # --- HOME override (highest priority) -----------------------------
        self._home_requested = threading.Event()   # set by _on_sbus on HOME
        self._current_arm_goal_handle = None        # active arm trajectory handle
        self._arm_handle_lock = threading.Lock()

        # --- Flipper state ------------------------------------------------
        self._front_flipper_cmd = 0              # latest -1/0/+1 from SBUS
        self._rear_flipper_cmd = 0
        self._flipper_cmd_lock = threading.Lock()
        self._front_flipper_pos = 0.0            # accumulated position (rad)
        self._rear_flipper_pos = 0.0
        self._flipper_tick_count = 0             # rate limiter
        self._flipper_send_interval = 5          # send every Nth tick (4 Hz at 20 Hz)
        self._last_flipper_collision_log = 0.0   # for log throttling
        # Previous tick's flipper cmd values, used to detect a fresh
        # command (start from rest, or a direction reversal) on either
        # side — those transitions trigger an accumulator snap to actual
        # so the trajectory we send isn't anchored to a stale setpoint.
        self._prev_front_flipper_cmd = 0
        self._prev_rear_flipper_cmd = 0
        # Single-flight guard: ReentrantCallbackGroup means flipper_tick
        # can be dispatched concurrently when a previous call is blocked
        # on the validity service.  Only one tick may run the collision
        # check at a time; others early-return without committing.
        self._flipper_check_busy = threading.Lock()

        # --- Flipper sequence runner --------------------------------------
        # Drives the flippers to a target pose, stepped by _flipper_tick so
        # it runs in PARALLEL with whatever the worker thread is doing to
        # the arm.  `groups` is a list of joint-name tuples processed in
        # order; joints inside one group move together.  HOME uses
        # [(front,), (rear,)] — front to zero, then rear.  STAIR uses
        # [(rear,), (front,)] — rear to 35 deg, then front (reverse order,
        # so only one end lifts at a time).  Label is None when no
        # sequence is running.
        self._flipper_seq_label = None         # None | 'HOME' | 'STAIR'
        self._flipper_seq_groups = []          # [(joint, ...), ...]
        self._flipper_seq_targets = {}         # joint -> target position
        self._flipper_seq_index = 0            # active group
        self._flipper_seq_deadline = 0.0       # monotonic deadline, this group
        self._flipper_seq_last_log = 0.0       # log throttle while blocked
        self._flipper_seq_final_sent = False   # final value sent, this group

        # --- STAIR mode ---------------------------------------------------
        # Triggered by the CH2 'FIRING' switch position, repurposed: the arm
        # goes to the stair pose and the flippers to +/-35 deg, then the arm
        # LATCHES locked until operation_mode returns to ARMED.  Drive and
        # flippers stay live throughout so the operator can climb and trim.
        self._stair_state = None               # None | 'MOVING' | 'HOLD'
        self._stair_arm_done = False           # arm pose command dispatched
        self._stair_deadline = 0.0             # arm-arrival deadline

        # --- Event queue (thread-safe) ------------------------------------
        self._event_queue = SimpleQueue()

        # --- Callback group (reentrant for concurrent service calls) ------
        self._cb_group = ReentrantCallbackGroup()

        # --- Subscriptions ------------------------------------------------
        self.create_subscription(
            SbusControl, sbus_topic, self._on_sbus, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            UInt8, fire_mode_topic, self._on_fire_mode, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            JointState, joint_state_topic, self._on_joint_state, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            EmptyMsg, '/coordinator/reset', self._on_reset, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            Point, '/coordinator/test_goal', self._on_test_goal, 10,
            callback_group=self._cb_group)

        # --- Publisher ----------------------------------------------------
        self._diag_pub = self.create_publisher(String, '/coordinator/diagnostics', 10)
        self.create_timer(
            1.0 / self._diagnostics_rate, self._publish_diagnostics,
            callback_group=self._cb_group)

        # SBus joint state passthrough — echoes drive_{left,right} and
        # claw_cmd as a sensor_msgs/JointState on a dedicated topic.
        # Drives in velocity[], gripper in effort[].  Read by anything that
        # wants directly-commanded-joint telemetry (RViz, rqt, logging).
        if self._sjs_rate > 0.0:
            self._sjs_pub = self.create_publisher(
                JointState, self._sjs_topic, 10)
            self.create_timer(
                1.0 / self._sjs_rate, self._publish_sbus_joint_state,
                callback_group=self._cb_group)
            self.get_logger().info(
                f'SBus joint state passthrough on {self._sjs_topic} '
                f'@ {self._sjs_rate} Hz '
                f'(drive_scale={self._sjs_drive_scale}, '
                f'gripper_scale={self._sjs_grip_scale})')
        else:
            self._sjs_pub = None
            self.get_logger().info(
                'SBus joint state passthrough disabled '
                '(sbus_joint_state_rate <= 0).')

        # Arm joint command stream — publishes the latest commanded 6 arm
        # joint positions on a dedicated topic at a steady rate.  Lets a
        # hardware bridge subscribe to a plain JointState topic instead of
        # implementing a ros2_control hardware interface plugin or writing
        # an action client.  See HARDWARE_INTEGRATION_GUIDE.md for usage.
        if self._ajc_rate > 0.0:
            self._ajc_pub = self.create_publisher(
                JointState, self._ajc_topic, 10)
            self.create_timer(
                1.0 / self._ajc_rate, self._publish_arm_joint_commands,
                callback_group=self._cb_group)
            self.get_logger().info(
                f'Arm joint command stream on {self._ajc_topic} '
                f'@ {self._ajc_rate} Hz (joints: {ARM_JOINTS}).')
        else:
            self._ajc_pub = None
            self.get_logger().info(
                'Arm joint command stream disabled '
                '(arm_joint_commands_rate <= 0).')

        # Flipper joint command stream — same idea for the 2 flipper joints.
        # Lets a hardware bridge drive flippers via topic instead of the
        # /flipper_controller/follow_joint_trajectory action.  As with the
        # arm topic, the last value is held forever between updates — the
        # consumer must run its own staleness watchdog using header.stamp.
        if self._fjc_rate > 0.0:
            self._fjc_pub = self.create_publisher(
                JointState, self._fjc_topic, 10)
            self.create_timer(
                1.0 / self._fjc_rate, self._publish_flipper_joint_commands,
                callback_group=self._cb_group)
            self.get_logger().info(
                f'Flipper joint command stream on {self._fjc_topic} '
                f'@ {self._fjc_rate} Hz (joints: {FLIPPER_JOINTS}).')
        else:
            self._fjc_pub = None
            self.get_logger().info(
                'Flipper joint command stream disabled '
                '(flipper_joint_commands_rate <= 0).')

        # --- Service clients ----------------------------------------------
        # IK is now closed-form (arm_ik module); no MoveIt /compute_ik needed.
        # /check_state_validity still used for collision; /plan_kinematic_path
        # still used for HOME (operator-driven goals use single-point trajectories).
        self._validity_client = self.create_client(
            GetStateValidity, '/check_state_validity',
            callback_group=self._cb_group)
        self._plan_client = self.create_client(
            GetMotionPlan, '/plan_kinematic_path',
            callback_group=self._cb_group)

        # --- Action clients -----------------------------------------------
        self._traj_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            callback_group=self._cb_group)
        self._flipper_client = ActionClient(
            self, FollowJointTrajectory,
            '/flipper_controller/follow_joint_trajectory',
            callback_group=self._cb_group)

        # --- Wait for services --------------------------------------------
        self.get_logger().info('Waiting for MoveIt services...')
        for client, svc_name in [
            (self._validity_client, '/check_state_validity'),
            (self._plan_client, '/plan_kinematic_path'),
        ]:
            while not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f'  waiting for {svc_name} ...')

        self.get_logger().info('Waiting for arm_controller action server...')
        while not self._traj_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info(
                '  waiting for /arm_controller/follow_joint_trajectory ...')

        self.get_logger().info('Waiting for flipper_controller action server...')
        while not self._flipper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info(
                '  waiting for /flipper_controller/follow_joint_trajectory ...')

        self.get_logger().info('All services available.  Coordinator is IDLE.')
        self.get_logger().info(
            f'Flipper collision check: '
            f'{"ENABLED (full-robot pairs)" if self._flipper_collision_check else "DISABLED"}')

        # --- Flipper timer ------------------------------------------------
        self.create_timer(
            1.0 / flipper_rate, self._flipper_tick,
            callback_group=self._cb_group)

        # --- Arm integration timer ----------------------------------------
        self.create_timer(
            1.0 / arm_tick_rate, self._arm_tick,
            callback_group=self._cb_group)

        # --- Watchdog timer (1 Hz) ----------------------------------------
        self.create_timer(1.0, self._watchdog_tick, callback_group=self._cb_group)

        # --- Worker thread ------------------------------------------------
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

    # ======================================================================
    # Utility: wait for a future without calling spin
    # ======================================================================

    def _wait_for_future(self, future, timeout_sec=5.0, interruptible=True):
        """Block until *future* completes or *timeout_sec* expires.

        Unlike rclpy.spin_until_future_complete(), this does NOT try to
        spin the node.  The MultiThreadedExecutor running in main() handles
        all callback dispatch; the worker thread only needs to poll.

        If interruptible is True (default), a HOME request will abort the
        wait and return None.  Set False for the HOME trajectory itself."""
        deadline = time.monotonic() + timeout_sec
        while not future.done():
            if interruptible and self._home_requested.is_set():
                return None                 # HOME override
            if time.monotonic() > deadline:
                return None
            time.sleep(0.01)          # yield — executor processes callbacks
        return future.result()

    # ======================================================================
    # Properties
    # ======================================================================

    @property
    def state(self):
        with self._state_lock:
            return self._state

    @state.setter
    def state(self, new_state):
        with self._state_lock:
            old = self._state
            self._state = new_state
        if old != new_state:
            self.get_logger().info(f'State: {old.name} -> {new_state.name}')

    # ======================================================================
    # Subscription callbacks (thin — producers)
    # ======================================================================

    def _on_sbus(self, msg: SbusControl):
        """Process incoming SBUS RC message.  Thin callback — stores the
        latest command deltas and handles HOME/FIRING priority.  Actual
        arm goal integration happens in _arm_tick()."""

        # --- Heartbeat: record message arrival for watchdog ---------------
        self._last_sbus_time = time.monotonic()
        if self._watchdog_triggered:
            self._watchdog_triggered = False
            self.get_logger().info('SBUS heartbeat restored — resuming.')

        # --- SBus joint state passthrough: latch drive/claw values --------
        # Single-float assignments (GIL-atomic); the timer reads them.
        # wrist_roll is intentionally NOT latched here — the timer reads
        # self._ee_roll_cmd directly, which is the same -1/0/+1 value used
        # by the arm pipeline (filtered for FIRING / DISARMED / mode gating
        # downstream in this callback).  This means the published roll
        # velocity automatically respects the same operator-mode rules as
        # the rest of the arm.
        self._sjs_drive_left  = float(msg.drive_left)
        self._sjs_drive_right = float(msg.drive_right)
        self._sjs_claw_cmd    = float(msg.claw_cmd)

        # --- STAIR mode (CH2 up) ------------------------------------------
        # Owns the whole command set while active: handles its own entry,
        # release and masking, and short-circuits the normal ARM/HOME/DRIVE
        # handling below.
        if self._update_stair_mode(msg):
            return

        # --- FIRING active: ignore control_mode, only process pitch -------
        # When firing mode is active (triggered by /fire_mode topic), zero
        # all arm commands except ee_pitch (which drives wrist_pan in
        # firing mode).  Flippers remain active.
        if self._firing_active or self._firing_pending:
            with self._flipper_cmd_lock:
                self._front_flipper_cmd = msg.front_flipper
                self._rear_flipper_cmd = msg.rear_flipper
            with self._arm_cmd_lock:
                self._arm_x_cmd = 0
                self._arm_y_cmd = 0
                self._arm_z_cmd = 0
                self._ee_pitch_cmd = msg.ee_pitch
                self._ee_roll_cmd = 0             # no roll in FIRING
                self._telescope_cmd = 0           # no telescope in FIRING
            return

        # --- Flippers: always active, regardless of control_mode ----------
        with self._flipper_cmd_lock:
            self._front_flipper_cmd = msg.front_flipper
            self._rear_flipper_cmd = msg.rear_flipper

        # --- Arm deltas + orientation + telescope: store for tick ----------
        # X-axis sign inversion: the URDF's turret_Joint has rpy="3.1416 0 0",
        # which makes URDF +X rotation produce CW motion from above.  In
        # practice the operator's "+X" stick produces motion in the URDF -X
        # direction (whether due to motor wiring, URDF authoring, or
        # operator-frame orientation — all three would manifest the same
        # way).  Inverting the joystick X here keeps the operator's "+X"
        # consistent with the direction they see the wrist physically move.
        # Y, Z, pitch, roll, telescope are unchanged.  If you fix the URDF
        # or the motor wiring later, remove the negation on _arm_x_cmd.
        if msg.control_mode == SbusControl.CONTROL_MODE_ARM:
            with self._arm_cmd_lock:
                self._arm_x_cmd = -msg.arm_x_cmd       # X-axis inverted
                self._arm_y_cmd = msg.arm_y_cmd
                self._arm_z_cmd = msg.arm_z_cmd
                self._ee_pitch_cmd = msg.ee_pitch
                self._ee_roll_cmd = msg.ee_roll
                self._telescope_cmd = msg.telescope_cmd
        else:
            with self._arm_cmd_lock:
                self._arm_x_cmd = 0
                self._arm_y_cmd = 0
                self._arm_z_cmd = 0
                self._ee_pitch_cmd = 0
                self._ee_roll_cmd = 0
                self._telescope_cmd = 0

        # --- HOME command (HIGHEST PRIORITY) ------------------------------
        if msg.control_mode == SbusControl.CONTROL_MODE_HOME:
            if self._at_home:
                return                          # dedup
            self._at_home = True

            with self._goal_lock:
                self._pending_goal = 'HOME'
            self._home_requested.set()

            with self._arm_handle_lock:
                handle = self._current_arm_goal_handle
                self._current_arm_goal_handle = None
            if handle is not None:
                try:
                    handle.cancel_goal_async()
                    self.get_logger().info('HOME: cancelled running trajectory.')
                except Exception as e:
                    self.get_logger().warn(f'HOME cancel failed: {e}')

            self._last_goal_display = {'x': self._home_pos[0],
                                        'y': self._home_pos[1],
                                        'z': self._home_pos[2]}
            self._last_goal_time = datetime.now().isoformat(timespec='seconds')
            self._event_queue.put(EventType.GOAL)
            self.get_logger().warn('HOME command accepted.')
            return

    def _update_stair_mode(self, msg: SbusControl) -> bool:
        """Handle STAIR mode entry, release and command masking.

        STAIR is the CH2-up switch position (operation_mode ==
        OPERATION_MODE_STAIR), repurposed from the old SBUS FIRING trigger
        that nothing consumed.  On entry the arm moves to the stair pose and
        the flippers to the stair angle one at a time (rear, then front),
        the two legs running in parallel; once both arrive the arm LATCHES
        locked.  Only ARMED releases it.

        Drive and flippers stay live throughout, so the operator can climb
        the stairs and trim the flippers with the arm held safely put.

        Returns True when stair mode owns the command set, in which case
        _on_sbus skips its normal handling entirely.
        """
        op = msg.operation_mode

        if (op == SbusControl.OPERATION_MODE_STAIR
                and self._stair_state is None):
            self._enter_stair_mode()
        elif (op == SbusControl.OPERATION_MODE_ARMED
                and self._stair_state is not None):
            self._exit_stair_mode()

        if self._stair_state is None:
            return False

        # ----- masking while stair mode is active --------------------------
        # Drive stays live for the whole of stair mode — the point of the
        # pose is to drive up the stairs in it.
        self._sjs_drive_left  = float(msg.drive_left)
        self._sjs_drive_right = float(msg.drive_right)
        # The gripper sits on the arm bus, so it follows the arm freeze
        # policy rather than the drive one.
        if self._stair_freeze_gripper:
            self._sjs_claw_cmd = 0.0

        # The arm is frozen for all of stair mode: while MOVING the pose
        # move owns it, and once HOLD latches the operator stays locked out
        # until operation_mode returns to ARMED.  Zeroing _ee_roll_cmd here
        # also freezes the wrist_roll passthrough on /sbus/joint_states.
        with self._arm_cmd_lock:
            self._arm_x_cmd = 0
            self._arm_y_cmd = 0
            self._arm_z_cmd = 0
            self._ee_pitch_cmd = 0
            self._ee_roll_cmd = 0
            self._telescope_cmd = 0

        # Flippers: owned by the pose sequence while MOVING, handed back to
        # the operator once the pose has latched.
        with self._flipper_cmd_lock:
            if self._stair_state == 'HOLD':
                self._front_flipper_cmd = msg.front_flipper
                self._rear_flipper_cmd = msg.rear_flipper
            else:
                self._front_flipper_cmd = 0
                self._rear_flipper_cmd = 0
        return True

    def _enter_stair_mode(self):
        """Begin the stair pose: flippers to the stair angle (rear first,
        then front, on the flipper timer) and the arm to the stair pose
        (through the worker, on the same priority path HOME uses).  The arm
        and flipper legs run in parallel with each other; _arm_tick latches
        HOLD once both have finished."""
        self._stair_state = 'MOVING'
        self._stair_arm_done = False
        self._stair_deadline = time.monotonic() + self._stair_arm_timeout
        self.get_logger().warn(
            f'STAIR mode entered: arm -> stair pose, flippers -> '
            f'front {math.degrees(self._stair_front):+.1f} deg / '
            f'rear {math.degrees(self._stair_rear):+.1f} deg. '
            f'Arm locks once the pose is reached; drive and flippers '
            f'stay live.')

        # Flipper leg — both flippers move together.
        self._start_flipper_stair()

        # Arm leg — same pre-emption path as HOME.  _home_requested aborts
        # any in-flight IK/planning; _pending_goal tells the worker which
        # priority pose to actually run (see _process_priority_goal).
        with self._goal_lock:
            self._pending_goal = 'STAIR'
        self._home_requested.set()
        with self._arm_handle_lock:
            handle = self._current_arm_goal_handle
            self._current_arm_goal_handle = None
        if handle is not None:
            try:
                handle.cancel_goal_async()
                self.get_logger().info('STAIR: cancelled running trajectory.')
            except Exception as e:
                self.get_logger().warn(f'STAIR cancel failed: {e}')
        self._last_goal_display = {'stair_pose': True}
        self._last_goal_time = datetime.now().isoformat(timespec='seconds')
        self._event_queue.put(EventType.GOAL)

    def _exit_stair_mode(self):
        """Release the stair latch (operation_mode back to ARMED).

        Nothing moves on release.  Any still-running flipper sequence is
        abandoned in place, and the arm accumulators are re-anchored to the
        ACTUAL joint positions so the operator resumes from reality rather
        than from whatever pose the coordinator last assumed.  Re-arm is
        required so a stick held through the release cannot jerk the arm.
        """
        was = self._stair_state
        self._stair_state = None
        self._stair_arm_done = False
        self._abort_flipper_sequence('STAIR released')

        with self._js_lock:
            js = self._last_joint_state
        if js is not None:
            self._initial_state_synced = False
            self._sync_state_from_joints(js)
        else:
            self.get_logger().warn(
                'STAIR release: no joint state to re-sync from; '
                'accumulators left at the stair pose.')
        self._rearm_required = True
        self._at_home = False
        self.get_logger().warn(
            f'STAIR mode released from {was} (operation_mode ARMED) — '
            f'accumulators re-synced to actual, re-arm required '
            f'(centre all sticks).')

    def _arm_at_pose(self, target, tol):
        """True when every arm joint is measured within *tol* of *target*.

        wrist_roll is excluded for the same reason the goal-arrival check
        excludes it: it is a direct operator passthrough that legitimately
        lags its commanded value.
        """
        with self._js_lock:
            js = self._last_joint_state
        if js is None:
            return False
        phys = {n: js.position[i] for i, n in enumerate(js.name)
                if n in ARM_JOINTS}
        if len(phys) != len(ARM_JOINTS):
            return False
        return all(abs(phys[j] - target[j]) <= tol
                   for j in ARM_JOINTS if j != 'wrist_roll_Joint')

    def _on_fire_mode(self, msg: UInt8):
        """Handle firing mode transitions from the /fire_mode topic.

        msg.data == 1  →  enter firing mode (HOME arm first, then activate)
        msg.data == 0  →  exit firing mode (clear state, resume normal)

        Replaces the SBUS operation_mode-based FIRING trigger.  The SBUS
        callback still provides the ee_pitch stick input that drives
        wrist_pan during firing — this callback only controls entry/exit.

        Flipper policy: firing mode homes the ARM only.  The flippers never
        move automatically on entry, during, or on exit of firing mode —
        they respond to operator stick commands and nothing else.
        """
        fire = msg.data == 1

        if fire and not self._firing_active and not self._firing_pending:
            # --- FIRING entry: HOME the arm, then enter firing control ---
            self._firing_pending = True
            self.get_logger().warn(
                'FIRING mode (via /fire_mode): homing arm first...')
            # Firing homes the ARM only.  If a flipper sequence from an
            # earlier HOME is still running, abort it here — for the whole
            # of firing mode the flippers move only on operator command.
            self._abort_flipper_sequence('FIRING mode entered')
            with self._goal_lock:
                self._pending_goal = 'HOME'
            self._home_requested.set()
            with self._arm_handle_lock:
                handle = self._current_arm_goal_handle
                self._current_arm_goal_handle = None
            if handle is not None:
                try:
                    handle.cancel_goal_async()
                except Exception:
                    pass
            self._event_queue.put(EventType.GOAL)

        elif not fire and (self._firing_active or self._firing_pending):
            # --- FIRING exit: return to normal ---
            self._firing_active = False
            self._firing_pending = False
            self._firing_wrist_pan = 0.0
            self.get_logger().info(
                'FIRING mode exited (via /fire_mode).')

    def _on_test_goal(self, msg: Point):
        """Debug absolute-goal entry point.  Bypasses integration, deadband,
        and re-arm.  Useful for `ros2 topic pub /coordinator/test_goal ...`."""
        goal = (msg.x, msg.y, msg.z)
        with self._goal_lock:
            self._pending_goal = goal
        self._last_goal_display = {
            'x': round(goal[0], 4),
            'y': round(goal[1], 4),
            'z': round(goal[2], 4),
        }
        self._last_goal_time = datetime.now().isoformat(timespec='seconds')
        self._event_queue.put(EventType.GOAL)
        self.get_logger().info(
            f'Test goal: x={goal[0]:.4f}  y={goal[1]:.4f}  z={goal[2]:.4f}')

    def _on_joint_state(self, msg: JointState):
        # Timestamp from header, falling back to wall-clock if not set
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t < 1.0:                                # missing/zero stamp
            t = self.get_clock().now().nanoseconds * 1e-9

        with self._js_lock:
            # Compute finite-difference velocity with light low-pass.  Used
            # only as a fallback because the hardware reports zero velocity.
            if self._js_prev_time is not None:
                dt = t - self._js_prev_time
                if 0.001 < dt < 0.5:               # plausible interval
                    alpha = 0.5                     # smoothing factor
                    for i, jname in enumerate(msg.name):
                        prev = self._js_prev_pos.get(jname)
                        if prev is not None:
                            v_inst = (msg.position[i] - prev) / dt
                            v_old = self._joint_velocities.get(jname, 0.0)
                            self._joint_velocities[jname] = (
                                alpha * v_inst + (1 - alpha) * v_old)
            # Update history
            for i, jname in enumerate(msg.name):
                self._js_prev_pos[jname] = msg.position[i]
            self._js_prev_time = t
            self._last_joint_state = msg

        # Record wall-clock arrival for the joint-state watchdog.
        # Outside the lock — single float assignment is GIL-atomic.
        self._last_js_wall_time = time.monotonic()
        if self._js_watchdog_triggered:
            self._js_watchdog_triggered = False
            self.get_logger().info(
                'Joint-state feedback restored — resuming motion.')

        # One-shot sync of last_valid_goal/pitch/roll/telescope on the first
        # joint state we receive.  Outside the lock — _sync_state_from_joints
        # reads msg by value and takes any locks it needs.
        if not self._initial_state_synced:
            self._sync_state_from_joints(msg)

    def _sync_state_from_joints(self, msg: JointState) -> None:
        """One-shot startup sync: anchor the operator-goal state to the
        actual arm pose so the coordinator doesn't reject the first user
        command with 'delta exceeded' against a stale home assumption.

        Called from _on_joint_state on the first message that contains all
        six arm joints.  Sets:
          _last_valid_goal / _current_goal  -> FK(joints).wrist_pan
          _last_valid_pitch / _current_pitch -> derived from joints
          _last_valid_roll  / _current_roll  -> wrist_roll
          _last_valid_telescope / _current_telescope -> telescope joint
        and flips _initial_state_synced so subsequent ticks see the
        normal FK-anchored accumulator behaviour.

        If the joint state is missing any arm joint, we skip and try
        again on the next message.
        """
        # Pull the six arm joints from the message.
        joint_pos = {}
        for i, jname in enumerate(msg.name):
            joint_pos[jname] = msg.position[i]
        required = ('turret_Joint', 'shoulder_Joint', 'elbow_Joint',
                    'telescope_Joint', 'wrist_pan_Joint', 'wrist_roll_Joint')
        missing = [j for j in required if j not in joint_pos]
        if missing:
            # Wait for a later message that includes them.  No log spam:
            # the broadcaster will deliver them within a couple of ticks.
            return

        try:
            F = fk_mod.fk(
                joint_pos['turret_Joint'],
                joint_pos['shoulder_Joint'],
                joint_pos['elbow_Joint'],
                joint_pos['telescope_Joint'],
                joint_pos['wrist_pan_Joint'],
                joint_pos['wrist_roll_Joint'],
            )
            wp = fk_mod.pos(F, 'wrist_pan')
        except Exception as e:
            self.get_logger().error(
                f'Initial-sync FK failed: {e}.  Will retry on next joint '
                'state; keeping home-position defaults for now.')
            return

        # Derived operator-frame quantities.  The IK uses
        #   wrist_pan = -(pitch_world + shoulder + elbow)
        # so the inverse is
        #   pitch_world = -(wrist_pan + shoulder + elbow)
        pitch_world = -(joint_pos['wrist_pan_Joint']
                        + joint_pos['shoulder_Joint']
                        + joint_pos['elbow_Joint'])
        roll_world  = joint_pos['wrist_roll_Joint']
        telescope_q = joint_pos['telescope_Joint']

        new_goal = (float(wp[0]), float(wp[1]), float(wp[2]))

        self._last_valid_goal      = new_goal
        self._current_goal         = new_goal
        self._last_valid_pitch     = pitch_world
        self._current_pitch        = pitch_world
        self._last_valid_roll      = roll_world
        self._current_roll         = roll_world
        self._last_valid_telescope = telescope_q
        self._current_telescope    = telescope_q
        self._initial_state_synced = True

        # NOTE: we intentionally do NOT seed _latest_arm_joint_target or
        # _latest_flipper_joint_target here.  On real hardware the
        # /joint_states source is often mock_components/GenericSystem,
        # which starts at initial_positions.yaml (all zeros).  If we
        # seeded the publisher targets from those zeros, the 50 Hz
        # /arm_joint_commands and /flipper_joint_commands publishers
        # would immediately command the real hardware to position 0 —
        # potentially moving the arm and flippers through un-checked
        # paths and causing collisions.
        #
        # Instead, both targets stay None until the first operator-
        # triggered, collision-checked dispatch:
        #   - _latest_arm_joint_target:    set on first IK success
        #   - _latest_flipper_joint_target: set on first flipper dispatch
        #
        # The publishers skip None targets, so the real hardware bridge
        # receives no commands until motion is intentional and safe.

        # Same for the flipper command stream — seed only the
        # ACCUMULATORS (for physical-cap math) but NOT the publisher
        # target.  The publisher stays silent until the first operator-
        # triggered, collision-checked flipper dispatch.
        if all(j in joint_pos for j in FLIPPER_JOINTS):
            # Seed the flipper ACCUMULATORS so the first flipper command
            # doesn't target 0.0 (the __init__ default) when the flippers
            # are at a non-zero position after a restart.  Note: on a
            # hybrid setup (mock ros2_control + real motors), these values
            # come from mock hardware and may not reflect reality.
            self._front_flipper_pos = float(
                joint_pos['front_flipper_Joint'])
            self._rear_flipper_pos = float(
                joint_pos['rear_flipper_Joint'])

        # Loud INFO so it's obvious in the logs what just happened.
        home_dx = new_goal[0] - self._home_pos[0]
        home_dy = new_goal[1] - self._home_pos[1]
        home_dz = new_goal[2] - self._home_pos[2]
        home_dist = math.sqrt(home_dx*home_dx + home_dy*home_dy + home_dz*home_dz)
        self.get_logger().info(
            f'Initial state sync from joint state: '
            f'wrist=({new_goal[0]:+.4f}, {new_goal[1]:+.4f}, {new_goal[2]:+.4f})  '
            f'pitch={pitch_world:+.3f}  roll={roll_world:+.3f}  '
            f'tele={telescope_q:+.4f}  '
            f'({home_dist*1000:.0f} mm from home).')

    def _on_reset(self, _msg: EmptyMsg):
        self.get_logger().info('Manual reset received.')
        self._event_queue.put(EventType.RESET)

    # ======================================================================
    # Diagnostics
    # ======================================================================

    def _publish_diagnostics(self):
        diag = {
            'state': self.state.name,
            'planning_attempts': self._planning_attempts,
            'last_goal': self._last_goal_display,
            'last_goal_time': self._last_goal_time,
            'last_error': self._last_error,
            'current_goal': {
                'x': round(self._current_goal[0], 4),
                'y': round(self._current_goal[1], 4),
                'z': round(self._current_goal[2], 4),
            },
            'last_valid_goal': {
                'x': round(self._last_valid_goal[0], 4),
                'y': round(self._last_valid_goal[1], 4),
                'z': round(self._last_valid_goal[2], 4),
            },
            'rearm_required': self._rearm_required,
            'last_rollback_reason': self._last_rollback_reason,
            'last_rollback_time': self._last_rollback_time,
            'at_home': self._at_home,
            'current_pitch': round(self._current_pitch, 3),
            'last_valid_pitch': round(self._last_valid_pitch, 3),
            'current_roll': round(self._current_roll, 3),
            'last_valid_roll': round(self._last_valid_roll, 3),
            'current_telescope': round(self._current_telescope, 4),
            'last_valid_telescope': round(self._last_valid_telescope, 4),
            'operation_mode': ('FIRING' if self._firing_active
                               else ('FIRING_PENDING' if self._firing_pending
                                     else 'ARMED')),
            'firing_active': self._firing_active,
            'firing_wrist_pan': round(self._firing_wrist_pan, 3),
            'watchdog_triggered': self._watchdog_triggered,
            'flipper_seq': self._flipper_seq_label,
            'stair_state': self._stair_state,
        }
        msg = String()
        msg.data = json.dumps(diag)
        self._diag_pub.publish(msg)

    # ======================================================================
    # SBus joint state passthrough
    # ======================================================================

    # Joint names and indices for the passthrough JointState message.
    # Kept as class-level constants so external consumers can refer to
    # them programmatically if needed.
    _SJS_JOINT_NAMES = ['left_drive_Joint', 'right_drive_Joint',
                        'gripper_Joint', 'wrist_roll_Joint']
    _SJS_IDX_LEFT_DRIVE  = 0
    _SJS_IDX_RIGHT_DRIVE = 1
    _SJS_IDX_GRIPPER     = 2
    _SJS_IDX_WRIST_ROLL  = 3

    def _publish_sbus_joint_state(self):
        """Emit a sensor_msgs/JointState carrying the directly-commanded
        joints (drives + gripper + wrist_roll) from the latest /sbus/control
        message.

        Field layout:
            name[]     = ['left_drive_Joint', 'right_drive_Joint',
                          'gripper_Joint',    'wrist_roll_Joint']
            position[] = [0.0, 0.0, 0.0, 0.0]            (no encoder feedback)
            velocity[] = [drive_left * scale,  drive_right * scale,
                          0.0,                 wrist_roll_cmd]
            effort[]   = [0.0, 0.0, claw_cmd * scale, 0.0]

        wrist_roll is published as a discrete -1/0/+1 from the SBus roll
        stick (read directly from self._ee_roll_cmd, the same latched value
        the arm pipeline uses, so it inherits FIRING/DISARMED/mode gating).
        Downstream consumer interprets the sign as direction and the
        magnitude as 'go at the configured fixed speed'.

        The SBUS watchdog (2 s) zeros the underlying variables on link
        loss, so the publisher automatically reflects safe-stop state
        without a separate stale-timeout mechanism.
        """
        if self._sjs_pub is None:
            return

        # Read the latest latched values.  The SBUS watchdog (2 s) zeros
        # these variables on link loss, so no separate stale-timeout is
        # needed here — the publisher always reflects the current state.
        drive_l = self._sjs_drive_left  * self._sjs_drive_scale
        drive_r = self._sjs_drive_right * self._sjs_drive_scale
        claw    = self._sjs_claw_cmd    * self._sjs_grip_scale
        # Coerce to -1 / 0 / +1 regardless of what's in _ee_roll_cmd
        # (already the case, but be defensive against future changes).
        roll_raw = self._ee_roll_cmd
        roll = 1.0 if roll_raw > 0 else (-1.0 if roll_raw < 0 else 0.0)

        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = list(self._SJS_JOINT_NAMES)
        # Position reserved (zeros) — no encoders on these joints.
        m.position = [0.0, 0.0, 0.0, 0.0]
        # Drives + wrist_roll are velocity-mode; gripper is torque-mode.
        m.velocity = [0.0, 0.0, 0.0, 0.0]
        m.velocity[self._SJS_IDX_LEFT_DRIVE]  = drive_l
        m.velocity[self._SJS_IDX_RIGHT_DRIVE] = drive_r
        m.velocity[self._SJS_IDX_WRIST_ROLL]  = roll
        m.effort   = [0.0, 0.0, 0.0, 0.0]
        m.effort[self._SJS_IDX_GRIPPER]       = claw
        self._sjs_pub.publish(m)

    # ======================================================================
    # Arm joint command stream
    # ======================================================================

    def _publish_arm_joint_commands(self):
        """Emit a sensor_msgs/JointState carrying the latest commanded arm
        joint positions.

        Field layout:
            name[]     = ARM_JOINTS  (turret, shoulder, elbow, telescope,
                                      wrist_pan, wrist_roll — fixed order)
            position[] = latest IK solution for each joint
            velocity[] = []  (empty — hardware does its own derivation)
            effort[]   = []  (empty)

        The target is set in two places:
          - _sync_state_from_joints  (seeds with measured joints on startup)
          - _process_goal IK success (updates with each new IK solution)

        Between updates the publisher repeats the last known target.  This
        is intentional: zero positions would collapse the arm.  Hardware
        should look at the message's header.stamp to detect coordinator
        silence and apply its own staleness watchdog.

        If no IK has run yet and no initial sync has occurred, the publisher
        emits nothing.  This avoids a misleading "go to home" message
        before the actual arm pose is known.
        """
        if self._ajc_pub is None:
            return
        if self._latest_arm_joint_target is None:
            return  # no command available yet

        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = list(ARM_JOINTS)
        # Look up by name — robust if ik_solution's dict ordering differs
        # from ARM_JOINTS' canonical order (it shouldn't, but be defensive).
        m.position = [float(self._latest_arm_joint_target[j]) for j in ARM_JOINTS]
        # velocity/effort left empty; hardware derives velocity from
        # successive positions if it needs to.
        m.velocity = []
        m.effort = []
        self._ajc_pub.publish(m)

    def _publish_flipper_joint_commands(self):
        """Emit a sensor_msgs/JointState carrying the latest commanded
        flipper joint positions.  Symmetric counterpart to the arm topic.

        Field layout:
            name[]     = FLIPPER_JOINTS  (front_flipper, rear_flipper — fixed order)
            position[] = latest commanded positions for each
            velocity[] = []  (empty — hardware derives if needed)
            effort[]   = []  (empty)

        The target is set in two places:
          - _sync_state_from_joints (seeds with measured joints on startup)
          - _send_flipper_trajectory (updates on every flipper dispatch)

        Between updates the publisher repeats the last known target.  As
        with the arm topic, the last value is held forever — never zeroed
        on watchdog.  Hardware must run its own staleness watchdog using
        header.stamp.

        Publisher emits nothing until either the initial sync has run or
        a flipper trajectory has been dispatched.
        """
        if self._fjc_pub is None:
            return
        if self._latest_flipper_joint_target is None:
            return  # no command available yet

        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = list(FLIPPER_JOINTS)
        m.position = [float(self._latest_flipper_joint_target[j])
                      for j in FLIPPER_JOINTS]
        m.velocity = []
        m.effort = []
        self._fjc_pub.publish(m)

    # ======================================================================
    # Watchdog (RC link loss detection)
    # ======================================================================

    def _watchdog_tick(self):
        """Called at 1 Hz.  If no SBUS message has arrived for
        watchdog_timeout seconds, cancel all motion and hold position."""
        if self._watchdog_triggered:
            return  # already in watchdog state

        # Joint-state feedback watchdog (independent of SBUS)
        self._check_joint_state_watchdog()

        elapsed = time.monotonic() - self._last_sbus_time
        if elapsed < self._watchdog_timeout:
            return

        self._watchdog_triggered = True
        self.get_logger().error(
            f'WATCHDOG: no SBUS for {elapsed:.1f}s — cancelling all motion.')

        # Cancel running arm trajectory
        with self._arm_handle_lock:
            handle = self._current_arm_goal_handle
            self._current_arm_goal_handle = None
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass

        # Zero all commands so ticks produce no motion
        with self._arm_cmd_lock:
            self._arm_x_cmd = 0
            self._arm_y_cmd = 0
            self._arm_z_cmd = 0
            self._ee_pitch_cmd = 0
            self._ee_roll_cmd = 0
            self._telescope_cmd = 0
        with self._flipper_cmd_lock:
            self._front_flipper_cmd = 0
            self._rear_flipper_cmd = 0

        # Abort any in-flight flipper HOME.  The watchdog contract is to
        # cancel all motion and hold, and the homing sequence does not read
        # the (now zeroed) operator commands, so it would otherwise keep
        # driving the flippers straight through an RC link loss.
        self._abort_flipper_sequence('SBUS watchdog')

        # Zero drive/claw passthrough so /sbus/joint_states publishes
        # safe-stop values immediately, not after the old 60 s timeout.
        self._sjs_drive_left  = 0.0
        self._sjs_drive_right = 0.0
        self._sjs_claw_cmd    = 0.0

        # Drain event queue
        with self._goal_lock:
            self._pending_goal = None
        while not self._event_queue.empty():
            try:
                self._event_queue.get_nowait()
            except Empty:
                break

        # Snap state to physical joints
        self._snap_to_current_joints()

    def _check_joint_state_watchdog(self):
        """Called from _watchdog_tick.  If /joint_states has gone silent
        longer than joint_state_timeout, block all motion and log an
        error.  Resumes automatically when messages return (handled in
        _on_joint_state)."""
        if self._last_js_wall_time is None:
            # No message ever received — too early to judge.  The
            # coordinator already blocks on services at startup, so
            # motion can't begin until joint states arrive.
            return

        elapsed = time.monotonic() - self._last_js_wall_time
        if elapsed < self._js_timeout:
            return

        if self._js_watchdog_triggered:
            return  # already in watchdog state

        self._js_watchdog_triggered = True
        self.get_logger().error(
            f'JOINT-STATE WATCHDOG: no /joint_states for {elapsed:.1f}s '
            '— blocking all motion until feedback returns.')

        # Cancel running arm trajectory
        with self._arm_handle_lock:
            handle = self._current_arm_goal_handle
            self._current_arm_goal_handle = None
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass

    # ======================================================================
    # Trajectory velocity safety check
    # ======================================================================

    # URDF velocity limits per joint (rad/s for revolute, m/s for prismatic)
    _VELOCITY_LIMITS = {
        'turret_Joint':     1.57,
        'shoulder_Joint':   1.05,
        'elbow_Joint':      1.57,
        'telescope_Joint':  0.1,
        'wrist_pan_Joint':  2.09,
        'wrist_roll_Joint': 3.14,
    }

    def _check_trajectory_velocity(self, trajectory):
        """Verify no joint in the trajectory exceeds its URDF velocity limit.
        Returns True if safe, False if any violation found."""
        jt = trajectory.joint_trajectory
        names = jt.joint_names
        points = jt.points

        if len(points) < 2:
            return True

        for i in range(1, len(points)):
            p0 = points[i - 1]
            p1 = points[i]
            dt_sec = (p1.time_from_start.sec - p0.time_from_start.sec)
            dt_nsec = (p1.time_from_start.nanosec - p0.time_from_start.nanosec)
            dt = dt_sec + dt_nsec * 1e-9
            if dt < 1e-6:
                continue  # skip zero-duration segments

            for j_idx, jname in enumerate(names):
                if jname not in self._VELOCITY_LIMITS:
                    continue
                delta = abs(p1.positions[j_idx] - p0.positions[j_idx])
                vel = delta / dt
                limit = self._VELOCITY_LIMITS[jname]
                if vel > limit * 1.2:  # 20% margin
                    self.get_logger().error(
                        f'SAFETY: {jname} velocity {vel:.2f} exceeds '
                        f'limit {limit:.2f} (segment {i}, dt={dt:.3f}s)')
                    return False
        return True

    # ======================================================================
    # Flipper control (runs on executor timer, independent of arm pipeline)
    # ======================================================================

    # Per-call validity timeout.  Short enough that even if every call
    # times out, the flipper accumulator stays bounded; long enough that
    # a healthy MoveIt always answers within budget.  20 Hz tick * 0.2 s
    # = at most one timeout per 200 ms (with single-flight lock).
    _FLIPPER_VALIDITY_TIMEOUT = 0.2

    def _flipper_tick(self):
        """Called by flipper timer.  Reads latest commands, accumulates
        positions (after a MoveIt collision check), and sends to
        flipper_controller at a reduced rate.

        The collision check uses the current arm joint state plus the
        proposed flipper positions; if the resulting pose would collide
        (e.g. flipper into arm or flipper into flipper), the accumulator
        is held at its previous value.  If both flippers are commanded
        together and only one is unsafe, the safe one is allowed to
        advance.

        Single-flight: only one tick at a time may have a validity
        request in flight.  Others early-return without advancing the
        accumulator, so no trajectories are emitted past the last-known-
        safe setpoint.
        """
        # Joint-state feedback watchdog: block if no feedback
        if self._js_watchdog_triggered:
            return

        # A running sequence (HOME or STAIR) owns the flippers.  Operator
        # stick input is ignored until it finishes or times out, matching
        # the arm's "once HOME starts, it completes" policy.
        # Note _prev_{front,rear}_flipper_cmd are deliberately NOT updated
        # while a sequence runs, so a stick held throughout reads as a fresh
        # command on the first tick afterwards and snaps the accumulator to
        # actual before moving.
        if self._flipper_seq_label is not None:
            self._flipper_seq_tick()
            return

        with self._flipper_cmd_lock:
            front_cmd = self._front_flipper_cmd
            rear_cmd = self._rear_flipper_cmd

        # Detect "fresh" commands: either a transition from rest (prev=0,
        # current!=0) or a direction reversal (prev and current have
        # opposite sign).  In both cases the accumulator may be ahead of
        # actual from previous motion, and we need to clear that gap so
        # the operator's command takes effect immediately.
        front_fresh = (front_cmd != 0 and front_cmd != self._prev_front_flipper_cmd)
        rear_fresh  = (rear_cmd  != 0 and rear_cmd  != self._prev_rear_flipper_cmd)
        # Save cmds for next tick BEFORE the early-return below so a
        # release-then-resume sequence is correctly detected as fresh.
        self._prev_front_flipper_cmd = front_cmd
        self._prev_rear_flipper_cmd = rear_cmd

        if front_cmd == 0 and rear_cmd == 0:
            return

        # Snap accumulators to actual joint positions in two cases:
        #   - uncommanded side (cmd == 0): trajectory target = actual,
        #     so the JTC doesn't twitch the unmoved flipper.
        #   - fresh-commanded side (start or reversal): clears any gap
        #     between accumulator and actual that built up from
        #     controller lag during the prior motion.
        self._snap_flipper_accumulator_to_actual(
            sync_front=(front_cmd == 0) or front_fresh,
            sync_rear=(rear_cmd  == 0) or rear_fresh)

        # Compute proposed new positions
        proposed_front = self._front_flipper_pos + front_cmd * self._flipper_step
        proposed_rear  = self._rear_flipper_pos  + rear_cmd  * self._flipper_step

        # Clamp to URDF limits
        lo, hi = FLIPPER_LIMITS['front_flipper_Joint']
        proposed_front = max(lo, min(hi, proposed_front))
        lo, hi = FLIPPER_LIMITS['rear_flipper_Joint']
        proposed_rear  = max(lo, min(hi, proposed_rear))

        # Physical-position cap: prevent accumulator from running ahead
        # of the actual motor beyond flipper_ahead_limit.  Same pattern
        # as the telescope and roll physical caps.
        fl = self._flipper_ahead_limit
        with self._js_lock:
            js = self._last_joint_state
        if js is not None:
            for i, jname in enumerate(js.name):
                if jname == 'front_flipper_Joint' and front_cmd != 0:
                    phys = js.position[i]
                    proposed_front = max(phys - fl,
                                         min(phys + fl, proposed_front))
                elif jname == 'rear_flipper_Joint' and rear_cmd != 0:
                    phys = js.position[i]
                    proposed_rear = max(phys - fl,
                                        min(phys + fl, proposed_rear))

        # No actual change after clamp? Skip.
        actually_changed = (
            abs(proposed_front - self._front_flipper_pos) > 1e-9 or
            abs(proposed_rear  - self._rear_flipper_pos)  > 1e-9)
        if not actually_changed:
            return

        if self._flipper_collision_check:
            # Single-flight: if a previous tick's check is still pending,
            # don't queue another one.  Just hold this tick.
            if not self._flipper_check_busy.acquire(blocking=False):
                return
            try:
                committed = self._flipper_collision_gate(
                    front_cmd, rear_cmd, proposed_front, proposed_rear)
            finally:
                self._flipper_check_busy.release()
            if committed is None:
                return       # blocked, nothing to do
            proposed_front, proposed_rear = committed

        # Commit (possibly partial)
        self._front_flipper_pos = proposed_front
        self._rear_flipper_pos  = proposed_rear

        # Rate-limit sends to prevent action client goal handle overflow
        self._flipper_tick_count += 1
        if self._flipper_tick_count < self._flipper_send_interval:
            return
        self._flipper_tick_count = 0
        self._send_flipper_trajectory()

    def _flipper_collision_gate(self, front_cmd, rear_cmd,
                                proposed_front, proposed_rear):
        """Run collision checks against the proposed flipper positions.
        Returns a (front, rear) tuple to commit, or None if the move is
        fully blocked (caller must early-return).

        - If the combined motion is safe: commit both.
        - If both flippers are commanded and the combined motion fails:
          probe each side individually; commit whichever side passes,
          hold the other.  Block both only if neither side passes.
        - If only one flipper is commanded and the check fails: block.
        """
        timeout = self._FLIPPER_VALIDITY_TIMEOUT

        jd = self._build_flipper_check_state(proposed_front, proposed_rear)
        if jd is None:
            return None    # already logged inside helper

        if self._check_collision(jd, group_name='', timeout_sec=timeout):
            return (proposed_front, proposed_rear)

        # Combined motion blocked.
        both_commanded = (front_cmd != 0 and rear_cmd != 0)
        if not both_commanded:
            # Single-side command can't be split.  Block.
            self._log_flipper_blocked(front_cmd, rear_cmd,
                                      proposed_front, proposed_rear)
            return None

        # Try front-only first.
        jd_f = self._build_flipper_check_state(
            proposed_front, self._rear_flipper_pos)
        if jd_f is not None and self._check_collision(
                jd_f, group_name='', timeout_sec=timeout):
            return (proposed_front, self._rear_flipper_pos)

        # Try rear-only.
        jd_r = self._build_flipper_check_state(
            self._front_flipper_pos, proposed_rear)
        if jd_r is not None and self._check_collision(
                jd_r, group_name='', timeout_sec=timeout):
            return (self._front_flipper_pos, proposed_rear)

        # Fully blocked.
        self._log_flipper_blocked(front_cmd, rear_cmd,
                                  proposed_front, proposed_rear)
        return None

    def _log_flipper_blocked(self, front_cmd, rear_cmd,
                             proposed_front, proposed_rear):
        """Throttled WARN that the flipper accumulator was held."""
        now = time.monotonic()
        if now - self._last_flipper_collision_log <= 1.0:
            return
        self._last_flipper_collision_log = now
        self.get_logger().warn(
            f'Flipper blocked: '
            f'front {self._front_flipper_pos:+.3f}->{proposed_front:+.3f} '
            f'(cmd {front_cmd:+d})  '
            f'rear {self._rear_flipper_pos:+.3f}->{proposed_rear:+.3f} '
            f'(cmd {rear_cmd:+d})')

    def _snap_flipper_accumulator_to_actual(self, sync_front, sync_rear):
        """Snap one or both flipper accumulators to the corresponding
        actual joint positions from the latest joint state.

        Used in two cases:

        - Un-commanded side (cmd == 0): keeps the trajectory's target
          equal to the joint's actual position so the JTC has nothing
          to correct on the un-commanded side.

        - Fresh command (start or direction reversal): clears any
          accumulator-vs-actual gap that built up from controller lag,
          so reversing direction takes effect immediately rather than
          requiring the accumulator to first catch back down to actual.

        No-op if neither flag is set or the joint state isn't available.
        """
        if not (sync_front or sync_rear):
            return
        with self._js_lock:
            js = self._last_joint_state
        if js is None:
            return
        for i, jname in enumerate(js.name):
            if sync_front and jname == 'front_flipper_Joint':
                self._front_flipper_pos = js.position[i]
            elif sync_rear and jname == 'rear_flipper_Joint':
                self._rear_flipper_pos = js.position[i]

    def _build_flipper_check_state(self, front_pos, rear_pos):
        """Build a joint dict (current arm state + proposed flipper
        positions) suitable for /check_state_validity.  Returns None if
        the latest joint state is missing or doesn't include all arm
        joints — caller treats that as fail-safe (block motion)."""
        with self._js_lock:
            js = self._last_joint_state
        if js is None:
            now = time.monotonic()
            if now - self._last_flipper_collision_log > 2.0:
                self._last_flipper_collision_log = now
                self.get_logger().warn(
                    'Flipper collision check: no joint state yet — '
                    'blocking motion.')
            return None
        joint_dict = {}
        for i, jname in enumerate(js.name):
            if jname in ARM_JOINTS:
                joint_dict[jname] = js.position[i]
        if len(joint_dict) != len(ARM_JOINTS):
            now = time.monotonic()
            if now - self._last_flipper_collision_log > 2.0:
                self._last_flipper_collision_log = now
                self.get_logger().warn(
                    f'Flipper collision check: only got '
                    f'{len(joint_dict)}/{len(ARM_JOINTS)} arm joints from '
                    f'joint state — blocking motion.')
            return None
        joint_dict['front_flipper_Joint'] = front_pos
        joint_dict['rear_flipper_Joint']  = rear_pos
        return joint_dict

    def _send_flipper_trajectory(self):
        """Send current flipper positions to flipper_controller (non-blocking)."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = FLIPPER_JOINTS

        point = JointTrajectoryPoint()
        point.positions = [self._front_flipper_pos, self._rear_flipper_pos]
        point.velocities = [0.0, 0.0]
        point.time_from_start = Duration(sec=0, nanosec=100_000_000)  # 0.1 s
        goal.trajectory.points.append(point)

        # Fire-and-forget — don't wait for acceptance or result
        self._flipper_client.send_goal_async(goal)

        # Mirror this dispatch into the flipper joint command topic stream so
        # both channels carry the same target.  Hardware bridges that consume
        # the topic instead of the action see exactly what the action sends.
        self._latest_flipper_joint_target = {
            'front_flipper_Joint': float(self._front_flipper_pos),
            'rear_flipper_Joint':  float(self._rear_flipper_pos),
        }

    # ======================================================================
    # Flipper sequence runner (parallel to the arm; groups run in order)
    # ======================================================================

    def _flipper_joint_position(self, jname):
        """Measured position of one flipper joint from the latest joint
        state, or None if the joint state is missing or lacks that joint."""
        with self._js_lock:
            js = self._last_joint_state
        if js is None:
            return None
        for i, n in enumerate(js.name):
            if n == jname:
                return js.position[i]
        return None

    def _start_flipper_sequence(self, label, targets, groups):
        """Begin a flipper sequence.

        targets : {joint_name: position}
        groups  : list of joint-name tuples, processed in order.  Joints
                  within a group move together; the next group starts only
                  once the current one has finished or timed out.

        Called from the worker thread; stepped by _flipper_tick on the
        executor, so it advances in parallel with the arm rather than
        blocking it.
        """
        # Start both accumulators from the measured positions, so moving
        # joints ramp from reality and held joints target where they
        # actually are (nothing to correct on the joints that stay put).
        self._snap_flipper_accumulator_to_actual(sync_front=True,
                                                 sync_rear=True)
        self._flipper_seq_targets = dict(targets)
        self._flipper_seq_groups = [tuple(g) for g in groups]
        self._flipper_seq_index = 0
        self._flipper_seq_final_sent = False
        self._flipper_seq_deadline = (
            time.monotonic() + self._flipper_home_timeout)
        self._flipper_seq_label = label      # set LAST — tick reads this

        def short(j):
            return j.replace('_flipper_Joint', '')
        order = ' then '.join('+'.join(short(j) for j in g)
                              for g in self._flipper_seq_groups)
        tgt = '  '.join(f'{short(j)}={targets[j]:+.4f}' for j in sorted(targets))
        self.get_logger().warn(
            f'Flipper {label} sequence started: {order}  ({tgt}, '
            f'step={self._flipper_home_step:.3f} rad/tick, '
            f'tol={self._flipper_home_tol:.3f} rad, '
            f'timeout={self._flipper_home_timeout:.1f}s per group).')

    def _start_flipper_home(self):
        """Flipper leg of a HOME command: front to zero first, then rear.
        No-op when flipper_home_enable is false."""
        if not self._flipper_home_enable:
            return
        self._start_flipper_sequence(
            'HOME', dict(HOME_FLIPPERS),
            [('front_flipper_Joint',), ('rear_flipper_Joint',)])

    def _start_flipper_stair(self):
        """Flipper leg of STAIR mode: one flipper at a time, REAR first and
        then FRONT, so only one end of the robot is ever lifting.

        Note the order is the reverse of HOME's (front then rear).  Always
        runs — the stair pose is the whole point of the mode, so it is not
        gated on flipper_home_enable."""
        self._start_flipper_sequence(
            'STAIR',
            {'front_flipper_Joint': self._stair_front,
             'rear_flipper_Joint':  self._stair_rear},
            [('rear_flipper_Joint',), ('front_flipper_Joint',)])

    def _abort_flipper_sequence(self, reason: str):
        """Stop the active flipper sequence early.  Accumulators are left
        where they are, so the flippers hold their current commanded
        position rather than snapping anywhere."""
        if self._flipper_seq_label is None:
            return
        label = self._flipper_seq_label
        self._flipper_seq_label = None
        # Cancel the outstanding command: the last dispatched target can be
        # up to flipper_ahead_limit beyond the measured position, so without
        # this the motor would keep driving to it and complete up to ~0.05
        # rad of sequence-commanded motion AFTER the abort.  Re-dispatching
        # the measured position stops the flipper where it actually is.
        self._snap_flipper_accumulator_to_actual(sync_front=True,
                                                 sync_rear=True)
        self._send_flipper_trajectory()
        self._prev_front_flipper_cmd = 0
        self._prev_rear_flipper_cmd = 0
        self.get_logger().warn(
            f'Flipper {label} sequence aborted: {reason}. '
            f'Holding at measured position.')

    def _flipper_seq_tick(self):
        """One step of the active flipper sequence.  Called from
        _flipper_tick while _flipper_seq_label is set.

        Every joint in the active group ramps toward its target at
        flipper_home_step per tick; joints outside the group hold at their
        current accumulator value.  Each proposal is clamped to the joint's
        configured range and then capped to flipper_ahead_limit from the
        MEASURED position, so the command can never run away from the motor
        and a slow motor simply sets the pace.

        A group completes when every joint in it has reached its target AND
        been measured within flipper_home_tolerance of it, or when the group
        deadline expires — loud, but not fatal, so one stuck flipper cannot
        block the rest of the sequence.
        """
        label = self._flipper_seq_label
        group = self._flipper_seq_groups[self._flipper_seq_index]

        # ----- propose a new command for each joint in the active group ----
        proposals = {}
        phys_now = {}
        for jname in group:
            target = self._flipper_seq_targets[jname]
            cur = getattr(self, FLIPPER_POS_ATTR[jname])
            step = self._flipper_home_step
            if abs(cur - target) <= step:
                proposed = target
            else:
                proposed = cur - math.copysign(step, cur - target)

            # Range clamp FIRST, physical cap LAST, so the cap always has the
            # final say and the command can never jump more than
            # flipper_ahead_limit from the measured joint — even if the
            # configured range disagrees with what the hardware reports.
            lo, hi = FLIPPER_LIMITS[jname]
            proposed = max(lo, min(hi, proposed))
            phys = self._flipper_joint_position(jname)
            phys_now[jname] = phys
            if phys is not None:
                fl = self._flipper_ahead_limit
                proposed = max(phys - fl, min(phys + fl, proposed))
            proposals[jname] = proposed

        changed = any(
            abs(proposals[j] - getattr(self, FLIPPER_POS_ATTR[j])) > 1e-9
            for j in group)

        # ----- one combined collision check for the whole group ------------
        if self._flipper_collision_check and changed:
            if not self._flipper_check_busy.acquire(blocking=False):
                return              # a check is already in flight; hold
            try:
                jd = self._build_flipper_check_state(
                    proposals.get('front_flipper_Joint',
                                  self._front_flipper_pos),
                    proposals.get('rear_flipper_Joint',
                                  self._rear_flipper_pos))
                safe = (jd is not None and self._check_collision(
                    jd, group_name='',
                    timeout_sec=self._FLIPPER_VALIDITY_TIMEOUT))
            finally:
                self._flipper_check_busy.release()
            if not safe:
                now = time.monotonic()
                if now - self._flipper_seq_last_log > 1.0:
                    self._flipper_seq_last_log = now
                    self.get_logger().warn(
                        f'Flipper {label}: motion held by collision check '
                        f'(or validity timeout) — waiting for it to clear '
                        f'or for the group to time out.')
                # Hold every joint in the group at its current command.
                proposals = {j: getattr(self, FLIPPER_POS_ATTR[j])
                             for j in group}

        for jname, value in proposals.items():
            setattr(self, FLIPPER_POS_ATTR[jname], value)

        # ----- dispatch ----------------------------------------------------
        # Rate-limited exactly like teleop, plus ONE guaranteed send on the
        # tick the group lands on target so the final exact value cannot be
        # swallowed by the rate limiter.  _flipper_seq_final_sent keeps that
        # from repeating every tick while we wait for the motors to arrive.
        at_target = all(
            abs(proposals[j] - self._flipper_seq_targets[j]) <= 1e-9
            for j in group)
        self._flipper_tick_count += 1
        send = self._flipper_tick_count >= self._flipper_send_interval
        if at_target and not self._flipper_seq_final_sent:
            self._flipper_seq_final_sent = True
            send = True
        if send:
            self._flipper_tick_count = 0
            self._send_flipper_trajectory()

        # ----- group completion --------------------------------------------
        arrived = at_target and all(
            phys_now[j] is None
            or abs(phys_now[j] - self._flipper_seq_targets[j])
            <= self._flipper_home_tol
            for j in group)
        timed_out = time.monotonic() > self._flipper_seq_deadline
        if not (arrived or timed_out):
            return

        n_groups = len(self._flipper_seq_groups)
        detail = '  '.join(
            j.replace('_flipper_Joint', '') + '='
            + (f'{phys_now[j]:+.3f}' if phys_now[j] is not None else 'unknown')
            for j in group)
        if arrived:
            self.get_logger().info(
                f'Flipper {label}: group {self._flipper_seq_index + 1}/'
                f'{n_groups} at target ({detail}).')
        else:
            self.get_logger().error(
                f'Flipper {label}: group {self._flipper_seq_index + 1}/'
                f'{n_groups} TIMEOUT after {self._flipper_home_timeout:.1f}s '
                f'({detail}, tolerance {self._flipper_home_tol:.3f}). '
                f'Giving up on this group and moving on.')

        self._flipper_seq_index += 1
        if self._flipper_seq_index < n_groups:
            self._flipper_seq_final_sent = False
            self._flipper_seq_deadline = (
                time.monotonic() + self._flipper_home_timeout)
            return

        self._flipper_seq_label = None
        # Force the next operator command to read as "fresh" so it snaps the
        # accumulator to actual before moving (see _flipper_tick).
        self._prev_front_flipper_cmd = 0
        self._prev_rear_flipper_cmd = 0
        self.get_logger().warn(f'Flipper {label} sequence complete.')

    # ======================================================================
    # Arm integration tick (runs on executor timer)
    # ======================================================================

    def _arm_tick(self):
        """Integrate latest joystick deltas into _current_goal,
        _current_pitch, and _current_roll.  Handles FIRING mode
        (direct wrist_pan control).  Does nothing while HOME is in
        flight or re-arm is pending."""

        # STAIR: latch into HOLD once BOTH legs of the stair pose have
        # finished (arm pose commanded and physically arrived, flipper
        # sequence complete), or once the backstop deadline expires.  Pure
        # state evaluation, so it runs even while a watchdog blocks motion
        # below and cannot leave the mode stuck half-entered.
        if self._stair_state == 'MOVING':
            timed_out = time.monotonic() > self._stair_deadline
            flippers_done = self._flipper_seq_label is None
            arm_done = self._stair_arm_done and self._arm_at_pose(
                self._stair_arm_joints, self._goal_arrival_tolerance)
            if (flippers_done and arm_done) or timed_out:
                self._stair_state = 'HOLD'
                if timed_out and not (flippers_done and arm_done):
                    self.get_logger().error(
                        f'STAIR pose TIMEOUT after '
                        f'{self._stair_arm_timeout:.1f}s '
                        f'(arm_done={arm_done} flippers_done={flippers_done}) '
                        f'— latching anyway. Arm LOCKED until '
                        f'operation_mode returns to ARMED.')
                else:
                    self.get_logger().warn(
                        'STAIR pose reached — arm LOCKED until '
                        'operation_mode returns to ARMED. '
                        'Drive and flippers remain live.')

        # Watchdog: if RC link lost, block all motion
        if self._watchdog_triggered:
            return

        # Joint-state feedback watchdog: block if no feedback
        if self._js_watchdog_triggered:
            return

        # STAIR owns the arm for the whole mode — no accumulation, no
        # dispatch, regardless of what the sticks say.  (_on_sbus already
        # zeroes the arm commands; this is the belt-and-braces guard.)
        if self._stair_state is not None:
            return

        # While HOME is being processed, don't integrate — HOME wins.
        if self._home_requested.is_set():
            return

        with self._arm_cmd_lock:
            dx_cmd = self._arm_x_cmd
            dy_cmd = self._arm_y_cmd
            dz_cmd = self._arm_z_cmd
            pitch_cmd = self._ee_pitch_cmd
            roll_cmd = self._ee_roll_cmd
            tele_cmd = self._telescope_cmd

        # --- FIRING mode: direct wrist_pan control only -------------------
        if self._firing_active:
            if pitch_cmd == 0:
                # Pitch released in FIRING — cancel trajectory
                if 'firing_pitch' in self._prev_active_inputs:
                    self._prev_active_inputs = set()
                    with self._arm_handle_lock:
                        handle = self._current_arm_goal_handle
                        self._current_arm_goal_handle = None
                    if handle is not None:
                        try:
                            handle.cancel_goal_async()
                        except Exception:
                            pass
                return
            self._prev_active_inputs = {'firing_pitch'}
            # Reverse direction for firing mode
            new_pan = self._firing_wrist_pan + (-pitch_cmd) * self._pitch_step
            lo, hi = JOINT_LIMITS['wrist_pan_Joint']
            new_pan = max(lo, min(hi, new_pan))

            # Collision check before committing
            firing_joints = {j: 0.0 for j in ARM_JOINTS}
            firing_joints['wrist_pan_Joint'] = new_pan
            if not self._check_collision(firing_joints):
                self.get_logger().warn(
                    f'FIRING: wrist_pan={new_pan:.3f} in collision, blocked.')
                return

            self._firing_wrist_pan = new_pan

            # Rate-limit sends to prevent action client crash
            self._firing_tick_count += 1
            if self._firing_tick_count >= self._firing_send_interval:
                self._firing_tick_count = 0
                self._send_firing_trajectory()
            return

        all_centered = (dx_cmd == 0 and dy_cmd == 0 and dz_cmd == 0
                        and pitch_cmd == 0 and roll_cmd == 0
                        and tele_cmd == 0)

        # Build set of currently active inputs
        current_active = set()
        if dx_cmd != 0 or dy_cmd != 0 or dz_cmd != 0:
            current_active.add('pos')
        if pitch_cmd != 0:
            current_active.add('pitch')
        if roll_cmd != 0:
            current_active.add('roll')
        if tele_cmd != 0:
            current_active.add('telescope')

        # Detect any input that was just released
        released = self._prev_active_inputs - current_active
        self._prev_active_inputs = current_active

        if released:
            # Log enough state to diagnose post-release behaviour
            self.get_logger().info(
                f'Released: {released} '
                f'tele_cur={self._current_telescope:+.4f} '
                f'tele_lv={self._last_valid_telescope:+.4f} '
                f'tele_pending={self._pending_telescope:+.4f} '
                f'pipe_busy={self.state.name}')

            # Cancel running trajectory immediately — stop all motion
            with self._arm_handle_lock:
                handle = self._current_arm_goal_handle
                self._current_arm_goal_handle = None
            if handle is not None:
                try:
                    handle.cancel_goal_async()
                    self.get_logger().info('Cancel sent for in-flight trajectory.')
                except Exception as e:
                    self.get_logger().warn(f'Cancel failed: {e}')
            else:
                self.get_logger().info('Release: no in-flight handle to cancel.')

            # Mark any in-flight pipeline goal as stale — if the worker
            # is mid-IK/planning, it will skip execution when it finishes.
            self._pipeline_stale = True

            # Snap accumulated state to current physical joints so the
            # next goal doesn't repeat the overshoot.
            self._snap_to_current_joints()

            # Drain any queued goals — they have stale values
            with self._goal_lock:
                self._pending_goal = None
                self._pending_pitch = 0.0
                self._pending_roll = 0.0
                self._pending_telescope = 0.0
            while not self._event_queue.empty():
                try:
                    self._event_queue.get_nowait()
                except Empty:
                    break

            self.get_logger().debug(
                f'Released: {released} — trajectory cancelled, state snapped.')

        # Re-arm handling: block until all inputs centered
        if self._rearm_required:
            if all_centered:
                self._rearm_required = False
                self.get_logger().info('Re-arm complete: sticks centered.')
            return

        if all_centered:
            return

        # Accumulate position (per-axis hybrid: snap uncommanded axes to
        # FK-anchored last_valid, accumulate commanded axes open-loop).
        #
        # The rationale: with _last_valid_goal anchored to the FK-actual wrist
        # position after each tick, we want uncommanded axes to STAY at that
        # actual position (so a tick that doesn't command X can't carry stale
        # X demand forward).  But we also want commanded axes to retain the
        # open-loop responsiveness — so that holding +X joystick continues
        # to drive the goal forward at arm_step per tick, exactly as before,
        # rather than collapsing to "1 step beyond actual wrist".  Per-axis
        # selection gives both.
        if dx_cmd != 0 or dy_cmd != 0 or dz_cmd != 0:
            lv = self._last_valid_goal
            # Per axis: if commanded, accumulate from previous goal; if not,
            # snap to last_valid (which is FK-actual after previous tick).
            nx = (self._current_goal[0] + dx_cmd * self._arm_step
                  if dx_cmd != 0 else lv[0])
            ny = (self._current_goal[1] + dy_cmd * self._arm_step
                  if dy_cmd != 0 else lv[1])
            nz = (self._current_goal[2] + dz_cmd * self._arm_step
                  if dz_cmd != 0 else lv[2])
            # Cap each axis independently against last_valid ± max_ahead so
            # the open-loop branch can't run too far ahead of FK-actual.
            ma = self._pos_max_ahead
            nx = max(lv[0] - ma, min(lv[0] + ma, nx))
            ny = max(lv[1] - ma, min(lv[1] + ma, ny))
            nz = max(lv[2] - ma, min(lv[2] + ma, nz))
            self._current_goal = (nx, ny, nz)
        else:
            # No position command at all — snap fully to last_valid so any
            # accumulated drift collapses cleanly during orientation-only
            # or telescope-only ticks.
            nx, ny, nz = self._last_valid_goal
            self._current_goal = (nx, ny, nz)

        # Accumulate pitch (capped to max_ahead from last_valid)
        if pitch_cmd != 0:
            new_pitch = self._current_pitch + pitch_cmd * self._pitch_step
            lv = self._last_valid_pitch
            new_pitch = max(lv - self._pitch_max_ahead,
                            min(lv + self._pitch_max_ahead, new_pitch))
            new_pitch = max(-self._pitch_limit,
                            min(self._pitch_limit, new_pitch))
            self._current_pitch = new_pitch

        # Accumulate roll, capped against the PHYSICAL wrist_roll position
        # with a configurable window (roll_ahead_limit).  Same pattern as
        # the telescope physical cap — prevents the accumulator from
        # running ahead of the motor indefinitely.
        if roll_cmd != 0:
            phys_roll = None
            with self._js_lock:
                js = self._last_joint_state
            if js is not None:
                for i, jname in enumerate(js.name):
                    if jname == 'wrist_roll_Joint':
                        phys_roll = js.position[i]
                        break

            new_roll = self._current_roll + roll_cmd * self._roll_step

            # Cap against physical position (primary), with last_valid
            # fallback if joint state is not yet available.
            rl = self._roll_ahead_limit
            if phys_roll is not None:
                new_roll = max(phys_roll - rl,
                               min(phys_roll + rl, new_roll))
            else:
                lv = self._last_valid_roll
                new_roll = max(lv - self._roll_max_ahead,
                               min(lv + self._roll_max_ahead, new_roll))

            new_roll = max(-self._roll_limit,
                           min(self._roll_limit, new_roll))
            self._current_roll = new_roll

        # Accumulate telescope, capped against the PHYSICAL telescope position
        # with a fixed 2 cm window (not against _last_valid which marches
        # forward at dispatch rate even when the motor doesn't follow).
        # This bounds the commanded position to never run more than 2 cm
        # ahead of where the motor actually is.
        if tele_cmd != 0:
            # Read the current physical telescope position from joint state.
            phys_tele = None
            with self._js_lock:
                js = self._last_joint_state
            if js is not None:
                for i, jname in enumerate(js.name):
                    if jname == 'telescope_Joint':
                        phys_tele = js.position[i]
                        break

            raw_advance = self._current_telescope + tele_cmd * self._telescope_step

            # Hard cap: configurable window ahead/behind physical,
            # then URDF joint limit.
            tl = self._telescope_ahead_limit
            if phys_tele is not None:
                after_phys_cap = max(phys_tele - tl,
                                     min(phys_tele + tl, raw_advance))
            else:
                # Fall back to last_valid if joint state is missing
                after_phys_cap = max(self._last_valid_telescope - tl,
                                     min(self._last_valid_telescope + tl,
                                         raw_advance))

            tlo, thi = JOINT_LIMITS['telescope_Joint']
            after_limit = max(tlo, min(thi, after_phys_cap))

            # Diagnostics: log when cap or joint-limit clips, plus periodically.
            phys_clipped = abs(raw_advance - after_phys_cap) > 1e-9
            limit_clipped = abs(after_phys_cap - after_limit) > 1e-9
            self._tele_log_counter = getattr(self, '_tele_log_counter', 0) + 1
            if phys_clipped or limit_clipped or self._tele_log_counter % 20 == 0:
                phys_str = (f'{phys_tele:+.4f}' if phys_tele is not None
                            else 'unknown')
                self.get_logger().info(
                    f'Telescope accum: cmd={tele_cmd:+d} '
                    f'cur={self._current_telescope:+.4f} -> {after_limit:+.4f}  '
                    f'phys={phys_str} cap=±{tl:.3f}  '
                    f'phys_clip={phys_clipped} limit_clip={limit_clipped}')
            self._current_telescope = after_limit
        else:
            # Reset the periodic-log counter so the first tick after release
            # of an extend hold doesn't immediately log.
            self._tele_log_counter = 0

        # Deadband: any change triggers a goal
        dx = nx - self._last_valid_goal[0]
        dy = ny - self._last_valid_goal[1]
        dz = nz - self._last_valid_goal[2]
        pos_dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        pitch_delta = abs(self._current_pitch - self._last_dispatched_pitch)
        roll_delta = abs(self._current_roll - self._last_dispatched_roll)
        tele_delta = abs(self._current_telescope - self._last_dispatched_telescope)

        if (pos_dist < self._goal_deadband
                and pitch_delta < self._pitch_deadband
                and roll_delta < self._roll_deadband
                and tele_delta < self._telescope_deadband):
            return

        # Enqueue position + orientation + telescope
        with self._goal_lock:
            self._pending_goal = self._current_goal
            self._pending_pitch = self._current_pitch
            self._pending_roll = self._current_roll
            self._pending_telescope = self._current_telescope
        self._last_dispatched_pitch = self._current_pitch
        self._last_dispatched_roll = self._current_roll
        self._last_dispatched_telescope = self._current_telescope
        self._pipeline_stale = False  # fresh goal — clear stale flag

        # Operator is actively commanding motion
        self._at_home = False

        self._last_goal_display = {
            'x': round(nx, 4),
            'y': round(ny, 4),
            'z': round(nz, 4),
            'pitch': round(self._current_pitch, 3),
            'roll': round(self._current_roll, 3),
            'telescope': round(self._current_telescope, 4),
        }
        self._last_goal_time = datetime.now().isoformat(timespec='seconds')
        self._event_queue.put(EventType.GOAL)
        self.get_logger().info(
            f'Arm tick: goal ({nx:.4f}, {ny:.4f}, {nz:.4f}) '
            f'pitch={self._current_pitch:.3f} roll={self._current_roll:.3f} '
            f'tele={self._current_telescope:.4f}')

    def _send_firing_trajectory(self):
        """Send a direct joint trajectory for FIRING mode: all joints at
        home (0) except wrist_pan_Joint at the accumulated firing angle.
        Bypasses IK and OMPL entirely — direct joint control."""
        joints = {j: 0.0 for j in ARM_JOINTS}
        joints['wrist_pan_Joint'] = self._firing_wrist_pan

        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = [joints[j] for j in ARM_JOINTS]
        point.velocities = [0.0] * len(ARM_JOINTS)
        point.time_from_start = Duration(sec=0, nanosec=100_000_000)
        traj.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self._traj_client.send_goal_async(goal)

        # Keep the /arm_joint_commands topic in sync.  Without this,
        # the 50 Hz publisher keeps broadcasting wrist_pan=0 (from HOME)
        # while the 4 Hz action sends the actual firing angle — the
        # high-rate "hold at 0" overwrites the low-rate command on any
        # hardware bridge that reads from the topic.
        self._latest_arm_joint_target = dict(joints)
        self.get_logger().debug(
            f'FIRING: wrist_pan={self._firing_wrist_pan:.3f} rad')

    def _snap_to_current_joints(self):
        """Reset accumulators and deadband anchors to the last VERIFIED
        position (_last_valid) on input release.

        Previous design set ``_last_valid = _current`` (promoting
        accumulated state to verified).  This caused a loophole: when
        hardware was lagging, the arrival check would hold _last_valid,
        but each release–repress cycle would launder the unverified
        _current into _last_valid, letting the commanded position creep
        forward despite frozen motors.

        The fix: always snap _current TO _last_valid (like a soft
        rollback).  On re-press, accumulation restarts from the last
        position the hardware was verified to have reached.  If the
        hardware was tracking normally, _last_valid ≈ _current, so the
        change is negligible.
        """
        # Reset accumulators to the last verified position.
        self._current_goal = self._last_valid_goal
        self._current_pitch = self._last_valid_pitch
        self._current_roll = self._last_valid_roll
        self._current_telescope = self._last_valid_telescope

        # Sync deadband anchors so the next tick doesn't immediately
        # re-dispatch the snapped-back position (deadband would be 0).
        self._last_dispatched_pitch = self._last_valid_pitch
        self._last_dispatched_roll = self._last_valid_roll
        self._last_dispatched_telescope = self._last_valid_telescope

    # ======================================================================
    # Rollback (soft failure — not a FAULT)
    # ======================================================================

    def _rollback(self, reason: str):
        """Handle an IK or planning failure.  Snap _current_goal,
        _current_pitch and _current_roll back to last valid, block
        further accumulation until sticks center, and return to IDLE."""
        self._current_goal = self._last_valid_goal
        self._current_pitch = self._last_valid_pitch
        self._current_roll = self._last_valid_roll
        self._current_telescope = self._last_valid_telescope
        self._rearm_required = True
        self._last_rollback_reason = reason
        self._last_rollback_time = datetime.now().isoformat(timespec='seconds')
        self.get_logger().warn(
            f'Rollback: {reason} — snap to last valid, re-arm required.')
        self._last_accepted_goal = self._last_valid_goal
        self.state = State.IDLE

    # ======================================================================
    # Worker thread — single consumer
    # ======================================================================

    def _worker_loop(self):
        """Main event loop.  Runs on a daemon thread.  Only this thread
        writes to self.state."""
        self.get_logger().info('Worker thread started.')
        while rclpy.ok():
            try:
                event = self._event_queue.get(timeout=0.1)
            except Empty:
                continue

            try:
                if event == EventType.RESET:
                    if self.state == State.FAULT:
                        self.state = State.IDLE
                        self._last_error = ''
                    continue

                if event == EventType.GOAL:
                    # Auto-reset from FAULT on new goal
                    if self.state == State.FAULT:
                        self.state = State.IDLE
                        self._last_error = ''

                    # Drain stale GOAL events — we always use _pending_goal
                    while not self._event_queue.empty():
                        try:
                            stale = self._event_queue.get_nowait()
                            if stale == EventType.RESET:
                                pass  # ignore during goal processing
                        except Empty:
                            break

                    # Grab the latest goal + pitch + roll + telescope
                    with self._goal_lock:
                        goal = self._pending_goal
                        ee_pitch = self._pending_pitch
                        ee_roll = self._pending_roll
                        ee_telescope = self._pending_telescope
                        self._pending_goal = None
                        self._pending_pitch = 0.0
                        self._pending_roll = 0.0
                        self._pending_telescope = 0.0

                    if goal is None:
                        continue

                    self._process_goal(goal, ee_pitch, ee_roll, ee_telescope)

            except Exception as e:
                self.get_logger().error(f'Worker exception: {e}')
                import traceback
                self.get_logger().error(traceback.format_exc())
                self.state = State.FAULT
                self._last_error = str(e)

    # ======================================================================
    # Goal processing pipeline
    # ======================================================================

    def _process_goal(self, goal, ee_pitch=0.0, ee_roll=0.0, ee_telescope=0.0):
        """Run the full VALIDATE -> PLANNING -> EXECUTING pipeline.
        Aborts mid-pipeline if HOME is requested.
        On IK/plan failure, rolls back to last valid goal (not FAULT)."""

        if goal == 'HOME':
            self._process_home()
            return

        if goal == 'STAIR':
            self._process_stair()
            return

        # Check for a priority pose before starting (e.g. one arrived while
        # this goal was queued)
        if self._home_requested.is_set():
            self.get_logger().info('Goal aborted: priority pose requested.')
            self._process_priority_goal()
            return

        x, y, z = goal
        self.get_logger().info(
            f'Processing goal: ({x:.4f}, {y:.4f}, {z:.4f}) '
            f'pitch={ee_pitch:.3f} roll={ee_roll:.3f} tele={ee_telescope:.4f}  '
            f'last_valid=({self._last_valid_goal[0]:.4f}, '
            f'{self._last_valid_goal[1]:.4f}, {self._last_valid_goal[2]:.4f})')

        # ----- VALIDATE (closed-form IK + delta + collision) -------------
        self.state = State.VALIDATE
        self._planning_attempts = 1   # closed-form IK is single-shot

        ik_solution = self._solve_ik_with_retries(
            x, y, z, ee_roll=ee_roll,
            ee_pitch=ee_pitch, ee_telescope=ee_telescope)
        if self._home_requested.is_set():
            self.get_logger().info(
                'VALIDATE aborted: priority pose requested.')
            self._process_priority_goal()
            return
        if ik_solution is None:
            self._rollback(f'IK unreachable at ({x:.3f}, {y:.3f}, {z:.3f})')
            return

        self.get_logger().info(
            f'IK solution: ' +
            ', '.join(f'{k}={v:.3f}' for k, v in ik_solution.items()))

        # ----- PLANNING (single-point trajectory; no OMPL) ---------------
        # The closed-form IK already produces a unique, valid solution.
        # We linearly interpolate from current joint state to the target
        # in joint space.  No OMPL planning, no orientation constraint —
        # wrist_pan is computed directly from the desired pitch.
        if self._pipeline_stale:
            self.get_logger().info(
                'Goal dropped: inputs released during IK.')
            self._pipeline_stale = False
            self.state = State.IDLE
            return

        self.state = State.PLANNING
        trajectory = self._build_single_point_trajectory(ik_solution)

        if self._home_requested.is_set():
            self.get_logger().info(
                'PLANNING aborted: priority pose requested.')
            self._process_priority_goal()
            return
        if trajectory is None:
            self._rollback(
                f'Trajectory build failed at ({x:.3f}, {y:.3f}, {z:.3f}) '
                f'pitch={ee_pitch:.3f} roll={ee_roll:.3f}')
            return

        # ----- EXECUTING (non-blocking) -----------------------------------
        # Check if inputs were released while we were in IK/planning.
        # If so, the trajectory is stale — don't execute.
        if self._pipeline_stale:
            self.get_logger().info(
                'Goal dropped: inputs released during pipeline.')
            self._pipeline_stale = False
            self.state = State.IDLE
            return

        self.state = State.EXECUTING
        self._execute_trajectory(trajectory)

        # ----------- FK-anchor the accumulator to actual wrist position ---
        # In the singular region near home, the IK projects the commanded
        # target onto the slewed turret heading, so the wrist actually ends
        # up at FK(joints) — not at `goal`.  If we anchor _last_valid_goal
        # to the COMMANDED goal, then on subsequent ticks the accumulator
        # carries the un-tracked X/Y "demand" forward, and a pure-Y push
        # after a partially-tracked +X push drags the X demand along —
        # producing the symptom: "Y command also moves wrist in X".
        #
        # Anchoring to FK-actual instead means each tick's accumulator
        # starts from where the wrist truly is.  Uncommanded axes hold;
        # commanded axes accumulate from reality, not from a stale demand.
        try:
            F = fk_mod.fk(
                ik_solution['turret_Joint'],
                ik_solution['shoulder_Joint'],
                ik_solution['elbow_Joint'],
                ik_solution['telescope_Joint'],
                ik_solution.get('wrist_pan_Joint', 0.0),
                ik_solution.get('wrist_roll_Joint', 0.0),
            )
            wp = fk_mod.pos(F, 'wrist_pan')
            anchored_goal = (float(wp[0]), float(wp[1]), float(wp[2]))
        except Exception as e:
            # FK should never fail with valid joints, but if it does, fall
            # back to the commanded goal so we don't break dispatch.
            self.get_logger().warning(
                f'FK anchor failed ({e}); using commanded goal as last_valid.')
            anchored_goal = goal

        # ── Always publish the latest IK solution ──────────────────────
        # The trajectory was already sent above.  Now update the publisher
        # target so /arm_joint_commands reflects the latest command.  The
        # hardware bridge needs the command regardless of whether the
        # arrival check passes.  Save the PREVIOUS target first — the
        # arrival check needs it as the reference position.
        prev_arm_target = self._latest_arm_joint_target
        self._latest_arm_joint_target = dict(ik_solution)

        # ── Hardware arrival check ──────────────────────────────────────
        # Verify the physical joints have reached (within tolerance) the
        # PREVIOUS IK target (not the one just published).  If the
        # hardware hasn't caught up, rollback to the last verified
        # position and require re-arm (operator must release all sticks
        # before motion resumes).
        _DELTA_ARRIVAL_JOINTS = [j for j in ARM_JOINTS
                                 if j != 'wrist_roll_Joint']
        hardware_arrived = True
        arrival_info = ''
        if prev_arm_target is not None:
            with self._js_lock:
                js = self._last_joint_state
            if js is not None:
                phys_joints = {}
                for i, jname in enumerate(js.name):
                    if jname in ARM_JOINTS:
                        phys_joints[jname] = js.position[i]
                max_arr_delta = 0.0
                worst_arr_joint = ''
                for j in _DELTA_ARRIVAL_JOINTS:
                    prev_val = prev_arm_target.get(j, 0.0)
                    phys_val = phys_joints.get(j, 0.0)
                    d = abs(prev_val - phys_val)
                    if d > max_arr_delta:
                        max_arr_delta = d
                        worst_arr_joint = j
                if max_arr_delta > self._goal_arrival_tolerance:
                    hardware_arrived = False
                    arrival_info = (
                        f'{worst_arr_joint} delta {max_arr_delta:.3f} rad '
                        f'from previous target '
                        f'(tolerance {self._goal_arrival_tolerance:.3f})')

        if hardware_arrived:
            # Success: advance _last_valid to the FK-anchored position.
            self._last_valid_goal = anchored_goal
            self._last_valid_pitch = ee_pitch
            self._last_valid_roll = ee_roll
            self._last_valid_telescope = ee_telescope
            self.get_logger().info(
                f'Goal succeeded: cmd=({x:.4f}, {y:.4f}, {z:.4f}) '
                f'fk=({anchored_goal[0]:.4f}, {anchored_goal[1]:.4f}, '
                f'{anchored_goal[2]:.4f}) pitch={ee_pitch:.3f} '
                f'roll={ee_roll:.3f} tele={ee_telescope:.4f} '
                f'— last_valid anchored to FK')
        else:
            # Hardware hasn't reached the previous target.  The command
            # and trajectory have already been sent (so the hardware has
            # the latest target), but we rollback the accumulators and
            # require re-arm so the operator must release all sticks
            # before further motion.
            self._rollback(f'Hardware lagging: {arrival_info}')
            self.get_logger().warn(
                f'Goal sent + rollback: cmd=({x:.4f}, {y:.4f}, {z:.4f}) '
                f'fk=({anchored_goal[0]:.4f}, {anchored_goal[1]:.4f}, '
                f'{anchored_goal[2]:.4f}) — {arrival_info}')

        self.state = State.IDLE

    def _process_priority_goal(self):
        """Run whichever priority pose is pending.  HOME and STAIR share the
        _home_requested event as their pre-emption signal, so the pending
        goal is what distinguishes them."""
        with self._goal_lock:
            pending = self._pending_goal
        if pending == 'STAIR':
            self._process_stair()
        else:
            self._process_home()

    def _move_arm_to_pose(self, target_joints, label='HOME'):
        """Plan (with retries) and execute a joint-space move to
        *target_joints*, falling back to direct joint-space interpolation
        when OMPL cannot find a path.  Shared by HOME and STAIR.

        Leaves _latest_arm_joint_target set to the commanded pose so the
        /arm_joint_commands stream — which is what actually drives the real
        hardware — carries the pose too, not just the action.
        """
        self.state = State.PLANNING
        self._planning_attempts = 0

        # Multiple retries with increasing planning time.  This handles
        # configurations that are hard to plan from (e.g. wrist_roll at pi
        # from a previous IK).
        trajectory = None
        timeouts = [5.0, 10.0, 15.0]
        for i, timeout in enumerate(timeouts):
            self.get_logger().info(
                f'{label} planning attempt {i+1}/{len(timeouts)} '
                f'(timeout={timeout}s)')
            trajectory = self._plan_to_joint_state(
                target_joints, interruptible=False, timeout_override=timeout)
            if trajectory is not None:
                break
            self.get_logger().warn(
                f'{label} planning attempt {i+1} failed, '
                f'{"retrying..." if i < len(timeouts)-1 else "giving up."}')

        if trajectory is not None:
            self.get_logger().info(f'{label}: using planned trajectory.')
        else:
            # OMPL exhausted — fall back to direct joint-space interpolation.
            # No collision checking: the operator asked for a priority pose.
            self.get_logger().warn(
                f'{label}: OMPL failed — falling back to direct trajectory.')
            trajectory = self._build_direct_joint_trajectory(
                target_joints, label=label)

        self.state = State.EXECUTING
        self._execute_trajectory(trajectory, is_home=True)

        # Keep the arm joint command topic in sync with the action.  A
        # priority pose bypasses the IK pipeline that would otherwise update
        # this, so without the explicit assignment /arm_joint_commands would
        # keep publishing the pre-move joint positions and a hardware bridge
        # subscribed only to the topic would never see the pose.
        self._latest_arm_joint_target = dict(target_joints)

    def _anchor_accumulators_to_pose(self, joints):
        """Reset the operator accumulators to a commanded joint pose.

        Position anchors to FK of the pose (so it stays correct for any
        pose, not just home), and pitch/roll/telescope invert the same
        relations _sync_state_from_joints uses.
        """
        try:
            F = fk_mod.fk(joints['turret_Joint'], joints['shoulder_Joint'],
                          joints['elbow_Joint'], joints['telescope_Joint'],
                          joints['wrist_pan_Joint'],
                          joints['wrist_roll_Joint'])
            wp = fk_mod.pos(F, 'wrist_pan')
            anchor = (float(wp[0]), float(wp[1]), float(wp[2]))
        except Exception as e:
            self.get_logger().warning(
                f'Pose anchor FK failed ({e}); falling back to the '
                f'configured home_position.')
            anchor = self._home_pos

        pitch = -(joints['wrist_pan_Joint'] + joints['shoulder_Joint']
                  + joints['elbow_Joint'])
        roll = joints['wrist_roll_Joint']
        tele = joints['telescope_Joint']

        self._current_goal = anchor
        self._last_valid_goal = anchor
        self._last_accepted_goal = anchor
        self._current_pitch = pitch
        self._last_valid_pitch = pitch
        self._last_dispatched_pitch = pitch
        self._pending_pitch = 0.0
        self._current_roll = roll
        self._last_valid_roll = roll
        self._last_dispatched_roll = roll
        self._pending_roll = 0.0
        self._current_telescope = tele
        self._last_valid_telescope = tele
        self._last_dispatched_telescope = tele
        self._pending_telescope = 0.0

    def _process_stair(self):
        """STAIR arm leg: move the arm to the stair pose.

        The flipper leg was started at stair entry and is running on the
        flipper timer, so it advances in parallel with the planning here.
        _arm_tick latches HOLD once both legs report done.
        """
        self._home_requested.clear()          # consume the flag
        self.get_logger().info(
            'Processing STAIR: draining queue, planning arm pose...')

        with self._goal_lock:
            self._pending_goal = None
        while not self._event_queue.empty():
            try:
                self._event_queue.get_nowait()
            except Empty:
                break

        target = dict(self._stair_arm_joints)
        self._move_arm_to_pose(target, label='STAIR')
        self._anchor_accumulators_to_pose(target)

        # The arm no longer sits at the home pose (unless the stair pose IS
        # home), so a later HOME must not be deduplicated away.
        self._at_home = False
        self._stair_arm_done = True
        self.state = State.IDLE
        self.get_logger().info(
            'STAIR arm pose sent; waiting for arrival to latch.')

    def _process_home(self):
        """HOME command: bypass IK, plan directly to all-joints-zero.
        Runs non-interruptibly — once HOME starts, it completes.
        Clears the _home_requested flag at the start."""
        self._home_requested.clear()          # consume the flag
        self.get_logger().info('Processing HOME: draining queue, planning...')

        # Drain any stale GOAL events from the queue so we don't
        # immediately process another goal after HOME.
        with self._goal_lock:
            self._pending_goal = None
        while not self._event_queue.empty():
            try:
                self._event_queue.get_nowait()
            except Empty:
                break

        # Kick the flippers off FIRST, before arm planning.  The sequence is
        # stepped by the flipper timer, so it runs in parallel with the arm
        # homing below instead of waiting out up to 30 s of OMPL attempts.
        #
        # EXCEPT when this HOME is the FIRING-entry pose: firing homes the
        # ARM only.  The flippers must not move unless the operator
        # commands them by stick, so the automatic sequence is skipped and
        # _flipper_tick keeps serving manual input throughout.
        if self._firing_pending:
            self.get_logger().info(
                'HOME via FIRING entry: flippers left under manual control '
                '(no automatic flipper homing).')
        else:
            self._start_flipper_home()

        # Plan + execute the arm move (shared with STAIR).
        self._move_arm_to_pose(HOME_JOINTS, label='HOME')

        # Success: snap state to home.  Also clear re-arm — HOME is an
        # operator-initiated override, the operator gets a clean slate.
        self._current_goal = self._home_pos
        self._last_valid_goal = self._home_pos
        self._last_accepted_goal = self._home_pos
        self._current_pitch = 0.0
        self._last_valid_pitch = 0.0
        self._last_dispatched_pitch = 0.0
        self._pending_pitch = 0.0
        self._current_roll = 0.0
        self._last_valid_roll = 0.0
        self._last_dispatched_roll = 0.0
        self._pending_roll = 0.0
        self._current_telescope = 0.0
        self._last_valid_telescope = 0.0
        self._last_dispatched_telescope = 0.0
        self._pending_telescope = 0.0
        self._at_home = True
        self._rearm_required = False

        # If this HOME was triggered by FIRING entry, activate firing mode
        if self._firing_pending:
            self._firing_pending = False
            self._firing_active = True
            self._firing_wrist_pan = 0.0
            self.get_logger().info('FIRING mode active — wrist_pan direct control.')

        self.state = State.IDLE
        self.get_logger().info('Home trajectory sent.')

    def _build_direct_joint_trajectory(self, target_joints, label='HOME'):
        """Build a simple joint-space trajectory from the current position to
        *target_joints*.  Bypasses MoveIt entirely — no collision checking.

        Used as the guaranteed fallback when OMPL can't find a path to a
        priority pose.  Trajectory time is computed from the largest joint
        displacement and a conservative speed, ensuring smooth motion.
        """
        # Read current joint positions
        current = {}
        with self._js_lock:
            js = self._last_joint_state
        if js is not None:
            for i, jname in enumerate(js.name):
                if jname in ARM_JOINTS:
                    current[jname] = js.position[i]
        # Fill any missing joints with the target (no motion on that joint)
        # rather than 0 — assuming 0 would command an unrelated joint to
        # move when we simply have no feedback for it.
        for j in ARM_JOINTS:
            if j not in current:
                current[j] = target_joints[j]

        # Compute duration from max displacement.
        # Conservative speed: 0.5 rad/s revolute, 0.05 m/s prismatic.
        max_time = 2.0  # minimum 2 seconds
        for j in ARM_JOINTS:
            displacement = abs(current[j] - target_joints[j])
            if j == 'telescope_Joint':
                t = displacement / 0.05   # prismatic, slow
            else:
                t = displacement / 0.5    # revolute, conservative
            max_time = max(max_time, t)
        # Round up with margin
        duration_sec = int(max_time + 1.5)

        self.get_logger().info(
            f'Direct {label} trajectory: duration={duration_sec}s, '
            f'joints={", ".join(f"{j}={current[j]:.3f}->{target_joints[j]:.3f}" for j in ARM_JOINTS)}')

        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS

        # Start point (current)
        p0 = JointTrajectoryPoint()
        p0.positions = [current[j] for j in ARM_JOINTS]
        p0.velocities = [0.0] * len(ARM_JOINTS)
        p0.time_from_start = Duration(sec=0, nanosec=0)
        traj.points.append(p0)

        # End point (target pose)
        p1 = JointTrajectoryPoint()
        p1.positions = [target_joints[j] for j in ARM_JOINTS]
        p1.velocities = [0.0] * len(ARM_JOINTS)
        p1.time_from_start = Duration(sec=duration_sec, nanosec=0)
        traj.points.append(p1)

        # Wrap in RobotTrajectory so _execute_trajectory can use it
        rt = RobotTrajectory()
        rt.joint_trajectory = traj
        return rt

    def _build_single_point_trajectory(self, ik_solution):
        """Build a 2-point JointTrajectory from current joint state to the
        IK solution.  Duration is set so the fastest joint moves at no more
        than the configured speed limit.  Returns a RobotTrajectory.

        Speed limits are conservative because there are no intermediate
        waypoints — the controller interpolates linearly in joint space.
        Tune via constants below if needed.
        """
        # Read current joint positions, hardware velocities (if populated),
        # and our own finite-difference velocity estimates.  Hardware on
        # this robot reports velocity as zero even while the joint is
        # moving, so we have to derive it from the position stream
        # ourselves — used below to set p0.velocities in the trajectory.
        current = {}
        current_vel = {}
        with self._js_lock:
            js = self._last_joint_state
            fd_vel = dict(self._joint_velocities)        # snapshot under lock
        if js is None:
            self.get_logger().warn('Cannot build trajectory: no joint state.')
            return None
        has_hw_vel = len(js.velocity) >= len(js.name)
        for i, jname in enumerate(js.name):
            if jname in ARM_JOINTS:
                current[jname] = js.position[i]
                v_hw = js.velocity[i] if has_hw_vel else 0.0
                # Prefer hardware velocity when it's actually non-zero,
                # otherwise fall back to our FD estimate.
                current_vel[jname] = (
                    v_hw if abs(v_hw) > 1e-9
                    else fd_vel.get(jname, 0.0))
        if len(current) != len(ARM_JOINTS):
            self.get_logger().warn('Cannot build trajectory: joint state incomplete.')
            return None

        # Compute duration from max joint displacement.
        REVOLUTE_SPEED  = 0.5    # rad/s — conservative, no waypoints to validate
        PRISMATIC_SPEED = 0.05   # m/s
        MIN_DURATION    = 0.1    # seconds
        max_time = MIN_DURATION
        worst_j = None
        for j in ARM_JOINTS:
            disp = abs(ik_solution[j] - current.get(j, 0.0))
            speed = PRISMATIC_SPEED if j == 'telescope_Joint' else REVOLUTE_SPEED
            t = disp / speed
            if t > max_time:
                max_time = t
                worst_j = j
        duration_nsec = int(max_time * 1e9)

        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS

        p0 = JointTrajectoryPoint()
        p0.positions = [current[j] for j in ARM_JOINTS]
        # Use actual current velocity so the controller doesn't reset to zero
        # on every preempt.  See the diagnostic comment above the joint-state
        # read.
        p0.velocities = [current_vel.get(j, 0.0) for j in ARM_JOINTS]
        p0.time_from_start = Duration(sec=0, nanosec=0)
        traj.points.append(p0)

        p1 = JointTrajectoryPoint()
        p1.positions = [ik_solution[j] for j in ARM_JOINTS]
        p1.velocities = [0.0] * len(ARM_JOINTS)
        p1.time_from_start = Duration(
            sec=duration_nsec // 1_000_000_000,
            nanosec=duration_nsec % 1_000_000_000)
        traj.points.append(p1)

        rt = RobotTrajectory()
        rt.joint_trajectory = traj

        # Detailed trajectory log — every dispatch.  Useful for diagnosing
        # any unexpected jump.  Telescope is called out separately because
        # it's been the source of intermittent issues.
        tcur = current.get('telescope_Joint', 0.0)
        tvel = current_vel.get('telescope_Joint', 0.0)
        ttgt = ik_solution.get('telescope_Joint', 0.0)
        tdisp = ttgt - tcur     # signed: negative = extending in URDF
        tele_warn = ''
        if abs(tdisp) > 0.020:  # >2cm telescope move in a single trajectory
            tele_warn = '  *** LARGE TELESCOPE MOVE ***'
        self.get_logger().info(
            f'Trajectory build: dur={max_time*1000:.0f}ms '
            f'worst={worst_j}  '
            f'telescope: {tcur:+.4f}@{tvel*1000:+.0f}mm/s -> {ttgt:+.4f} '
            f'({tdisp*1000:+.1f}mm){tele_warn}')
        # Per-joint detail at debug level only (chatty).
        per_joint = '  '.join(
            f'{j.replace("_Joint",""):8s} {current.get(j,0):+.4f} -> '
            f'{ik_solution.get(j,0):+.4f} (Δ={ik_solution.get(j,0)-current.get(j,0):+.4f})'
            for j in ARM_JOINTS)
        self.get_logger().debug(f'Trajectory per-joint: {per_joint}')
        return rt

    # ======================================================================
    # IK (closed-form, single-shot)
    # ======================================================================

    def _solve_ik_with_retries(self, x, y, z,
                               ee_roll=0.0, ee_pitch=0.0, ee_telescope=0.0):
        """Closed-form IK + delta + collision check.  Single-shot: there is
        no random-seed retry because the solver is deterministic.  Name kept
        for diff continuity with the old MoveIt-backed version.

        Returns {joint_name: position} on success, or None on any failure.
        Failures are logged with full diagnostic context — see _log_ik_failure.
        """
        # Read current joints once for current_turret (dead-zone hold) and
        # the joint-delta safety check.
        current_joints = {}
        with self._js_lock:
            js = self._last_joint_state
        if js is not None:
            for i, jname in enumerate(js.name):
                if jname in ARM_JOINTS:
                    current_joints[jname] = js.position[i]

        current_turret = current_joints.get('turret_Joint', 0.0)

        # Closed-form solve
        result = arm_ik.solve(
            x=x, y=y, z=z,
            telescope=ee_telescope,
            pitch_world=ee_pitch,
            roll_world=ee_roll,
            current_turret=current_turret,
            turret_scale_r=self._turret_scale_r,
        )
        if not result.success:
            self._log_ik_failure(result, current_joints, kind='solver')
            return None

        solution = result.joints

        # Joint delta check: reject solutions with large jumps.
        # wrist_roll_Joint is excluded because it is a direct operator
        # override — the IK solver stamps the accumulated ee_roll value
        # into the solution unchanged.  Its delta against the physical
        # joint state is expected to grow during continuous roll commands
        # (the physical motor lags the commanded position).  Including it
        # here would trigger a false rollback after a few seconds of
        # sustained roll input, locking out accumulation via _rearm_required
        # until the operator releases all sticks.
        _DELTA_CHECK_JOINTS = [j for j in ARM_JOINTS
                               if j != 'wrist_roll_Joint']
        if current_joints:
            max_delta = 0.0
            worst_joint = ''
            per_joint_delta = {}
            for j in _DELTA_CHECK_JOINTS:
                delta = abs(solution[j] - current_joints.get(j, 0.0))
                per_joint_delta[j] = delta
                if delta > max_delta:
                    max_delta = delta
                    worst_joint = j
            if max_delta > self._max_joint_delta:
                self._log_ik_failure(
                    result, current_joints, kind='delta',
                    extra={'worst_joint': worst_joint,
                           'max_delta': max_delta,
                           'limit': self._max_joint_delta,
                           'per_joint_delta': per_joint_delta})
                return None

        # Call /check_state_validity (validates full configuration)
        if not self._check_collision(solution):
            self._log_ik_failure(result, current_joints, kind='collision')
            return None

        return solution

    def _log_ik_failure(self, result, current_joints, kind='solver', extra=None):
        """Emit a multi-line WARN with full IK failure context.

        kind:    'solver'    — arm_ik.solve() rejected
                 'delta'     — IK ok but joint jump exceeded max_joint_delta
                 'collision' — IK ok but pose in collision
        extra:   optional dict with kind-specific fields (e.g. delta info).

        The diagnostics dict from IKResult is the source of truth for the
        solver-internal values; this method just formats them and adds
        coordinator-side context (current joints, last-valid goal, etc.).
        """
        d = result.diagnostics or {}

        def fmt(key, fmt_spec='+.4f'):
            """Pretty-print a diagnostics value or '?' if not yet set."""
            if key not in d:
                return '?'
            v = d[key]
            if isinstance(v, bool):
                return str(v)
            try:
                return f'{v:{fmt_spec}}'
            except (TypeError, ValueError):
                return str(v)

        lines = []
        lines.append(f'=== IK FAILURE ({kind}): {result.failure_reason or kind} ===')

        # ---- Inputs / context -------------------------------------------
        lines.append(
            f'  Target:           ({fmt("input_x", "+.4f")}, '
            f'{fmt("input_y", "+.4f")}, {fmt("input_z", "+.4f")})')
        lines.append(
            f'  EE commands:      pitch_world={fmt("input_pitch_world")}  '
            f'roll_world={fmt("input_roll_world")}  '
            f'telescope={fmt("input_telescope")}')
        lines.append(
            f'  current_turret:   {fmt("input_current_turret")}')
        lines.append(
            f'  Last valid goal:  ({self._last_valid_goal[0]:+.4f}, '
            f'{self._last_valid_goal[1]:+.4f}, {self._last_valid_goal[2]:+.4f})  '
            f'pitch={self._last_valid_pitch:+.4f}  '
            f'roll={self._last_valid_roll:+.4f}  '
            f'tele={self._last_valid_telescope:+.4f}')
        if current_joints:
            cj = '  '.join(
                f'{j.replace("_Joint",""):>8}={current_joints.get(j, 0.0):+.4f}'
                for j in ARM_JOINTS)
            lines.append(f'  Current joints:   {cj}')

        # ---- Stage 1: turret --------------------------------------------
        lines.append(
            f'  [Turret]   r={fmt("r","+.4f")}m '
            f'scale={fmt("scale_factor","+.3f")} '
            f'max_step={fmt("max_step_effective","+.3f")} '
            f'turret_q={fmt("turret_q")} '
            f'in_range={fmt("turret_in_range")}')
        if 'y_local' in d:
            lines.append(
                f'  [Plane]    y_local={fmt("y_local")} '
                f'(signed: {"forward" if d["y_local"] >= 0 else "behind"})  '
                f'x_local={fmt("x_local")} (proj err)  '
                f'h={fmt("h")}  D={fmt("D","+.4f")}')

        # ---- Stage 2: triangle / elbow / shoulder -----------------------
        if 'D_min' in d:
            lines.append(
                f'  [Reach]    D_min={fmt("D_min","+.4f")}  '
                f'D_max={fmt("D_max","+.4f")}  '
                f'forearm L={fmt("L_forearm","+.4f")}  '
                f'elbow_offset_t={fmt("elbow_offset_t")}')
        if 'cos_vertex' in d:
            lines.append(
                f'  [Triangle] cos_vertex={fmt("cos_vertex","+.4f")}  '
                f'vertex_angle={fmt("vertex_angle")}')
        if 'elbow_q_raw' in d:
            lines.append(
                f'  [Elbow]    raw={fmt("elbow_q_raw")} '
                f'final={fmt("elbow_q_final")} '
                f'clipped={fmt("elbow_clipped")} '
                f'(URDF: [{fmt("elbow_q_min")}, {fmt("elbow_q_max")}])')
        if 'shoulder_q_raw' in d:
            lines.append(
                f'  [Shoulder] alpha={fmt("alpha")} psi={fmt("psi")} '
                f'beta={fmt("beta")}')
            lines.append(
                f'             raw={fmt("shoulder_q_raw")} '
                f'final={fmt("shoulder_q_final")} '
                f'clipped={fmt("shoulder_clipped")} '
                f'(URDF: [{fmt("shoulder_q_min")}, {fmt("shoulder_q_max")}])')

        # ---- Stage 3: wrist pan -----------------------------------------
        if 'raw_pan' in d:
            lines.append(
                f'  [Wrist_pan] raw={fmt("raw_pan")} wrapped={fmt("wrist_pan_q")} '
                f'in_range={fmt("wrist_pan_in_range")}')

        # ---- Kind-specific context --------------------------------------
        if kind == 'delta' and extra:
            lines.append(f'  [Delta]    worst_joint={extra["worst_joint"]} '
                         f'max_delta={extra["max_delta"]:+.4f} '
                         f'limit={extra["limit"]:+.4f}')
            pj = '  '.join(
                f'{j.replace("_Joint",""):>8}={extra["per_joint_delta"][j]:+.4f}'
                for j in extra["per_joint_delta"])
            lines.append(f'             per_joint:    {pj}')
        elif kind == 'collision' and result.joints:
            sol = '  '.join(
                f'{j.replace("_Joint",""):>8}={result.joints[j]:+.4f}'
                for j in ARM_JOINTS)
            lines.append(f'  [Solution] {sol}')

        # Single warn call so the message stays grouped in the log
        self.get_logger().warn('\n'.join(lines))

    def _check_collision(self, joint_dict, group_name=None, timeout_sec=5.0):
        """Call /check_state_validity.  Returns True if collision-free.

        group_name controls which collision pairs MoveIt evaluates:
          - None (default): use self._planning_group (= 'arm') — only
            pairs involving an arm link are checked.  Suitable for arm
            and firing checks.
          - '' (empty): check ALL non-disabled pairs in the robot.
            Required for flipper-vs-flipper checks (neither link is in
            the arm group, so 'arm' silently skips that pair).
          - any string: use that group.

        timeout_sec controls how long to wait for a response.  Use a
        small value (e.g. 0.2) for tick-rate calls so a slow MoveIt
        doesn't stall the executor; use the default for one-shot checks.
        Returns False on timeout (treated as not-safe).
        """

        req = GetStateValidity.Request()
        req.group_name = (self._planning_group if group_name is None
                          else group_name)
        rs = RobotState()
        rs.joint_state.name = list(joint_dict.keys())
        rs.joint_state.position = list(joint_dict.values())
        req.robot_state = rs

        future = self._validity_client.call_async(req)
        result = self._wait_for_future(future, timeout_sec=timeout_sec)

        if result is None:
            # Best-effort cancel so MoveIt doesn't keep working on a
            # request whose answer we'll never use.
            try:
                future.cancel()
            except Exception:
                pass
            self.get_logger().warn(
                f'State validity service call timed out after '
                f'{timeout_sec:.2f}s — treating as not-safe.')
            return False

        return result.valid

    def _plan_to_joint_state(self, joint_dict, interruptible=True,
                             timeout_override=None, ee_pitch=0.0):
        """Call /plan_kinematic_path with joint-space goal constraints.
        When ee_pitch is non-zero, loosens wrist_pan_Joint constraint and
        adds an OrientationConstraint so OMPL adjusts wrist_pan to achieve
        the desired end-effector pitch.
        Returns a RobotTrajectory on success, or None."""

        plan_timeout = timeout_override or self._planning_timeout
        has_pitch = abs(ee_pitch) > 0.01

        req = GetMotionPlan.Request()
        mpr = req.motion_plan_request
        mpr.group_name = self._planning_group
        mpr.num_planning_attempts = 10 if (timeout_override or has_pitch) else 5
        mpr.allowed_planning_time = plan_timeout
        mpr.max_velocity_scaling_factor = 1.0
        mpr.max_acceleration_scaling_factor = 1.0

        # Start state: empty -> MoveIt uses current state

        # Goal: joint-space constraints
        goal_constraints = Constraints()
        for joint_name, position in joint_dict.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = position
            # When orientation is requested, loosen shoulder, elbow, and
            # wrist_pan so OMPL can adjust them to satisfy the
            # OrientationConstraint while maintaining approximate position.
            # Shoulder and elbow compensate for the position shift caused
            # by wrist_pan adjustment.  Telescope is IK-free (automatic).
            # Turret stays tight (base rotation unchanged).
            if joint_name == 'wrist_pan_Joint' and has_pitch:
                jc.tolerance_above = 3.15   # full range — orientation DOF
                jc.tolerance_below = 3.15
            elif joint_name == 'shoulder_Joint' and has_pitch:
                jc.tolerance_above = 0.15   # ~8.6° slack for position compensation
                jc.tolerance_below = 0.15
            elif joint_name == 'elbow_Joint' and has_pitch:
                jc.tolerance_above = 0.15   # ~8.6° slack for position compensation
                jc.tolerance_below = 0.15
            else:
                jc.tolerance_above = 0.001
                jc.tolerance_below = 0.001
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)

        # Orientation constraint: desired tilt of end-effector link
        # Due to the kinematic chain (multiple 180° flips), the operator's
        # "pitch" (tilt up/down) maps to the ROLL component in base_link
        # RPY decomposition.  wrist_pan_Joint rotates around its local X
        # axis, which after chain transforms appears as roll in base_link.
        #
        # Desired orientation: RPY(ee_pitch, 0, π)
        # Quaternion formula for RPY(r, 0, π):
        #   qx = 0, qy = sin(r/2), qz = cos(r/2), qw = 0
        if has_pitch:
            oc = OrientationConstraint()
            oc.header.frame_id = 'base_link'
            oc.link_name = self._ee_link
            half_r = ee_pitch / 2.0
            oc.orientation.x = 0.0
            oc.orientation.y = math.sin(half_r)
            oc.orientation.z = math.cos(half_r)
            oc.orientation.w = 0.0
            oc.absolute_x_axis_tolerance = 0.15   # roll (= operator tilt): tight
            oc.absolute_y_axis_tolerance = 3.15   # pitch: free
            oc.absolute_z_axis_tolerance = 3.15   # yaw: free (covers turret)
            oc.weight = 1.0
            goal_constraints.orientation_constraints.append(oc)
            self.get_logger().info(
                f'Orientation constraint: tilt={ee_pitch:.3f} rad '
                f'({math.degrees(ee_pitch):.1f}°) '
                f'quat=({oc.orientation.x:.3f}, {oc.orientation.y:.3f}, '
                f'{oc.orientation.z:.3f}, {oc.orientation.w:.3f})')

        mpr.goal_constraints.append(goal_constraints)

        # Non-blocking call
        future = self._plan_client.call_async(req)
        result = self._wait_for_future(
            future, timeout_sec=plan_timeout + 5.0,
            interruptible=interruptible)

        if result is None:
            self.get_logger().warn('Planning service call timed out.')
            return None

        resp = result.motion_plan_response
        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(
                f'Planning failed: error code {resp.error_code.val}')
            return None

        if len(resp.trajectory.joint_trajectory.points) == 0:
            self.get_logger().warn('Planning returned empty trajectory.')
            return None

        return resp.trajectory

    def _execute_trajectory(self, trajectory, is_home=False):
        """Send trajectory to arm_controller (non-blocking).
        The controller will preempt any running trajectory.
        Stores the goal handle so HOME can cancel it later.
        Rejects trajectories that violate velocity limits (except HOME)."""

        # Velocity safety check (skip for HOME — must always succeed)
        if not is_home and not self._check_trajectory_velocity(trajectory):
            self.get_logger().error(
                'Trajectory rejected: velocity limit exceeded.')
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory.joint_trajectory

        # Non-blocking: send and don't wait for execution result.
        # HOME trajectory waits non-interruptibly so it can't be pre-empted by itself.
        send_future = self._traj_client.send_goal_async(goal)
        result = self._wait_for_future(
            send_future, timeout_sec=5.0, interruptible=not is_home)

        if result is None:
            self.get_logger().warn('Failed to send trajectory goal.')
            return

        if not result.accepted:
            self.get_logger().warn('Trajectory goal rejected by controller.')
            return

        # Store the goal handle so _on_sbus can cancel it on HOME
        with self._arm_handle_lock:
            self._current_arm_goal_handle = result

        # Late-cancel guard: if release fired during send_goal_async (between
        # send and getting the handle), the cancel call in the release block
        # saw a stale handle (None or previous) and didn't cancel this goal.
        # Now that we have the real handle stored, check the stale flag and
        # cancel here as well.  Without this, a release racing with send
        # leaves the trajectory uncancelled — visible to the operator as a
        # large residual motion after they let go.
        if not is_home and self._pipeline_stale:
            try:
                result.cancel_goal_async()
                self.get_logger().info(
                    'Late cancel: pipeline went stale during send.')
            except Exception as e:
                self.get_logger().warn(f'Late cancel failed: {e}')
            with self._arm_handle_lock:
                self._current_arm_goal_handle = None
            return

        kind = 'HOME' if is_home else 'trajectory'
        self.get_logger().info(
            f'{kind} sent ({len(goal.trajectory.points)} points).')
        # Return immediately — controller handles execution.
        # Next goal will preempt this one for fluid motion.


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)

    node = CoordinatorNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()