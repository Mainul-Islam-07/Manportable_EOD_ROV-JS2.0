#!/usr/bin/env python3
"""
arm_controller.py
=================
ROS 2 command bridge for the **Arm_CAN** bus motors.

This node does NOT own the CAN bus.  Motor objects are passed in by
``robot_bringup.py`` (which runs the heartbeat node in the same process).
Diagnostics are handled exclusively by ``motor_heartbeat_node.py``.

Loopback policy
---------------
Motors present in ``cfgs`` but absent from ``real_motors`` (missing at
startup or disabled at runtime) use *loopback*: IK commands are echoed
back as position feedback so the solver sees pseudo joint movement and
does not diverge.

``_motor_state`` (built from cfgs — all motors) holds command targets.
``_arm_state`` (built from real_motors — alive only) drives the CiA-402
state machine.  ``_read_csp_pos_cnt`` returns real telemetry for alive
motors and ``target_cnt`` (loopback) for the rest.
"""

import json
import math
import time as _time
from pathlib import Path
from threading import Lock

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from ros2_canbus.motor_config import MODE_CSP, MODE_CSV, MODE_CST


# =========================================================================
# Load config
# =========================================================================

CONFIG_FILE = str(Path(__file__).resolve().parent / "controller_config.json")

def _load_config():
    with open(CONFIG_FILE, 'r') as f:
        return json.load(f)

CFG = _load_config()
MY  = CFG['arm_controller']

# -- shared --
SM                       = CFG['state_machine']
IDLE_SECONDS             = float(SM['idle_seconds'])
RE_ARM_WAIT_S            = float(SM['re_arm_wait_s'])
EPS_CNT                  = int(SM['eps_cnt'])
EPS_VEL_CAN              = int(SM['eps_vel_can'])
EPS_TORQUE_CAN           = int(SM['eps_torque_can'])

RATES                    = CFG['rates_hz']
MOTOR_CMD_RATE_HZ        = float(RATES['motor_cmd'])

ROS_QUEUE_DEPTH          = int(CFG['ros_queue_depth'])

# -- arm-specific --
ROS_NODE_NAME            = MY['ros_node_name']

TOPICS                   = MY['topics']
TOPIC_ARM_CMD            = TOPICS['arm_cmd']
TOPIC_SBUS_JS            = TOPICS['sbus_js']
TOPIC_FLIPPER_FB         = TOPICS['flipper_feedback']
TOPIC_MOTOR_COMMANDS     = TOPICS['motor_commands']
TOPIC_JOINT_STATES       = TOPICS['joint_states']

ARM_JOINTS               = MY['joints']['arm_joints']
FLIPPER_JOINTS           = MY['joints']['flipper_joints']

SJS_IDX_GRIPPER          = int(MY['sbus_indices']['gripper_effort'])

PARAMS                   = MY['parameters']

TOPIC_ARM_FAULT_STOP   = CFG['heartbeat_node']['topics']['arm_fault_stop']
TOPIC_DRIVE_FAULT_STOP = CFG['heartbeat_node']['topics']['drive_fault_stop']
TOPIC_ARM_ACTIVE       = CFG['heartbeat_node']['topics']['arm_active_motors']

TOPIC_ARM_BATTERY_OK   = CFG['battery_monitor']['topics']['arm_battery_ok']

# Watchdog: if TPDO1 hasn't published within this window, the timer takes over
_JS_WATCHDOG_INTERVAL_S = 0.02   # 50 Hz timer
_JS_WATCHDOG_TIMEOUT_S  = 0.05   # 50 ms gap triggers fallback


# =========================================================================
# Motor controller node
# =========================================================================

class MotorController(Node):

    def __init__(self, real_motors: dict, cfgs: dict, start_latched: bool = False):
        super().__init__(ROS_NODE_NAME)

        self._real_motors = dict(real_motors)   # mutable copy
        self._cfg         = cfgs                # always ALL arm motor configs
        self._loopback_override = set()         # motors forced to loopback by heartbeat loss

        # Whole-arm-bus latch: once True, every arm joint is loopback and the
        # arm never sends CAN commands again (latched until process restart).
        # Set either at startup (a motor was missing at boot) or at runtime
        # (heartbeat loss on any arm motor → __ALL__ bus fault).
        self._arm_bus_latched = bool(start_latched)
        if self._arm_bus_latched:
            # Force every configured arm motor into loopback from the start.
            self._loopback_override = set(self._cfg.keys())

        self.declare_parameter('telescope_lead_m_per_rev',
                               float(PARAMS['telescope_lead_m_per_rev']))
        self.declare_parameter('csp_profile_vel_frac',
                               float(PARAMS['csp_profile_vel_frac']))
        self.declare_parameter('telescope_csp_profile_vel_frac',
                               float(PARAMS['telescope_csp_profile_vel_frac']))

        self._tele_lead     = float(self.get_parameter('telescope_lead_m_per_rev').value)
        self._vel_frac      = float(self.get_parameter('csp_profile_vel_frac').value)
        self._tele_vel_frac = float(self.get_parameter('telescope_csp_profile_vel_frac').value)

        self._motor_lock = Lock()

        # _motor_state: ALL motors (from cfgs) — targets always updated for loopback
        self._motor_state = {}
        for name, cfg in self._cfg.items():
            if cfg.mode == MODE_CSP:
                self._motor_state[name] = {
                    'mode': MODE_CSP, 'target_cnt': 0,
                    'vel_cnt': self._csp_profile_vel_cnt(name, cfg),
                }
            elif cfg.mode == MODE_CSV:
                self._motor_state[name] = {'mode': MODE_CSV, 'vel_cnt': 0}
            elif cfg.mode == MODE_CST:
                self._motor_state[name] = {'mode': MODE_CST, 'torque_cnt': 0}

        # _arm_state: only ALIVE motors — drives CiA-402 state machine
        self._arm_state = {}
        for name in self._real_motors:
            self._arm_state[name] = {
                'state': 'DISARMED', 'arming_complete_time': None,
                'last_run_value': None, 'last_active_time': None,
                'idle_since': None,
            }

        self._seed_csp_from_feedback()

        self._flipper_positions = [0.0] * len(FLIPPER_JOINTS)

        # Direct radian passthrough for loopback joints (no counts round-trip)
        # Order matches ARM_JOINTS: turret, shoulder, elbow, telescope, wrist_pan, wrist_roll
        self._loopback_positions = [0.0] * len(ARM_JOINTS)

        qos = QoSProfile(depth=ROS_QUEUE_DEPTH,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(JointState, TOPIC_ARM_CMD,    self._on_arm_cmd,    qos)
        self.create_subscription(JointState, TOPIC_SBUS_JS,    self._on_sbus_js,    qos)
        self.create_subscription(JointState, TOPIC_FLIPPER_FB, self._on_flipper_fb, qos)
        self.create_subscription(JointState, TOPIC_ARM_ACTIVE, self._on_arm_active, qos)
        self.create_subscription(String,     TOPIC_ARM_FAULT_STOP,   self._on_arm_fault,   qos)
        self.create_subscription(String,     TOPIC_DRIVE_FAULT_STOP, self._on_drive_fault, qos)

        # Encoder memory-battery status (latched).  A low arm battery means the
        # arm encoders' memory is unreliable → latch the whole arm bus off.
        batt_qos = QoSProfile(depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, TOPIC_ARM_BATTERY_OK,
                                 self._on_arm_battery, batt_qos)

        self._drive_fault_stop = False

        self._pub_motor_cmd    = self.create_publisher(JointState, TOPIC_MOTOR_COMMANDS, qos)
        self._pub_joint_states = self.create_publisher(JointState, TOPIC_JOINT_STATES,   qos)

        # TPDO1 interrupt (10 ms) is the primary joint-state driver
        self._last_js_publish = _time.monotonic()
        self._hook_tpdo1_trigger()

        # Watchdog timer: takes over joint-state publishing when TPDO1 is
        # unavailable (all arm motors missing at startup, or trigger motor
        # lost at runtime).
        self.create_timer(_JS_WATCHDOG_INTERVAL_S, self._js_watchdog_tick)

        self.create_timer(1.0 / MOTOR_CMD_RATE_HZ, self._command_tick)

        self.get_logger().info(
            f"motor_controller ready (Arm_CAN):\n"
            f"   active motors    : {sorted(self._real_motors.keys()) or '(none)'}\n"
            f"   loopback motors  : {sorted(set(self._cfg) - set(self._real_motors))}\n"
            f"   joint feedback   : {TOPIC_JOINT_STATES} (TPDO1 interrupt, 10 ms)\n"
            f"   flipper source   : {TOPIC_FLIPPER_FB}\n"
            f"   command snapshot : {TOPIC_MOTOR_COMMANDS} @ {MOTOR_CMD_RATE_HZ:.0f} Hz\n"
            f"   idle -> DISARM   : {IDLE_SECONDS:.1f} s")

    # -- seeding & helpers ------------------------------------------------

    def _seed_csp_from_feedback(self):
        for name, m in self._real_motors.items():
            cfg = self._cfg[name]
            if cfg.mode != MODE_CSP:
                continue
            pos_cnt = None
            try:
                pos_cnt = m.telemetry.data.feedback.position
            except Exception as e:
                self.get_logger().warn(f"{name}: seed read failed ({e})")
            if not isinstance(pos_cnt, int):
                self.get_logger().warn(f"{name}: seed not int ({pos_cnt!r}); using 0")
                pos_cnt = 0
            self._motor_state[name]['target_cnt'] = pos_cnt
            self._arm_state[name]['last_run_value'] = pos_cnt
            self.get_logger().info(f"{name}: CSP seeded at {pos_cnt} counts")

    def _csp_profile_vel_cnt(self, motor_name, cfg):
        frac = self._tele_vel_frac if motor_name == 'Telescopic' else self._vel_frac
        return int(round(frac * cfg.max_velocity))

    def _hook_tpdo1_trigger(self):
        """Hook TPDO1 postprocessing on the first real motor to trigger
        ``_publish_joint_states`` on each 10 ms CAN interrupt.
        By the time any motor's TPDO1 fires, the others on the same bus
        have already updated their telemetry from their own callbacks."""
        if not self._real_motors:
            return
        trigger_name = next(iter(self._real_motors))
        cb = self._real_motors[trigger_name].feedback.callback

        def on_tpdo1(position, velocity):
            self._publish_joint_states()

        cb.TPDO_1_Position_actual_value_Velocity_actual_value_Object_callback_postprocessing = on_tpdo1

    def _js_watchdog_tick(self):
        """Fallback joint-state publisher when TPDO1 is unavailable."""
        if not self._cfg:
            return
        if _time.monotonic() - self._last_js_publish > _JS_WATCHDOG_TIMEOUT_S:
            self._publish_joint_states()

    # -- setters ----------------------------------------------------------

    def _set_csp_target_rad(self, motor_name, q_rad):
        cfg = self._cfg[motor_name]
        cnt = cfg.clamp_position_cnt(cfg.rad_to_counts(q_rad))
        self._motor_state[motor_name] = {
            'mode': MODE_CSP, 'target_cnt': cnt,
            'vel_cnt': self._csp_profile_vel_cnt(motor_name, cfg),
        }

    def _set_cst_target_pct(self, motor_name, pct):
        cfg = self._cfg[motor_name]
        cnt = cfg.clamp_torque_cnt(int(round((pct / 100.0) * cfg.max_torque)))
        self._motor_state[motor_name] = {'mode': MODE_CST, 'torque_cnt': cnt}

    # -- subscription callbacks -------------------------------------------

    def _on_arm_cmd(self, msg):
        if not self._cfg:
            return
        pos = dict(zip(msg.name, msg.position))
        try:
            q_turret   = pos['turret_Joint']
            q_shoulder = pos['shoulder_Joint']
            q_elbow    = pos['elbow_Joint']
            q_tele_m   = pos['telescope_Joint']
            q_wp       = pos['wrist_pan_Joint']
            q_wr       = pos['wrist_roll_Joint']
        except KeyError as e:
            self.get_logger().warn(f"{TOPIC_ARM_CMD} missing {e}")
            return

        # Differential mix (keep in sync with the inverse in
        # _publish_joint_states):
        #   shoulder + -> L +, R -      elbow + -> L -, R -
        q_right_diff = -(q_shoulder + q_elbow)
        q_left_diff  =   q_shoulder - q_elbow

        tele_cfg  = self._cfg['Telescopic']
        tele_revs = q_tele_m / self._tele_lead if self._tele_lead != 0.0 else 0.0
        tele_cnt  = tele_cfg.clamp_position_cnt(
            int(round(tele_revs * tele_cfg.counts_per_output_rev)))

        with self._motor_lock:
            self._set_csp_target_rad('Turret',             q_turret)
            self._set_csp_target_rad('Right_Differential', q_right_diff)
            self._set_csp_target_rad('Left_Differential',  q_left_diff)
            self._set_csp_target_rad('Wrist',              q_wp)
            self._set_csp_target_rad('Gripper_360',        q_wr)
            self._motor_state['Telescopic'] = {
                'mode': MODE_CSP, 'target_cnt': tele_cnt,
                'vel_cnt': self._csp_profile_vel_cnt('Telescopic', tele_cfg),
            }
            # Store raw joint-space radians for exact loopback passthrough
            self._loopback_positions = [
                q_turret, q_shoulder, q_elbow, q_tele_m, q_wp, q_wr
            ]
            has_loopback = (bool(set(self._cfg) - set(self._real_motors))
                           or bool(self._loopback_override))

        # When loopback motors exist, publish immediately so the IK solver
        # sees feedback matching the command with zero lag.
        if has_loopback:
            self._publish_joint_states()

    def _on_sbus_js(self, msg):
        if not self._cfg:
            return
        if len(msg.effort) <= SJS_IDX_GRIPPER:
            self.get_logger().warn(
                f"{TOPIC_SBUS_JS} malformed (|eff|={len(msg.effort)})")
            return
        gripper_pct = float(msg.effort[SJS_IDX_GRIPPER])
        with self._motor_lock:
            self._set_cst_target_pct('Gripper', gripper_pct)
            t_cnt = self._motor_state['Gripper']['torque_cnt']
            arm_st = self._arm_state.get('Gripper', {}).get('state', 'NOT_IN_ARM_STATE')

    def _on_flipper_fb(self, msg):
        if len(msg.position) >= len(FLIPPER_JOINTS):
            self._flipper_positions = list(msg.position[:len(FLIPPER_JOINTS)])

    def _on_arm_active(self, msg):
        """Heartbeat node publishes active motor list on every state change.
        Immediately switch lost motors to loopback (no 3-second wait).
        Restore loopback override when heartbeat returns.

        Once the arm bus is latched (whole-bus heartbeat-loss policy), this
        is a no-op: loopback is permanent until process restart, so a
        returning heartbeat must NOT release any joint."""
        if self._arm_bus_latched:
            return
        with self._motor_lock:
            active_names = set(msg.name)
            for name in self._cfg:
                if name not in self._real_motors:
                    continue   # already permanently removed (fault)
                if name not in active_names:
                    if name not in self._loopback_override:
                        self._loopback_override.add(name)
                        self.get_logger().warn(
                            f"{name}: heartbeat lost → loopback")
                else:
                    if name in self._loopback_override:
                        self._loopback_override.discard(name)
                        self.get_logger().info(
                            f"{name}: heartbeat restored → real telemetry")

    def _is_real(self, motor_name):
        """True if motor is alive AND not overridden to loopback."""
        return (motor_name in self._real_motors
                and motor_name not in self._loopback_override)

    # -- fault handlers (arm vs drive are separate) -----------------------

    def _latch_arm_bus(self, reason: str):
        """Whole-arm-bus latch: disarm EVERY arm motor and put ALL arm joints
        into loopback, latched until process restart.  Never released.

        Shared by the heartbeat ``__ALL__`` fault and the memory-battery-low
        signal — both mean the arm bus can no longer be trusted to move."""
        if self._arm_bus_latched:
            return
        with self._motor_lock:
            self._arm_bus_latched = True
            # Disarm every real arm motor, then drop them all so the
            # command tick never sends CAN again and every joint reads
            # loopback.
            for name, m in list(self._real_motors.items()):
                try:
                    m.control.DISARM()
                except Exception as e:
                    self.get_logger().warn(
                        f"{name}: latch disarm error ({e})")
            self._real_motors.clear()
            self._arm_state.clear()
            self._loopback_override = set(self._cfg.keys())
        self.get_logger().error(
            f"ARM BUS LATCHED ({reason}). All arm joints now loopback, "
            f"latched until process restart.")
        self._publish_joint_states()

    def _on_arm_battery(self, msg):
        """Arm encoder memory-battery status.  ``False`` = below threshold:
        the arm encoders' retained memory is unreliable, so latch the whole
        arm bus off (disarm + loopback, until process restart).  A later
        recovery does NOT release the latch."""
        if msg.data:
            return   # battery OK
        if not self._arm_bus_latched:
            self._latch_arm_bus("arm memory battery LOW")

    def _on_arm_fault(self, msg):
        """Arm fault handler.

        Two cases:
          * ``__ALL__`` → whole-arm-bus heartbeat-loss latch: disarm EVERY
            arm motor and put ALL arm joints into loopback, latched until
            process restart.  Never released.
          * any other name → single-motor CiA-402 fault: disarm just that
            motor and loopback it (per-motor, unchanged legacy behavior).
        Drive is unaffected either way."""
        parts = msg.data.split('|', 1)
        motor_name = parts[0]
        reason = parts[1] if len(parts) > 1 else "unknown"

        # ── bus-wide latch ──
        if motor_name == "__ALL__":
            self._latch_arm_bus(reason)
            return

        # ── single-motor CiA-402 fault (per-motor, legacy) ──
        with self._motor_lock:
            if self._arm_bus_latched:
                return   # whole bus already down; nothing finer to do
            if motor_name not in self._real_motors:
                return   # already loopback or unknown motor

            # Disarm this specific motor
            try:
                self._real_motors[motor_name].control.DISARM()
            except Exception as e:
                self.get_logger().warn(
                    f"{motor_name}: fault disarm error ({e})")

            # Remove from real_motors → _read_csp_pos_cnt will use loopback
            del self._real_motors[motor_name]
            # Remove from arm_state → _command_tick won't send CAN commands
            self._arm_state.pop(motor_name, None)

        self.get_logger().error(
            f"ARM FAULT: {motor_name} ({reason}). "
            f"Switched to loopback. "
            f"Remaining active: {sorted(self._real_motors.keys()) or '(none)'}")

    def _on_drive_fault(self, msg):
        """Drive bus faulted — disarm ALL arm motors, full stop.
        Process will terminate shortly."""
        if self._drive_fault_stop:
            return
        self._drive_fault_stop = True
        self.get_logger().error(
            f"DRIVE FAULT received ({msg.data}). Disarming all arm motors.")
        with self._motor_lock:
            for name, m in self._real_motors.items():
                try:
                    m.control.DISARM()
                except Exception as e:
                    self.get_logger().warn(f"{name}: fault disarm error ({e})")
            for name in self._arm_state:
                self._arm_state[name]['state'] = 'FAULTED'
                self._arm_state[name]['idle_since'] = None
                self._arm_state[name]['arming_complete_time'] = None

    # -- command tick (ARM state machine + RUN dispatch) ------------------

    def _command_tick(self):
        if self._drive_fault_stop:
            return
        now = self.get_clock().now()

        # Phase 1: decide what to do under the lock (fast, no CAN I/O)
        actions = []
        with self._motor_lock:
            for name, arm in self._arm_state.items():
                # Skip motors forced to loopback (heartbeat lost)
                if name in self._loopback_override:
                    continue
                cfg   = self._cfg[name]
                state = self._motor_state[name]

                if cfg.mode == MODE_CSP:
                    cur = state['target_cnt']
                    lrv = arm['last_run_value']
                    non_idle = (lrv is None) or (abs(cur - lrv) > EPS_CNT)
                elif cfg.mode == MODE_CSV:
                    cur = state['vel_cnt'];      non_idle = abs(cur) > EPS_VEL_CAN
                elif cfg.mode == MODE_CST:
                    cur = state['torque_cnt'];    non_idle = abs(cur) > EPS_TORQUE_CAN
                else:
                    continue

                if arm['state'] == 'DISARMED':
                    if non_idle:
                        actions.append(('ARM', name, cfg.mode, None))
                elif arm['state'] == 'ARMING':
                    if arm['arming_complete_time'] and now >= arm['arming_complete_time']:
                        arm['state'] = 'ARMED'
                        arm['arming_complete_time'] = None
                        self.get_logger().info(f"{name}: ARMING -> ARMED")
                elif arm['state'] == 'ARMED':
                    if non_idle:
                        if cfg.mode == MODE_CSP:
                            actions.append(('RUN', name, cfg.mode,
                                            (state['target_cnt'], state['vel_cnt'])))
                        elif cfg.mode == MODE_CSV:
                            actions.append(('RUN', name, cfg.mode,
                                            (state['vel_cnt'],)))
                        elif cfg.mode == MODE_CST:
                            actions.append(('RUN', name, cfg.mode,
                                            (state['torque_cnt'],)))
                    else:
                        if arm['idle_since'] is None:
                            arm['idle_since'] = now
                            # Zero the output so the motor stops immediately
                            if cfg.mode == MODE_CST:
                                actions.append(('RUN', name, cfg.mode, (0,)))
                            elif cfg.mode == MODE_CSV:
                                actions.append(('RUN', name, cfg.mode, (0,)))
                        elapsed_s = (now - arm['idle_since']).nanoseconds * 1e-9
                        if elapsed_s >= IDLE_SECONDS:
                            actions.append(('DISARM', name, cfg.mode, None))

        # Phase 2: execute CAN commands outside the lock (slow SDO I/O)
        for action, name, mode, args in actions:
            try:
                if action == 'ARM':
                    self._real_motors[name].control.ARM()
                    with self._motor_lock:
                        self._arm_state[name]['arming_complete_time'] = (
                            now + Duration(seconds=RE_ARM_WAIT_S))
                        self._arm_state[name]['state'] = 'ARMING'
                        self._arm_state[name]['idle_since'] = None
                    self.get_logger().info(f"{name}: ARM -> ARMING")

                elif action == 'RUN':
                    if mode == MODE_CSP:
                        self._real_motors[name].mode.position.RUN(*args)
                    elif mode == MODE_CSV:
                        self._real_motors[name].mode.velocity.RUN(*args)
                    elif mode == MODE_CST:
                        self._real_motors[name].mode.torque.RUN(*args)
                    with self._motor_lock:
                        self._arm_state[name]['last_run_value'] = args[0]
                        self._arm_state[name]['last_active_time'] = now
                        self._arm_state[name]['idle_since'] = None

                elif action == 'DISARM':
                    self._real_motors[name].control.DISARM()
                    with self._motor_lock:
                        self._arm_state[name]['state'] = 'DISARMED'
                        self._arm_state[name]['idle_since'] = None
                        self._arm_state[name]['arming_complete_time'] = None
                    self.get_logger().info(f"{name}: idle -> DISARM")

            except Exception as e:
                self.get_logger().warn(f"{name}: {action} failed ({e})")

    # -- publishers -------------------------------------------------------

    def _read_csp_pos_cnt(self, motor_name):
        """Real telemetry for alive motors, loopback (target_cnt) for the rest."""
        if motor_name in self._real_motors:
            try:
                pos = self._real_motors[motor_name].telemetry.data.feedback.position
                if isinstance(pos, int):
                    return pos
            except Exception:
                pass
        return int(self._motor_state[motor_name]['target_cnt'])

    def _publish_joint_states(self):
        if not self._cfg:
            return
        self._last_js_publish = _time.monotonic()
        now = self.get_clock().now()
        with self._motor_lock:
            # -- turret (single motor) --
            if self._is_real('Turret'):
                q_turret = self._cfg['Turret'].counts_to_rad(
                    self._read_csp_pos_cnt('Turret'))
            else:
                q_turret = self._loopback_positions[0]

            # -- shoulder / elbow (differential pair) --
            if (self._is_real('Left_Differential')
                    or self._is_real('Right_Differential')):
                R_pos = self._cfg['Right_Differential'].counts_to_rad(
                    self._read_csp_pos_cnt('Right_Differential'))
                L_pos = self._cfg['Left_Differential'].counts_to_rad(
                    self._read_csp_pos_cnt('Left_Differential'))
                # Inverse of the mix in _on_arm_cmd:
                #   L = s - e ,  R = -(s + e)
                q_shoulder =  0.5 * (L_pos - R_pos)
                q_elbow    = -0.5 * (L_pos + R_pos)
            else:
                q_shoulder = self._loopback_positions[1]
                q_elbow    = self._loopback_positions[2]

            # -- telescope (single motor) --
            if self._is_real('Telescopic'):
                tele_cfg = self._cfg['Telescopic']
                tele_cnt = self._read_csp_pos_cnt('Telescopic')
                q_tele_m = (tele_cnt / tele_cfg.counts_per_output_rev) * self._tele_lead
            else:
                q_tele_m = self._loopback_positions[3]

            # -- wrist pan (single motor) --
            if self._is_real('Wrist'):
                q_wp = self._cfg['Wrist'].counts_to_rad(
                    self._read_csp_pos_cnt('Wrist'))
            else:
                q_wp = self._loopback_positions[4]

            # -- wrist roll (single motor) --
            if self._is_real('Gripper_360'):
                q_wr = self._cfg['Gripper_360'].counts_to_rad(
                    self._read_csp_pos_cnt('Gripper_360'))
            else:
                q_wr = self._loopback_positions[5]

        out = JointState()
        out.header.stamp = now.to_msg()
        out.name     = ARM_JOINTS + FLIPPER_JOINTS
        out.position = [q_turret, q_shoulder, q_elbow,
                        q_tele_m, q_wp, q_wr] + self._flipper_positions
        out.velocity = []; out.effort = []
        self._pub_joint_states.publish(out)

        snap = JointState(); snap.header.stamp = now.to_msg()
        with self._motor_lock:
            for name, st in self._motor_state.items():
                snap.name.append(name)
                if st['mode'] == MODE_CSP:
                    snap.position.append(float(st['target_cnt']))
                    snap.velocity.append(float(st['vel_cnt']))
                    snap.effort.append(0.0)
                elif st['mode'] == MODE_CSV:
                    snap.position.append(0.0)
                    snap.velocity.append(float(st['vel_cnt']))
                    snap.effort.append(0.0)
                elif st['mode'] == MODE_CST:
                    snap.position.append(0.0)
                    snap.velocity.append(0.0)
                    snap.effort.append(float(st['torque_cnt']))
        self._pub_motor_cmd.publish(snap)