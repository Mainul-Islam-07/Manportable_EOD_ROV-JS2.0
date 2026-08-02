#!/usr/bin/env python3
"""
drive_controller.py
===================
ROS 2 command bridge for the **Drive_CAN** bus motors (flippers + drives).

This node does NOT own the CAN bus.  Motor objects are passed in by
``robot_bringup.py`` (which runs the heartbeat node in the same process).
Diagnostics are handled exclusively by ``motor_heartbeat_node.py``.
"""

import json
import math
from pathlib import Path
from threading import Lock

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from ros2_canbus.motor_config import MODE_CSP, MODE_CSV, MODE_CST
from ros2_canbus.diagnostics import decode_cia402_state


# =========================================================================
# Load config
# =========================================================================

CONFIG_FILE = str(Path(__file__).resolve().parent / "controller_config.json")

def _load_config():
    with open(CONFIG_FILE, 'r') as f:
        return json.load(f)

CFG = _load_config()
MY  = CFG['drive_controller']

# -- shared --
SM                       = CFG['state_machine']
# Drive motors disarm shortly after the command reaches 0 (they still stop
# instantly via IDLE_STOP). Drive-specific override; the arm keeps the shared
# (long) idle_seconds so the manipulator holds position when idle.
IDLE_SECONDS             = float(MY.get('idle_seconds', SM['idle_seconds']))
RE_ARM_WAIT_S            = float(SM['re_arm_wait_s'])
EPS_CNT                  = int(SM['eps_cnt'])
EPS_VEL_CAN              = int(SM['eps_vel_can'])
EPS_TORQUE_CAN           = int(SM['eps_torque_can'])

RATES                    = CFG['rates_hz']
MOTOR_CMD_RATE_HZ        = float(RATES['motor_cmd'])

ROS_QUEUE_DEPTH          = int(CFG['ros_queue_depth'])

# -- drive-specific --
ROS_NODE_NAME            = MY['ros_node_name']

TOPICS                   = MY['topics']
TOPIC_FLIPPER_CMD        = TOPICS['flipper_cmd']
TOPIC_SBUS_JS            = TOPICS['sbus_js']
TOPIC_MOTOR_COMMANDS     = TOPICS['motor_commands']
TOPIC_JOINT_STATES       = TOPICS['joint_states']

FLIPPER_JOINTS           = MY['joints']['flipper_joints']

SJS_IDX_LEFT_DRIVE       = int(MY['sbus_indices']['left_drive_velocity'])
SJS_IDX_RIGHT_DRIVE      = int(MY['sbus_indices']['right_drive_velocity'])

PARAMS                   = MY['parameters']

TOPIC_DRIVE_FAULT_STOP   = CFG['heartbeat_node']['topics']['drive_fault_stop']

TOPIC_DRIVE_BATTERY_OK   = CFG['battery_monitor']['topics']['drive_battery_ok']

FLIPPER_WORM_RATIO       = float(PARAMS['flipper_worm_gear_ratio'])
FLIPPER_MOTORS           = {'Front_Flipper', 'Rear_Flipper'}
DRIVE_DIRECTION          = PARAMS['drive_direction']
FLIPPER_DIRECTION        = PARAMS['flipper_direction']


# =========================================================================
# Drive controller node
# =========================================================================

class DriveController(Node):

    def __init__(self, real_motors: dict, cfgs: dict):
        super().__init__(ROS_NODE_NAME)

        self._real_motors = real_motors
        self._cfg         = cfgs

        self.declare_parameter('drive_velocity_scale',
                               float(PARAMS['drive_velocity_scale']))
        self.declare_parameter('csp_profile_vel_frac',
                               float(PARAMS['csp_profile_vel_frac']))
        self.declare_parameter('flipper_csp_profile_vel_frac',
                               float(PARAMS['flipper_csp_profile_vel_frac']))

        self._drive_scale      = float(self.get_parameter('drive_velocity_scale').value)
        self._vel_frac         = float(self.get_parameter('csp_profile_vel_frac').value)
        self._flipper_vel_frac = float(self.get_parameter('flipper_csp_profile_vel_frac').value)

        self._motor_lock = Lock()
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

        self._arm_state = {}
        for name in self._real_motors:
            self._arm_state[name] = {
                'state': 'DISARMED', 'arming_complete_time': None,
                'last_run_value': None, 'last_active_time': None,
                'idle_since': None, 'arm_attempts': 0,
            }

        self._seed_csp_from_feedback()

        qos = QoSProfile(depth=ROS_QUEUE_DEPTH,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(JointState, TOPIC_FLIPPER_CMD, self._on_flipper_cmd, qos)
        self.create_subscription(JointState, TOPIC_SBUS_JS,     self._on_sbus_js,     qos)
        self.create_subscription(String,     TOPIC_DRIVE_FAULT_STOP, self._on_fault_stop, qos)

        # Encoder memory-battery status (latched).  A low drive battery means
        # the drive encoders' memory is unreliable → disarm the drive bus and
        # lock it out, latched until process restart.  Unlike a drive *fault*
        # this does NOT bring the whole process down — only the drive bus stops.
        batt_qos = QoSProfile(depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, TOPIC_DRIVE_BATTERY_OK,
                                 self._on_drive_battery, batt_qos)

        self._fault_stop = False
        self._battery_lockout = False

        self._pub_motor_cmd    = self.create_publisher(JointState, TOPIC_MOTOR_COMMANDS, qos)
        self._pub_joint_states = self.create_publisher(JointState, TOPIC_JOINT_STATES,   qos)

        # TPDO1 interrupt (10 ms) drives joint state publishing — no timer
        self._hook_tpdo1_trigger()

        self.create_timer(1.0 / MOTOR_CMD_RATE_HZ, self._command_tick)

        self.get_logger().info(
            f"drive_controller ready (Drive_CAN):\n"
            f"   active motors    : {sorted(self._real_motors.keys()) or '(none)'}\n"
            f"   loopback motors  : {sorted(set(self._cfg) - set(self._real_motors))}\n"
            f"   joint feedback   : {TOPIC_JOINT_STATES} (TPDO1 interrupt, 10 ms)\n"
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
        frac = self._flipper_vel_frac if motor_name in FLIPPER_MOTORS else self._vel_frac
        return int(round(frac * cfg.max_velocity))

    def _hook_tpdo1_trigger(self):
        """Hook TPDO1 postprocessing on the first real motor to trigger
        ``_publish_joint_states`` on each 10 ms CAN interrupt."""
        if not self._real_motors:
            return
        trigger_name = next(iter(self._real_motors))
        cb = self._real_motors[trigger_name].feedback.callback

        def on_tpdo1(position, velocity):
            self._publish_joint_states()

        cb.TPDO_1_Position_actual_value_Velocity_actual_value_Object_callback_postprocessing = on_tpdo1

    # -- setters ----------------------------------------------------------

    def _set_csp_target_rad(self, motor_name, q_rad):
        cfg = self._cfg[motor_name]
        cnt = cfg.clamp_position_cnt(cfg.rad_to_counts(q_rad))
        self._motor_state[motor_name] = {
            'mode': MODE_CSP, 'target_cnt': cnt,
            'vel_cnt': self._csp_profile_vel_cnt(motor_name, cfg),
        }

    def _set_csv_target_rad_s(self, motor_name, w_rad_s):
        cfg = self._cfg[motor_name]
        v_can = cfg.clamp_velocity_cnt(cfg.rad_per_s_to_can_vel(w_rad_s))
        self._motor_state[motor_name] = {'mode': MODE_CSV, 'vel_cnt': v_can}

    # -- subscription callbacks -------------------------------------------

    def _on_flipper_cmd(self, msg):
        pos = dict(zip(msg.name, msg.position))
        try:
            q_front = pos['front_flipper_Joint']
            q_rear  = pos['rear_flipper_Joint']
        except KeyError as e:
            self.get_logger().warn(f"{TOPIC_FLIPPER_CMD} missing {e}")
            return
        with self._motor_lock:
            self._set_csp_target_rad('Front_Flipper',
                                     q_front * FLIPPER_DIRECTION['Front_Flipper'])
            self._set_csp_target_rad('Rear_Flipper',
                                     q_rear * FLIPPER_DIRECTION['Rear_Flipper'])

    def _on_sbus_js(self, msg):
        if len(msg.velocity) <= max(SJS_IDX_LEFT_DRIVE, SJS_IDX_RIGHT_DRIVE):
            self.get_logger().warn(
                f"{TOPIC_SBUS_JS} malformed (|vel|={len(msg.velocity)})")
            return
        vL_pct = float(msg.velocity[SJS_IDX_LEFT_DRIVE])
        vR_pct = float(msg.velocity[SJS_IDX_RIGHT_DRIVE])
        with self._motor_lock:
            self._set_csv_target_rad_s(
                'Left_Drive',
                vL_pct * self._drive_scale * DRIVE_DIRECTION['Left_Drive'])
            self._set_csv_target_rad_s(
                'Right_Drive',
                vR_pct * self._drive_scale * DRIVE_DIRECTION['Right_Drive'])

    def _on_fault_stop(self, msg):
        """Received from heartbeat node — a motor faulted. Disarm everything."""
        if self._fault_stop:
            return
        self._fault_stop = True
        self.get_logger().error(
            f"FAULT STOP received ({msg.data}). Disarming all drive motors.")
        for name, m in self._real_motors.items():
            try:
                m.control.DISARM()
            except Exception as e:
                self.get_logger().warn(f"{name}: fault disarm error ({e})")
        with self._motor_lock:
            for name in self._arm_state:
                self._arm_state[name]['state'] = 'FAULTED'
                self._arm_state[name]['idle_since'] = None
                self._arm_state[name]['arming_complete_time'] = None

    def _on_drive_battery(self, msg):
        """Drive encoder memory-battery status.  ``False`` = below threshold:
        disarm every drive motor and lock the bus out, latched until process
        restart.  A later recovery does NOT release the lockout.

        Note: unlike ``_on_fault_stop`` this does NOT publish /drive_fault_stop,
        so the process keeps running (arm bus unaffected) — only drive stops."""
        if msg.data:
            return   # battery OK
        if self._battery_lockout:
            return
        self._battery_lockout = True
        self.get_logger().error(
            "DRIVE MEMORY BATTERY LOW. Disarming all drive motors "
            "(latched until process restart).")
        for name, m in self._real_motors.items():
            try:
                m.control.DISARM()
            except Exception as e:
                self.get_logger().warn(f"{name}: battery disarm error ({e})")
        with self._motor_lock:
            for name in self._arm_state:
                self._arm_state[name]['state'] = 'FAULTED'
                self._arm_state[name]['idle_since'] = None
                self._arm_state[name]['arming_complete_time'] = None

    # -- command tick (ARM state machine + RUN dispatch) ------------------

    def _command_tick(self):
        if not self._real_motors or self._fault_stop or self._battery_lockout:
            return
        now = self.get_clock().now()

        # Phase 1: decide what to do under the lock (fast, no CAN I/O)
        actions = []
        with self._motor_lock:
            for name, arm in self._arm_state.items():
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
                        # Only declare ARMED once the drive has actually reached
                        # CiA-402 Operation Enabled (brake released). Previously
                        # this was purely time-based, so a drive that dropped a
                        # controlword PDO (left in SWITCH_ON_DISABLED) or latched
                        # a fault was marked ARMED anyway and then had RUN streamed
                        # into it while still braked — the "one wheel won't run"
                        # symptom. If not enabled yet, re-issue ARM (which now
                        # also clears a fault) instead of pretending it worked.
                        cia = decode_cia402_state(self._statusword_raw(name))
                        if cia is None or cia == 'OPERATION_ENABLED':
                            # cia is None => no statusword telemetry yet; fall back
                            # to the old time-based behavior rather than block.
                            arm['state'] = 'ARMED'
                            arm['arming_complete_time'] = None
                            arm['arm_attempts'] = 0
                            self.get_logger().info(f"{name}: ARMING -> ARMED")
                        else:
                            arm['arm_attempts'] += 1
                            actions.append(('ARM', name, cfg.mode, None))
                            if arm['arm_attempts'] in (1, 5, 20):
                                self.get_logger().warn(
                                    f"{name}: not Operation Enabled after ARM "
                                    f"(state={cia}); re-arming "
                                    f"(attempt {arm['arm_attempts']})")
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
                            if cfg.mode == MODE_CSV:
                                actions.append(('IDLE_STOP', name, cfg.mode, None))
                            elif cfg.mode == MODE_CST:
                                actions.append(('IDLE_STOP', name, cfg.mode, None))
                            arm['idle_since'] = now
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

                elif action == 'IDLE_STOP':
                    if mode == MODE_CSV:
                        self._real_motors[name].mode.velocity.RUN(0)
                    elif mode == MODE_CST:
                        self._real_motors[name].mode.torque.RUN(0)

                elif action == 'DISARM':
                    self._real_motors[name].control.DISARM()
                    with self._motor_lock:
                        self._arm_state[name]['state'] = 'DISARMED'
                        self._arm_state[name]['idle_since'] = None
                        self._arm_state[name]['arming_complete_time'] = None
                        self._arm_state[name]['arm_attempts'] = 0
                    self.get_logger().info(f"{name}: idle -> DISARM")

            except Exception as e:
                self.get_logger().warn(f"{name}: {action} failed ({e})")

    def _statusword_raw(self, name):
        """Latest CiA-402 statusword (0x6041) for a drive motor, or None if
        telemetry isn't available yet. Kept fresh by the library's TPDO3
        interrupts — the same source the heartbeat node reads. Returning None
        makes the arming check fall back to time-based (never worse than before)."""
        try:
            return self._real_motors[name].telemetry.data.metadata.statusword.raw
        except Exception:
            return None

    # -- publishers -------------------------------------------------------

    def _read_csp_pos_cnt(self, motor_name):
        if motor_name in self._real_motors:
            try:
                pos = self._real_motors[motor_name].telemetry.data.feedback.position
                if isinstance(pos, int):
                    return pos
            except Exception:
                pass
        return int(self._motor_state[motor_name]['target_cnt'])

    def _publish_joint_states(self):
        now = self.get_clock().now()
        with self._motor_lock:
            front_cnt = self._read_csp_pos_cnt('Front_Flipper')
            rear_cnt  = self._read_csp_pos_cnt('Rear_Flipper')
            q_front   = (self._cfg['Front_Flipper'].counts_to_rad(front_cnt)
                         * FLIPPER_DIRECTION['Front_Flipper'])
            q_rear    = (self._cfg['Rear_Flipper'].counts_to_rad(rear_cnt)
                         * FLIPPER_DIRECTION['Rear_Flipper'])

        out = JointState()
        out.header.stamp = now.to_msg()
        out.name     = FLIPPER_JOINTS
        out.position = [q_front, q_rear]
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