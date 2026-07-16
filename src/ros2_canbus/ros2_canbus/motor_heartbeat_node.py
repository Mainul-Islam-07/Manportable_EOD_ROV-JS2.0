"""
motor_heartbeat_node.py
=======================
ROS 2 node that owns **both** CAN buses (Arm + Drive).

Publishing model:

- **Heartbeat** (event-driven): ``heartbeat_callback_postprocessing`` /
  ``heartbeat_timeout_callback`` — publishes active motor list on state change.
- **Diagnostics** (timer-driven, DIAG_PUBLISH_RATE_HZ): a ROS timer publishes
  ``/motor_diagnostics`` (one DiagnosticArray per bus) by snapshotting each
  motor's latest telemetry. CANopen TPDO3 interrupts keep that telemetry fresh,
  so the ROS publish rate is decoupled from CAN bus timing.

Fault policy
------------
- **Drive bus** fault (heartbeat lost ≥ 3 s or CiA-402 fault) →
  ``/drive_fault_stop`` → disarm ALL motors, trigger process shutdown.
- **Arm bus** fault (heartbeat lost ≥ 3 s or CiA-402 fault) →
  ``/arm_fault_stop`` → disarm arm motors only; drive continues.
"""

import json
import os
import time
from pathlib import Path
from threading import Lock

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from ros2_canbus.motor_config import load_motor_configs
from ros2_canbus.can_utils import (
    create_network, set_operational, scan_heartbeat,
    init_motors, attach_heartbeat, verify_and_fix_modes,
    safe_disarm_all, safe_disconnect,
)
from ros2_canbus.diagnostics import build_diag_status, is_motor_faulted


# =========================================================================
# Load config
# =========================================================================

CONFIG_FILE = str(Path(__file__).resolve().parent / "controller_config.json")

def _load_config():
    with open(CONFIG_FILE, 'r') as f:
        return json.load(f)

CFG        = _load_config()
HB_CFG     = CFG['heartbeat_node']
DRIVE_CFG  = CFG['drive_controller']

SETTINGS_FILE    = CFG['settings_file']
HEARTBEAT_WAIT_S = float(CFG['timing']['heartbeat_wait_s'])
HB_FAULT_TIMEOUT = float(CFG['timing']['heartbeat_fault_timeout_s'])
HB_POLL_RATE_HZ  = float(CFG['timing'].get('hb_poll_rate_hz', 2.0))
HB_LIVENESS_TIMEOUT_S = float(CFG['timing'].get('hb_liveness_timeout_s', 2.5))
DIAG_PUBLISH_RATE_HZ  = float(CFG['timing'].get('diag_publish_rate_hz', 10.0))
ROS_QUEUE_DEPTH  = int(CFG['ros_queue_depth'])
ROS_NODE_NAME    = HB_CFG['ros_node_name']

ARM_CAN   = HB_CFG['arm_can']
DRIVE_CAN = HB_CFG['drive_can']

TOPICS = HB_CFG['topics']
TOPIC_ARM_ACTIVE       = TOPICS['arm_active_motors']
TOPIC_DRIVE_ACTIVE     = TOPICS['drive_active_motors']
TOPIC_DIAGNOSTICS      = TOPICS['diagnostics']
TOPIC_ARM_FAULT_STOP   = TOPICS['arm_fault_stop']
TOPIC_DRIVE_FAULT_STOP = TOPICS['drive_fault_stop']

ARM_BUS_FILTER   = set(HB_CFG['arm_bus_filter'])
DRIVE_BUS_FILTER = set(HB_CFG['drive_bus_filter'])


# =========================================================================
# Arm failsafe policy (hardcoded)
# -------------------------------------------------------------------------
# Decides what the ARM bus does when an arm motor is MISSING at startup or
# LOSES its heartbeat at runtime:
#
#   True  -> FAILSAFE (current behavior): the ENTIRE arm bus goes loopback +
#            latched, every alive arm motor disarmed. Nothing on the arm moves.
#   False -> PARTIAL: run the arm motors that ARE present; loopback only the
#            missing/lost one(s). The rest of the arm stays armed/controllable.
#            NOTE: moving a partial arm (esp. beyond a dead mid-chain joint)
#            can be mechanically unsafe.
#
# Drive-bus policy and the memory-battery-LOW whole-arm latch are unaffected.
ARM_FAILSAFE_FULL_LOOPBACK = True


# =========================================================================
# Hardware bring-up (called before ROS init)
# =========================================================================

def bring_up_all(xlsx_path: str):
    """Bring up both CAN buses, scan heartbeats, init live motors.

    Returns
    -------
    nets : dict
    arm_motors : dict          (empty if any arm motor was unavailable)
    drive_motors : dict
    arm_cfgs : dict
    drive_cfgs : dict
    unavail_arm : dict         ``{name: reason}``
    unavail_drive : dict       ``{name: reason}``
    """
    arm_cfgs   = load_motor_configs(xlsx_path, ARM_BUS_FILTER)
    drive_cfgs = load_motor_configs(xlsx_path, DRIVE_BUS_FILTER)

    worm = float(DRIVE_CFG['parameters']['flipper_worm_gear_ratio'])
    for name in ('Front_Flipper', 'Rear_Flipper'):
        if name in drive_cfgs:
            drive_cfgs[name].counts_per_output_rev *= worm

    arm_active   = list(arm_cfgs.keys())
    drive_active = list(drive_cfgs.keys())

    nets = {}

    # ── Arm CAN ─────────────────────────────────────────────────────
    print("[HB-INIT] Creating Arm_CAN network...")
    arm_net = create_network(ARM_CAN['config_name'],
                             ARM_CAN['master_role'],
                             int(ARM_CAN['master_node_id']))
    nets["Arm_CAN"] = arm_net
    alive_arm, unavail_arm = scan_heartbeat(
        arm_active, arm_net, SETTINGS_FILE, HEARTBEAT_WAIT_S, tag="ARM")
    arm_motors, arm_fail = init_motors(alive_arm, arm_net, SETTINGS_FILE, tag="ARM")
    unavail_arm.update(arm_fail)
    verify_and_fix_modes(arm_motors, tag="ARM")
    attach_heartbeat(arm_motors, tag="ARM")
    set_operational(arm_net)

    # ── Drive CAN ───────────────────────────────────────────────────
    print("[HB-INIT] Creating Drive_CAN network...")
    drive_net = create_network(DRIVE_CAN['config_name'],
                               DRIVE_CAN['master_role'],
                               int(DRIVE_CAN['master_node_id']))
    nets["Drive_CAN"] = drive_net
    alive_drive, unavail_drive = scan_heartbeat(
        drive_active, drive_net, SETTINGS_FILE, HEARTBEAT_WAIT_S, tag="DRIVE")
    drive_motors, drive_fail = init_motors(alive_drive, drive_net, SETTINGS_FILE, tag="DRIVE")
    unavail_drive.update(drive_fail)
    verify_and_fix_modes(drive_motors, tag="DRIVE")
    attach_heartbeat(drive_motors, tag="DRIVE")
    set_operational(drive_net)

    print("[HB-INIT] Both buses operational.\n")
    time.sleep(0.5)

    safe_disarm_all(arm_motors, tag="ARM")
    safe_disarm_all(drive_motors, tag="DRIVE")

    if unavail_arm:
        print(f"[HB-WARN] Arm unavailable: {list(unavail_arm.keys())}")
    if unavail_drive:
        print(f"[HB-WARN] Drive unavailable: {list(unavail_drive.keys())}")
    print()

    return nets, arm_motors, drive_motors, arm_cfgs, drive_cfgs, unavail_arm, unavail_drive


# =========================================================================
# HeartbeatNode
# =========================================================================

class HeartbeatNode(Node):
    """Interrupt-driven heartbeat monitor and diagnostics publisher.

    Fault policy (asymmetric, per-motor for arm):
      • Drive motor heartbeat lost ≥ HB_FAULT_TIMEOUT or CiA-402 fault
        → publish ``/drive_fault_stop`` → ALL motors disarmed, process exits.
      • Arm motor heartbeat lost ≥ HB_FAULT_TIMEOUT or CiA-402 fault
        → publish ``/arm_fault_stop`` with motor name → that motor switches
        to loopback; remaining arm motors continue; drive unaffected.
    """

    def __init__(self, arm_motors: dict, drive_motors: dict):
        super().__init__(ROS_NODE_NAME)

        self._arm_motors   = arm_motors
        self._drive_motors = drive_motors
        self._lock         = Lock()

        self._arm_alive   = {name: True for name in arm_motors}
        self._drive_alive = {name: True for name in drive_motors}

        # Track when heartbeat was first lost (monotonic time).
        # Key present → heartbeat currently missing.  Value = time.monotonic()
        self._hb_lost_since = {}   # {motor_name: float}

        qos = QoSProfile(depth=ROS_QUEUE_DEPTH,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)

        self._pub_arm_active       = self.create_publisher(JointState, TOPIC_ARM_ACTIVE, qos)
        self._pub_drive_active     = self.create_publisher(JointState, TOPIC_DRIVE_ACTIVE, qos)
        self._pub_diags            = self.create_publisher(DiagnosticArray, TOPIC_DIAGNOSTICS, qos)
        self._pub_arm_fault_stop   = self.create_publisher(String, TOPIC_ARM_FAULT_STOP, qos)
        self._pub_drive_fault_stop = self.create_publisher(String, TOPIC_DRIVE_FAULT_STOP, qos)

        self._arm_faulted          = set()   # per-motor tracking (CiA-402)
        self._arm_bus_latched       = False  # whole-arm-bus down (heartbeat loss)
        self._drive_fault_triggered = False

        # Hook heartbeat callbacks (state-change driven)
        for name, m in arm_motors.items():
            self._hook_heartbeat(name, m, "arm")
        for name, m in drive_motors.items():
            self._hook_heartbeat(name, m, "drive")

        # Publish initial active lists
        self._publish_bus_active("arm")
        self._publish_bus_active("drive")

        # Liveness poll timer.  Liveness is judged from each motor's
        # authoritative ``last_heartbeat_time`` (a time.monotonic() value set
        # in Heartbeat_Lib.heartbeat_interrupt), NOT from the callback-driven
        # alive_map: the library watchdog flips ``is_heartbeat`` on timeout
        # but never invokes ``heartbeat_timeout_callback``, so the callback
        # path alone can leave a dead motor marked alive.
        self._hb_poll_timer = self.create_timer(
            1.0 / HB_POLL_RATE_HZ, self._poll_heartbeat_liveness)

        # Diagnostics timer.  Publishes /motor_diagnostics at a fixed rate,
        # decoupled from CAN TPDO3 timing.  TPDO3 interrupts keep each motor's
        # telemetry fresh; this timer just reads the latest stored values.
        self._diag_timer = self.create_timer(
            1.0 / DIAG_PUBLISH_RATE_HZ, self._publish_all_diagnostics)

        self.get_logger().info(
            f"motor_heartbeat ready (timer diagnostics + polled liveness):\n"
            f"   arm  active : {sorted(self._arm_motors.keys()) or '(none)'}\n"
            f"   drive active: {sorted(self._drive_motors.keys()) or '(none)'}\n"
            f"   heartbeat   : {TOPIC_ARM_ACTIVE}, {TOPIC_DRIVE_ACTIVE} "
            f"(publish on state change)\n"
            f"   liveness    : poll {HB_POLL_RATE_HZ:.1f} Hz, "
            f"timeout {HB_LIVENESS_TIMEOUT_S:.1f} s (from last_heartbeat_time)\n"
            f"   diagnostics : {TOPIC_DIAGNOSTICS} "
            f"(timer {DIAG_PUBLISH_RATE_HZ:.1f} Hz, both buses per message)\n"
            f"   HB fault timeout : {HB_FAULT_TIMEOUT:.1f} s")

    # -- accessors for robot_bringup ------------------------------------

    @property
    def arm_motors(self) -> dict:
        with self._lock:
            return dict(self._arm_motors)

    @property
    def drive_motors(self) -> dict:
        with self._lock:
            return dict(self._drive_motors)

    # -- heartbeat callback hooks (state-change) ------------------------

    def _hook_heartbeat(self, name: str, motor, bus: str):
        cb = motor.heartbeat.callback

        # Received hook: fast-path RESTORE detection between polls.  Marks
        # the motor alive immediately on the first beat back rather than
        # waiting up to one poll period.
        def on_hb_received(heartbeat_data, _n=name, _b=bus):
            self._on_heartbeat_change(_n, _b, alive=True)
        cb.heartbeat_callback_postprocessing = on_hb_received

        # Timeout hook: preserve the library's user-callback chain for
        # logging/side-effects, but do NOT set liveness here — the poll
        # (_poll_heartbeat_liveness) is the single authority for LOST, so
        # this avoids racing/double-counting _hb_lost_since.
        original_timeout = cb.heartbeat_timeout_callback
        def on_hb_timeout(_orig=original_timeout):
            _orig()
        cb.heartbeat_timeout_callback = on_hb_timeout

    def _on_heartbeat_change(self, name: str, bus: str, alive: bool):
        with self._lock:
            state_dict = self._arm_alive if bus == "arm" else self._drive_alive
            prev_alive = state_dict.get(name)
            state_dict[name] = alive

            if alive:
                # Heartbeat restored — clear the loss timer
                self._hb_lost_since.pop(name, None)
            else:
                # Heartbeat lost — record first-loss timestamp if not already set
                if name not in self._hb_lost_since:
                    self._hb_lost_since[name] = time.monotonic()

            if prev_alive == alive:
                return  # no state change for publish

        if alive:
            self.get_logger().info(f"[{bus.upper()}] {name}: heartbeat RESTORED")
        else:
            self.get_logger().warn(f"[{bus.upper()}] {name}: heartbeat LOST")
        self._publish_bus_active(bus)

    def _motor_is_alive(self, motor) -> bool:
        """Liveness via the timeout window on ``last_heartbeat_time``.

        Mirrors the standalone heartbeat_monitor_node logic: a motor is
        alive iff its most recent heartbeat arrived within
        ``HB_LIVENESS_TIMEOUT_S``.  ``last_heartbeat_time`` is a
        time.monotonic() stamp written by Heartbeat_Lib.heartbeat_interrupt;
        None means no beat has ever been seen.
        """
        hb = getattr(motor, "heartbeat", None)
        if hb is None:
            return False
        try:
            last = hb.get_status().get("last_heartbeat_time")
        except Exception:
            # Fall back to the library's own flag if get_status() fails
            return bool(getattr(hb, "is_heartbeat", False))
        if last is None:
            return False
        return (time.monotonic() - last) <= HB_LIVENESS_TIMEOUT_S

    def _poll_heartbeat_liveness(self):
        """Timer callback: re-evaluate every motor's liveness from its
        last_heartbeat_time, publish active lists on any state change, and
        keep the fault-timeout bookkeeping (``_hb_lost_since``) in sync.

        This is the authoritative liveness path.  The received/timeout
        callback hooks remain wired for fast restore detection, but this
        poll is what the dashboard and the fault logic ultimately trust.
        """
        arm_changed = drive_changed = False
        with self._lock:
            for name, m in self._arm_motors.items():
                if self._update_alive_locked(name, m, self._arm_alive):
                    arm_changed = True
            for name, m in self._drive_motors.items():
                if self._update_alive_locked(name, m, self._drive_alive):
                    drive_changed = True

        if arm_changed:
            self._publish_bus_active("arm")
        if drive_changed:
            self._publish_bus_active("drive")

        # Drive the fault-timeout escalation off the freshly-updated map.
        self._check_heartbeat_faults()

    def _update_alive_locked(self, name, motor, state_dict) -> bool:
        """Update one motor's entry in *state_dict* and _hb_lost_since.
        Caller MUST hold self._lock.  Returns True on a state change."""
        alive = self._motor_is_alive(motor)
        prev = state_dict.get(name)
        state_dict[name] = alive

        if alive:
            self._hb_lost_since.pop(name, None)
        elif name not in self._hb_lost_since:
            self._hb_lost_since[name] = time.monotonic()

        if prev == alive:
            return False
        bus = "ARM" if name in self._arm_motors else "DRIVE"
        if alive:
            self.get_logger().info(f"[{bus}] {name}: heartbeat RESTORED")
        else:
            self.get_logger().warn(f"[{bus}] {name}: heartbeat LOST")
        return True

    def _check_heartbeat_faults(self):
        """Called from the diagnostics timer path (DIAG_PUBLISH_RATE_HZ).
        If any motor has been without a heartbeat for ≥ HB_FAULT_TIMEOUT
        seconds, trigger the appropriate bus fault."""
        now = time.monotonic()
        with self._lock:
            for name, lost_time in list(self._hb_lost_since.items()):
                elapsed = now - lost_time
                if elapsed < HB_FAULT_TIMEOUT:
                    continue
                # Determine bus
                if name in self._drive_motors:
                    self._trigger_drive_fault(
                        name, f"heartbeat lost for {elapsed:.1f}s (>={HB_FAULT_TIMEOUT}s)")
                elif name in self._arm_motors:
                    reason = f"heartbeat lost for {elapsed:.1f}s (>={HB_FAULT_TIMEOUT}s)"
                    if ARM_FAILSAFE_FULL_LOOPBACK:
                        # FAILSAFE: heartbeat loss on ANY arm motor latches the
                        # whole arm bus (all joints loopback, latched to restart).
                        self._trigger_arm_bus_fault(name, reason)
                    else:
                        # PARTIAL: loopback just this motor; the rest keep running.
                        self._trigger_arm_fault(name, reason)

    # -- bus-specific fault triggers ------------------------------------

    def _trigger_arm_bus_fault(self, name: str, reason: str):
        """Whole-arm-bus heartbeat-loss latch.

        Policy: if ANY arm motor loses its heartbeat, the entire arm bus is
        declared down — every arm motor is disarmed and all arm joints go to
        loopback.  Latched until process restart: once set, it never clears,
        even if the missing heartbeat returns.  Drive bus is unaffected.

        Publishes the reserved name ``__ALL__`` on the arm-fault topic so the
        arm controller can distinguish a bus-wide latch from a single-motor
        CiA-402 fault.
        """
        if self._arm_bus_latched:
            return
        self._arm_bus_latched = True
        self.get_logger().error(
            f"ARM BUS LATCHED: {name} on Arm_CAN — {reason}. "
            f"ALL arm motors disarmed; entire arm now loopback "
            f"(latched until restart). Drive continues.")

        msg = String()
        msg.data = f"__ALL__|{reason} (triggered by {name})"
        self._pub_arm_fault_stop.publish(msg)

        # Best-effort disarm EVERY arm motor immediately.
        for mname, m in self._arm_motors.items():
            try:
                m.control.DISARM()
            except Exception:
                pass

    def _trigger_arm_fault(self, name: str, reason: str):
        """Disable one arm motor (loopback); others continue. Drive unaffected.

        This per-motor path is used for single-motor CiA-402 faults (drive
        is alive but latched an internal fault).  Heartbeat loss does NOT
        come here — it escalates via _trigger_arm_bus_fault.  If the arm bus
        is already latched bus-wide, per-motor faults are moot."""
        if self._arm_bus_latched or name in self._arm_faulted:
            return
        self._arm_faulted.add(name)
        self.get_logger().error(
            f"ARM FAULT: {name} on Arm_CAN — {reason}. "
            f"This motor will switch to loopback. Drive continues.")

        msg = String()
        msg.data = f"{name}|{reason}"
        self._pub_arm_fault_stop.publish(msg)

        # Best-effort disarm ONLY this motor
        if name in self._arm_motors:
            try:
                self._arm_motors[name].control.DISARM()
            except Exception:
                pass

    def _trigger_drive_fault(self, name: str, reason: str):
        """Disarm ALL motors and signal process shutdown."""
        if self._drive_fault_triggered:
            return
        self._drive_fault_triggered = True
        self.get_logger().error(
            f"DRIVE FAULT: {name} on Drive_CAN — {reason}. "
            f"ALL motors will be disarmed. System will terminate.")

        msg = String()
        msg.data = f"{name}|{reason}"
        self._pub_drive_fault_stop.publish(msg)

        # Best-effort disarm EVERYTHING immediately
        for mname, m in self._arm_motors.items():
            try:
                m.control.DISARM()
            except Exception:
                pass
        for mname, m in self._drive_motors.items():
            try:
                m.control.DISARM()
            except Exception:
                pass

    def _publish_bus_active(self, bus: str):
        with self._lock:
            if bus == "arm":
                alive_names = [n for n, a in self._arm_alive.items() if a]
                pub = self._pub_arm_active
            else:
                alive_names = [n for n, a in self._drive_alive.items() if a]
                pub = self._pub_drive_active
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = alive_names
        pub.publish(msg)

    # -- diagnostics publishing (timer-driven) --------------------------

    def _publish_all_diagnostics(self):
        """Timer-driven: snapshot the latest stored telemetry for BOTH buses
        into a single DiagnosticArray and publish it at DIAG_PUBLISH_RATE_HZ.

        Telemetry itself is kept fresh by the library's TPDO3 interrupts;
        this timer only snapshots and publishes the latest values, so the
        ROS publish rate is decoupled from CAN bus timing."""
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Isolate each bus: a hard failure while building one bus's status
        # must not suppress the other bus or the publish.  This preserves the
        # "arm down → drive continues" policy (per-motor build is already
        # wrapped; this guards the fault-trigger side-effects too).
        with self._lock:
            for bus in ("arm", "drive"):
                try:
                    self._append_bus_status(msg, bus)
                except Exception as e:
                    self.get_logger().warn(
                        f"diagnostics: {bus} status build failed: {e}")

        self._pub_diags.publish(msg)

        # ── heartbeat-loss timeout check ──
        self._check_heartbeat_faults()

    def _append_bus_status(self, msg: DiagnosticArray, bus: str):
        """Append one DiagnosticStatus per motor on *bus* to *msg* and run
        per-motor CiA-402 fault checks.  Caller must hold ``self._lock``."""
        motors = self._arm_motors if bus == "arm" else self._drive_motors
        label  = "Arm_CAN" if bus == "arm" else "Drive_CAN"
        alive_map = self._arm_alive if bus == "arm" else self._drive_alive
        for name, m in motors.items():
            try:
                status = build_diag_status(name, m, bus_label=label)
            except Exception as e:
                status = DiagnosticStatus()
                status.level = DiagnosticStatus.STALE
                status.name = name
                status.message = f"diag read failed: {e}"
                status.hardware_id = "?"

            # ── per-motor heartbeat liveness override (display only) ──
            # The liveness poll (_poll_heartbeat_liveness) is now the
            # single owner of alive_map and _hb_lost_since.  Here we only
            # reflect that state into the diagnostic message so a motor
            # whose heartbeat is currently lost is shown STALE on the
            # dashboard and never greys out from message staleness alone.
            # We do NOT mutate alive_map / _hb_lost_since here, avoiding a
            # race with the poll.
            if alive_map.get(name) is False:
                status.level = DiagnosticStatus.STALE
                status.message = "heartbeat lost"

            msg.status.append(status)

            # ── CiA-402 fault check (immediate, bus-specific) ──
            faulted, reason = is_motor_faulted(m)
            if faulted:
                if bus == "drive":
                    self._trigger_drive_fault(name, reason)
                else:
                    self._trigger_arm_fault(name, reason)