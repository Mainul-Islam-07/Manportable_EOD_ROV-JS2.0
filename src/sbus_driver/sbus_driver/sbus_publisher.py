#!/usr/bin/env python3
"""
sbus_publisher.py  —  sbus_driver package

ROS 2 node that reads SBUS frames from a serial port, validates them for
data integrity, interprets every channel through the stateful
ChannelInterpreter, and publishes a single composite SbusControl message
on /sbus/control.

Channel map
-----------
  CH1  – control_mode      (3-state: up=HOME  ctr=ARM  dn=DRIVE)
  CH2  – operation_mode    (3-state: up=STAIR   ctr=ARMED  dn=DISARMED)
  CH3  – arm_y_cmd         (3-state: -1/0/+1)
  CH4  – arm_x_cmd         (3-state: -1/0/+1)
  CH5  – arm_z_cmd         (3-state: -1/0/+1)
  CH6  – ee_roll           (3-state: -1/0/+1, for external process)
  CH7  – (unassigned)
  CH8  – drive_speed       (scaled: 0–100%)
  CH9  – rear_flipper      (3-state: -1/0/+1)
  CH10 – front_flipper     (3-state: -1/0/+1)
  CH11 – telescope_cmd     (3-state: -1/0/+1)
  CH12 – ee_pitch          (3-state: -1/0/+1 delta, coordinator accumulates)
  CH13 – light_state       (LEVEL: switch up=on(1)  down=off(0))
  CH14 – camera_axis       (LEVEL: switch up=tilt(1)  down=pan(0))
  CH15 – claw_cmd          (scroll: -100.0 to +100.0)
  CH16 – camera_cmd        (3-state: -1/0/+1)

Data integrity features
-----------------------
  * Frame START byte (0x0F) AND END byte (0x00 family) both validated.
    Misaligned frames are rejected and the reader resyncs byte-by-byte.
  * Channel sanity check: real SBUS stick values sit in [172, 1811].
    Frames with 2+ channels outside this range are rejected as garbage
    (catches SIYI MK32 air-unit fill patterns when ground unit is off).
  * Failsafe / frame_lost flags trigger an explicit DISARMED safe-state
    publish — NOT a serial-port reopen. The line is fine when the radio
    link drops; only the radio is gone.
  * Frame timeout: if no good frame arrives within 100 ms, the publisher
    emits a safe-state DISARMED message. Catches RC systems (like SIYI)
    that stop setting the failsafe flag after a few seconds of link loss.
  * Safe-state publishing is rate-limited to 20 Hz to avoid swamping
    subscribers when the timer ticks at 200 Hz.

Message field conventions (see sbus_interfaces/msg/SbusControl.msg):
  control_mode   : uint8   ARM=0  HOME=1  DRIVE=2
  operation_mode : uint8   ARMED=0  STAIR=1  DISARMED=2
  arm_{x,y,z}_cmd: int8    -1 / 0 / +1  (integrated by coordinator)
  ee_pitch       : int8    -1/0/+1 delta (accumulated by coordinator)
  ee_roll        : int8    -1/0/+1 (raw, for external process)
  claw_cmd       : float64 -100.0 to +100.0 (scroll wheel)
  telescope_cmd  : int8    RETRACT=-1  HOLD=0  EXTEND=+1
  flippers       : int8    always active, -1/0/+1
  light_state    : uint8   0=off  1=on (follows CH13 switch)
  camera_axis    : uint8   0=pan  1=tilt (follows CH14 switch)
  camera_cmd     : int8    -1/0/+1 (command for selected axis)

light_state / camera_axis are exempt from all masking
-----------------------------------------------------
Every other command field is forced to 0 when DISARMED and in the link-lost
safe state.  light_state and camera_axis are NOT: they follow their CH13/CH14
switches in every mode, and the safe-state message carries the last-known
values.  Neither field commands motion — camera_cmd is the actual camera
motion command and is still zeroed — so masking them bought no safety, while
losing the work lights on a disarm or a radio drop makes the robot harder to
see and recover.
"""

import time

import rclpy
from rclpy.node import Node
from sbus_interfaces.msg import SbusControl

import serial

# ── Defaults (overridable via ROS parameters) ─────────────────────────────────
_DEFAULT_PORT          = '/dev/ttyAMA0'
_DEFAULT_BAUD          = 100_000
_DEFAULT_TIMER_PERIOD  = 0.005          # seconds (200 Hz poll)
_DEFAULT_FRAME_TIMEOUT = 0.100          # seconds (100 ms)
_DEFAULT_OPEN_TIMEOUT  = 30.0           # seconds to keep retrying the first open
_OPEN_RETRY_PERIOD     = 1.0            # seconds between initial-open attempts

# ── Message constants (mirrored for readability) ──────────────────────────────
_MODE_ARM   = SbusControl.CONTROL_MODE_ARM    # 0
_MODE_HOME  = SbusControl.CONTROL_MODE_HOME   # 1
_MODE_DRIVE = SbusControl.CONTROL_MODE_DRIVE  # 2

_OP_ARMED    = SbusControl.OPERATION_MODE_ARMED      # 0
_OP_STAIR    = SbusControl.OPERATION_MODE_STAIR      # 1
_OP_DISARMED = SbusControl.OPERATION_MODE_DISARMED   # 2

# ── Frame integrity constants ─────────────────────────────────────────────────
# Valid SBUS frame end-byte values. Standard receivers use 0x00; some use the
# low nibble for digital channels, leaving the high nibble as a counter.
_SBUS_END_BYTES = frozenset({0x00, 0x04, 0x14, 0x24, 0x34})

# Real SBUS stick values from a working transmitter sit in [172, 1811] (the
# official 100% travel range). Even at extended end-points (>100%) real radios
# stay within roughly [100, 1900]. Values below 172 on a switch/dial channel
# are physically impossible — those channels are discrete.
_SBUS_VALID_LO = 172
_SBUS_VALID_HI = 1811

# Safe-state publish rate limit — don't flood subscribers at 200 Hz.
_SAFE_STATE_MIN_PERIOD_NS = 50_000_000   # 50 ms → 20 Hz


# =============================================================================
#  SBUS frame decoder
# =============================================================================

def _channels_look_sane(ch: list) -> bool:
    """
    Reject frames that look like failsafe garbage rather than real stick data.

    SIYI MK32 (and other digital RC air units) often DON'T set the SBUS
    failsafe flag when the ground unit powers off — they keep streaming
    frames with internal fill patterns where many channels sit far below
    the legitimate [172, 1811] range. This check is the only line of
    defense for that case.

    Allows at most 1 channel briefly outside spec (extreme end-point on
    a single stick); 2+ simultaneous out-of-range channels is garbage.
    """
    out_of_range = sum(1 for v in ch
                       if v < _SBUS_VALID_LO or v > _SBUS_VALID_HI)
    return out_of_range <= 1


def decode_sbus_frame(frame: bytes) -> dict | None:
    """
    Decode a 25-byte SBUS frame. Returns None on bad/garbage frames.
    Returns dict with 'channels' (16 raw values 0-2047), 'frame_lost', 'failsafe'.
    """
    if len(frame) != 25 or frame[0] != 0x0F:
        return None
    if frame[24] not in _SBUS_END_BYTES:
        return None
    d = frame[1:23]
    ch = [0] * 16
    ch[0]  = ( d[0]        | d[1]  << 8                ) & 0x07FF
    ch[1]  = ( d[1]  >> 3  | d[2]  << 5                ) & 0x07FF
    ch[2]  = ( d[2]  >> 6  | d[3]  << 2  | d[4]  << 10 ) & 0x07FF
    ch[3]  = ( d[4]  >> 1  | d[5]  << 7                ) & 0x07FF
    ch[4]  = ( d[5]  >> 4  | d[6]  << 4                ) & 0x07FF
    ch[5]  = ( d[6]  >> 7  | d[7]  << 1  | d[8]  << 9  ) & 0x07FF
    ch[6]  = ( d[8]  >> 2  | d[9]  << 6                ) & 0x07FF
    ch[7]  = ( d[9]  >> 5  | d[10] << 3                ) & 0x07FF
    ch[8]  = ( d[11]       | d[12] << 8                ) & 0x07FF
    ch[9]  = ( d[12] >> 3  | d[13] << 5                ) & 0x07FF
    ch[10] = ( d[13] >> 6  | d[14] << 2  | d[15] << 10 ) & 0x07FF
    ch[11] = ( d[15] >> 1  | d[16] << 7                ) & 0x07FF
    ch[12] = ( d[16] >> 4  | d[17] << 4                ) & 0x07FF
    ch[13] = ( d[17] >> 7  | d[18] << 1  | d[19] << 9  ) & 0x07FF
    ch[14] = ( d[19] >> 2  | d[20] << 6                ) & 0x07FF
    ch[15] = ( d[20] >> 5  | d[21] << 3                ) & 0x07FF
    if not _channels_look_sane(ch):
        return None
    flags = frame[23]
    return {
        'channels':   ch,
        'frame_lost': 1 if (flags & 0x04) else 0,
        'failsafe':   1 if (flags & 0x08) else 0,
    }


# =============================================================================
#  Raw channel classification helpers
# =============================================================================

def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))

def _scale_linear(value: float,
                  in_min: float, in_max: float,
                  out_min: float, out_max: float) -> float:
    value = _clamp(value, in_min, in_max)
    return out_min + (value - in_min) / (in_max - in_min) * (out_max - out_min)

# 3-state → int8 (-1 / 0 / +1) ------------------------------------------------

def _ud3(v: int) -> int:
    """up → +1   center → 0   down → -1"""
    if v > 1400: return -1
    if v > 400:  return  0
    return 1

def _lcr3(v: int) -> int:
    """left → -1   center → 0   right → +1"""
    if v > 1400: return -1
    if v > 400:  return  0
    return 1

def _rcl3(v: int) -> int:
    """right → +1   center → 0   left → -1  (inverted CH6 mapping)"""
    if v > 1400: return  1
    if v > 400:  return  0
    return -1

def _fcb3(v: int) -> int:
    """front → -1   center → 0   back → +1"""
    if v > 1400: return  1
    if v > 400:  return  0
    return -1

# 2-state → uint8 (0 / 1) -----------------------------------------------------

def _ud2(v: int) -> int:
    """down → 0   up → 1"""
    return 0 if v > 1400 else 1


def _classify_raw(channels: list) -> dict:
    """
    Map the 16 raw SBUS channel values (0-2047) to typed intermediates.

    Channel assignments:
      CH1  – control_mode         (3-state ud: up=HOME  ctr=ARM  dn=DRIVE)
      CH2  – operation_mode       (3-state ud: up=STAIR   ctr=ARMED  dn=DISARMED)
      CH3  – arm_y / fwd          (3-state ud)
      CH4  – arm_x / turn         (3-state ud)
      CH5  – arm_z                (3-state ud)
      CH6  – ee_roll              (3-state rcl, inverted)
      CH7  – (unassigned)         (scaled float, ignored)
      CH8  – drive_speed          (scaled int, 0-100 %)
      CH9  – rear_flipper         (3-state fcb)
      CH10 – front_flipper        (3-state fcb)
      CH11 – telescope_cmd        (3-state fcb)
      CH12 – ee_pitch             (3-state ud)
      CH13 – light_state          (2-state — toggled by interpreter)
      CH14 – camera_axis          (2-state — toggled by interpreter)
      CH15 – claw_cmd             (scaled float, -100 to +100)
      CH16 – camera_cmd           (3-state ud)
    """
    return {
        1:  _ud3(channels[0]),       # control_mode
        2:  _ud3(channels[1]),       # operation_mode
        3:  _ud3(channels[2]),       # arm_y / fwd
        4:  _ud3(channels[3]),       # arm_x / turn
        5:  _ud3(channels[4]),       # arm_z
        6:  _rcl3(channels[5]),      # ee_roll
        7:  float(_scale_linear(channels[6], 272, 1712, -100.0, 100.0)),  # unassigned
        8:  int(round(_scale_linear(channels[7], 272, 1712, 0.0, 100.0))),  # drive_speed
        9:  _fcb3(channels[8]),      # rear_flipper
        10: _fcb3(channels[9]),      # front_flipper
        11: _fcb3(channels[10]),     # telescope_cmd
        12: _ud3(channels[11]),      # ee_pitch
        13: _ud2(channels[12]),      # light_state (raw 2-state, edge-detected below)
        14: _ud2(channels[13]),      # camera_axis (raw 2-state, edge-detected below)
        15: float(_scale_linear(channels[14], 272, 1712, -100.0, 100.0)),  # claw_cmd
        16: _ud3(channels[15]),      # camera_cmd
    }


# =============================================================================
#  Stateful channel interpreter
# =============================================================================

class ChannelInterpreter:
    """
    Produces a fully-typed output dict mapping 1:1 onto SbusControl.msg
    fields.  Stateless for most fields — the coordinator owns arm/pitch
    integration.

    Persistent state held here:
      * claw_cmd      – held value of the scroll wheel
      * light_state   – ON/OFF, follows the CH13 maintained switch position
      * camera_axis   – PAN/TILT, follows the CH14 maintained switch position
    """

    def __init__(self):
        self.claw_cmd: float = 0.0
        # Follow the maintained CH13/CH14 switches.  These are retained across
        # ticks because SbusPublisher._publish_safe_state reads them: when the
        # radio link drops there is no frame to derive them from, and the safe
        # state must carry the last-known light/axis rather than zeroing them.
        self.light_state: int = 0      # 0=off, 1=on
        self.camera_axis: int = 0      # 0=pan, 1=tilt

    # ── Main interpret ────────────────────────────────────────────────────────

    def interpret(self, channels: list) -> dict:
        raw = _classify_raw(channels)

        # ── CH1: control_mode (3-state: up=HOME, center=ARM, down=DRIVE) ──
        if   raw[1] ==  1: mode = _MODE_HOME
        elif raw[1] == -1: mode = _MODE_DRIVE
        else:              mode = _MODE_ARM

        # ── CH2: operation_mode (3-state: up=STAIR, center=ARMED, down=DISARMED)
        if   raw[2] ==  1: op_mode = _OP_STAIR
        elif raw[2] == -1: op_mode = _OP_DISARMED
        else:              op_mode = _OP_ARMED

        drive_speed: int = raw[8]

        # ── Drive mixer (CH3, CH4) ───────────────────────────────────────────
        # CH3 = y-axis stick (fwd/back), CH4 = x-axis stick (turn).
        # Matches the arm mapping below: same stick channel drives the
        # same physical axis whether in DRIVE or ARM mode.
        if mode == _MODE_DRIVE:
            fwd   = -raw[3] * drive_speed       # CH3: down=fwd, up=rev (polarity reversed)
            turn  = raw[4] * (drive_speed / 2)  # CH4: left=left, right=right

            if fwd < 0:
                turn = - turn

            drive_left  = float(fwd + turn)
            drive_right = float(fwd - turn)
        else:
            drive_left = drive_right = 0.0

        # ── Arm deltas (CH3=y, CH4=x, CH5=z) — ARM mode only ─────────────
        if mode == _MODE_ARM:
            arm_x_cmd  = -raw[4]     # CH4
            arm_y_cmd  = -raw[3]     # CH3
            arm_z_cmd  = -raw[5]     # CH5

            # CH15: claw_cmd (scroll -100 to +100)
            self.claw_cmd = raw[15]
        else:
            arm_x_cmd = arm_y_cmd = arm_z_cmd = 0
            self.claw_cmd = 0.0

        # ── CH13: light_state — LEVEL FOLLOW the maintained switch ─────────
        # Switch position IS the state: up (raw 1) = on, down (raw 0) = off.
        # (Was rising-edge toggle, which suited a momentary button; with a
        # maintained switch that needed two flicks per change because the
        # down-flick produced no edge.)
        self.light_state = raw[13]
        light_state = self.light_state

        # ── CH14: camera_axis — LEVEL FOLLOW the maintained switch ─────────
        # up (raw 1) = tilt, down (raw 0) = pan.
        self.camera_axis = raw[14]
        camera_axis = self.camera_axis

        # ── Always-active fields ─────────────────────────────────────────────
        ee_pitch      = raw[12]      # CH12: always pass through
                                     # (the /fire_mode firing mode needs it)
        ee_roll       = raw[6]       # CH6: raw -1/0/+1
        telescope_cmd = raw[11]      # CH11
        rear_flipper  = raw[9]       # CH9
        front_flipper = raw[10]      # CH10
        camera_cmd    = raw[16]      # CH16

        # ── Build output dict ──────────────────────────────────────────────
        if mode == _MODE_HOME:
            result = dict(
                control_mode    = _MODE_HOME,
                operation_mode  = op_mode,
                drive_left      = 0.0,
                drive_right     = 0.0,
                drive_speed     = 0,
                arm_x_cmd       = 0,
                arm_y_cmd       = 0,
                arm_z_cmd       = 0,
                ee_roll         = 0,
                ee_pitch        = ee_pitch,
                claw_cmd        = 0.0,
                telescope_cmd   = 0,
                rear_flipper    = rear_flipper,
                front_flipper   = front_flipper,
                light_state     = light_state,
                camera_axis     = camera_axis,
                camera_cmd      = camera_cmd,
            )
        else:
            result = dict(
                control_mode    = mode,
                operation_mode  = op_mode,
                drive_left      = drive_left,
                drive_right     = drive_right,
                drive_speed     = drive_speed,
                arm_x_cmd       = arm_x_cmd,
                arm_y_cmd       = arm_y_cmd,
                arm_z_cmd       = arm_z_cmd,
                ee_roll         = ee_roll,
                ee_pitch        = ee_pitch,
                claw_cmd        = self.claw_cmd,
                telescope_cmd   = telescope_cmd,
                rear_flipper    = rear_flipper,
                front_flipper   = front_flipper,
                light_state     = light_state,
                camera_axis     = camera_axis,
                camera_cmd      = camera_cmd,
            )

        # ── DISARMED: zero every command except the exemptions below ───────
        # Exempt: control_mode / operation_mode carry the operator's intent, and
        # light_state / camera_axis are UI state rather than actuation — they
        # keep following their CH13/CH14 switches while disarmed so a disarm
        # doesn't kill the work lights or reset the camera selector.  camera_cmd
        # (the actual camera motion command) IS still zeroed.
        # Note: this zeros the OUTPUT only. self.claw_cmd internal held state is
        # preserved across DISARMED periods.
        if op_mode == _OP_DISARMED:
            for key in result:
                if key in ('operation_mode', 'control_mode',
                           'light_state', 'camera_axis'):
                    continue
                elif isinstance(result[key], float):
                    result[key] = 0.0
                else:
                    result[key] = 0

        return result


# =============================================================================
#  Serial helpers
# =============================================================================

def _open_serial(port: str, baud: int) -> serial.Serial:
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_EVEN,
        stopbits=serial.STOPBITS_TWO,
        timeout=0.05,
    )
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    # Wait one full SBUS frame period (~14 ms) before first read so we
    # start in the inter-frame gap, not mid-frame, then drain again.
    time.sleep(0.020)
    ser.reset_input_buffer()
    return ser


def _read_one_frame(ser: serial.Serial) -> bytes | None:
    """
    Read one *validated* 25-byte SBUS frame.

    Validates BOTH start (0x0F) and end (0x00 family) bytes. On a bad
    end byte, drops just the leading 0x0F and resyncs — a 0x0F that
    appears mid-payload of a previous frame is exactly how channel
    shift bugs (CH1 showing as CH12) get introduced.

    The resync loop is bounded so we don't spin forever on a dead line.
    """
    for _ in range(64):
        b = ser.read(1)
        if not b:
            return None
        if b[0] != 0x0F:
            continue
        rest = ser.read(24)
        if len(rest) != 24:
            return None
        frame = b + rest
        if frame[24] in _SBUS_END_BYTES:
            return frame
        # Bad end byte → frame was misaligned. Loop continues looking
        # for the next 0x0F candidate.
    return None


# =============================================================================
#  ROS 2 Node
# =============================================================================

class SbusPublisher(Node):

    def __init__(self):
        super().__init__('sbus_publisher')

        self.declare_parameter('serial_port',   _DEFAULT_PORT)
        self.declare_parameter('baud_rate',     _DEFAULT_BAUD)
        self.declare_parameter('timer_period',  _DEFAULT_TIMER_PERIOD)
        self.declare_parameter('frame_timeout', _DEFAULT_FRAME_TIMEOUT)
        self.declare_parameter('open_timeout',  _DEFAULT_OPEN_TIMEOUT)

        port           = self.get_parameter('serial_port').value
        baud           = self.get_parameter('baud_rate').value
        timer_period   = self.get_parameter('timer_period').value
        frame_timeout  = self.get_parameter('frame_timeout').value
        open_timeout   = self.get_parameter('open_timeout').value

        self._pub    = self.create_publisher(SbusControl, '/sbus/control', 10)
        self._interp = ChannelInterpreter()
        self._ser: serial.Serial | None = None
        self._port = port
        self._baud = baud
        self._debug_count = 0

        # ── Frame integrity tracking ────────────────────────────────────────
        self._frame_timeout_ns      = int(frame_timeout * 1e9)
        self._last_good_frame_ns    = 0       # last time a clean frame arrived
        self._safe_state_active     = False   # True while in link-lost state
        self._last_safe_publish_ns  = 0       # rate-limit for safe-state publish

        # Retry the first open: at boot the node can start before the UART is
        # ready (overlay still settling, serial-getty still releasing the line),
        # and a single failed open here used to kill the node for the whole
        # session — nothing respawns it. Later I/O failures are handled by
        # _reopen_serial() from the timer callback.
        self._ser = self._open_serial_with_retry(port, baud, open_timeout)

        self._timer = self.create_timer(timer_period, self._timer_cb)
        self.get_logger().info(
            f'sbus_publisher ready — /sbus/control @ {1.0/timer_period:.0f} Hz '
            f'(frame timeout = {frame_timeout*1000:.0f} ms)')

    # ── Timer callback ────────────────────────────────────────────────────────

    def _timer_cb(self) -> None:
        now_ns = self.get_clock().now().nanoseconds

        # Read one validated frame from the serial port. SerialException
        # on a real I/O failure (USB unplug etc.) → reopen the line.
        try:
            frame = _read_one_frame(self._ser)
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial read error: {exc}')
            self._reopen_serial()
            return

        # No frame available this tick. Force safe-state if silent too long.
        if frame is None:
            if now_ns - self._last_good_frame_ns > self._frame_timeout_ns:
                self._publish_safe_state(now_ns, 'no frames')
            return

        decoded = decode_sbus_frame(frame)
        if decoded is None:
            # Bad alignment / channels-out-of-range / bad end byte. SIYI air
            # unit emits this pattern when ground unit is off and the
            # failsafe flag is no longer being set.
            if now_ns - self._last_good_frame_ns > self._frame_timeout_ns:
                self._publish_safe_state(now_ns, 'decode failed')
            return

        # Valid frame, but receiver explicitly tells us link is down.
        # Do NOT reopen the serial port — the line is fine, only the
        # radio link is gone. Reopening would trash byte alignment.
        if decoded['failsafe'] or decoded['frame_lost']:
            reason = 'failsafe' if decoded['failsafe'] else 'frame_lost'
            self._publish_safe_state(now_ns, reason)
            return

        # ── Good frame ───────────────────────────────────────────────────
        if self._safe_state_active:
            self.get_logger().info('SBUS link restored')
            self._safe_state_active = False
        self._last_good_frame_ns = now_ns

        # Debug: print all 16 raw channel values periodically (~every 250 ms)
        ch = decoded['channels']
        self._debug_count += 1
        if self._debug_count % 50 == 0:
            self.get_logger().debug(
                f"CH1={ch[0]:4d} CH2={ch[1]:4d} CH3={ch[2]:4d} CH4={ch[3]:4d} "
                f"CH5={ch[4]:4d} CH6={ch[5]:4d} CH7={ch[6]:4d} CH8={ch[7]:4d} "
                f"CH9={ch[8]:4d} CH10={ch[9]:4d} CH11={ch[10]:4d} CH12={ch[11]:4d} "
                f"CH13={ch[12]:4d} CH14={ch[13]:4d} CH15={ch[14]:4d} CH16={ch[15]:4d}")

        out = self._interp.interpret(ch)
        self._publish(out)

    # ── Publish helpers ───────────────────────────────────────────────────────

    def _publish(self, out: dict) -> None:
        """Publish a fully-populated SbusControl message from the interpreter output."""
        def r(v: float) -> float:
            return round(float(v), 3)

        msg = SbusControl()
        msg.header.stamp             = self.get_clock().now().to_msg()
        msg.header.frame_id          = 'sbus'
        msg.control_mode             = int(out['control_mode'])
        msg.operation_mode           = int(out['operation_mode'])
        msg.drive_left               = r(out['drive_left'])
        msg.drive_right              = r(out['drive_right'])
        msg.drive_speed              = int(out['drive_speed'])
        msg.arm_x_cmd                = int(out['arm_x_cmd'])
        msg.arm_y_cmd                = int(out['arm_y_cmd'])
        msg.arm_z_cmd                = int(out['arm_z_cmd'])
        msg.ee_roll                  = int(out['ee_roll'])
        msg.ee_pitch                 = int(out['ee_pitch'])
        msg.claw_cmd                 = r(out['claw_cmd'])
        msg.telescope_cmd            = int(out['telescope_cmd'])
        msg.rear_flipper             = int(out['rear_flipper'])
        msg.front_flipper            = int(out['front_flipper'])
        msg.light_state              = int(out['light_state'])
        msg.camera_axis              = int(out['camera_axis'])
        msg.camera_cmd               = int(out['camera_cmd'])
        self._pub.publish(msg)

    def _publish_safe_state(self, now_ns: int, reason: str) -> None:
        """
        Publish an explicit DISARMED message with every command zeroed,
        except light_state / camera_axis which carry their last-known values.

        Called when the radio link is lost (failsafe flag, frame_lost flag,
        no frames within the timeout window, or decode/sanity failure).

        Logs ONCE on entry to safe state. Republishes at 20 Hz so
        downstream subscribers always have a fresh "link is down"
        message but we don't flood at 200 Hz timer rate.
        """
        if not self._safe_state_active:
            self.get_logger().warn(
                f'SBUS link lost ({reason}) — publishing safe state')
            self._safe_state_active = True
            self._last_safe_publish_ns = 0   # force first publish immediately

        if now_ns - self._last_safe_publish_ns < _SAFE_STATE_MIN_PERIOD_NS:
            return
        self._last_safe_publish_ns = now_ns

        msg = SbusControl()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sbus'
        msg.control_mode    = _MODE_ARM         # benign default
        msg.operation_mode  = _OP_DISARMED      # critical: gates everything off
        # Light and camera axis survive the link loss — neither commands motion,
        # and keeping the lights on makes the robot findable when the radio is
        # gone. Both default to 0 in ChannelInterpreter.__init__, so before the
        # first frame this still publishes lights-off / pan.
        msg.light_state     = int(self._interp.light_state)
        msg.camera_axis     = int(self._interp.camera_axis)
        # All other fields stay at their zero defaults from msg construction.
        self._pub.publish(msg)

    # ── Serial recovery ───────────────────────────────────────────────────────

    def _open_serial_with_retry(self, port: str, baud: int,
                                timeout_s: float) -> serial.Serial:
        """
        Open the port, retrying for up to timeout_s before giving up.

        Used only for the FIRST open, where the port may simply not exist yet
        on a cold boot. Raises the last SerialException if the window expires.
        """
        deadline = time.monotonic() + timeout_s
        attempt = 0
        while True:
            attempt += 1
            try:
                ser = _open_serial(port, baud)
                if attempt > 1:
                    self.get_logger().info(
                        f'SBUS serial opened on attempt {attempt}: {port} @ {baud} 8E2')
                else:
                    self.get_logger().info(f'SBUS serial opened: {port} @ {baud} 8E2')
                return ser
            except serial.SerialException as exc:
                if time.monotonic() >= deadline:
                    self.get_logger().error(
                        f'Cannot open serial port after {timeout_s:.0f}s '
                        f'({attempt} attempts): {exc}')
                    raise
                self.get_logger().warning(
                    f'Serial port {port} not ready (attempt {attempt}): {exc} '
                    f'— retrying in {_OPEN_RETRY_PERIOD:.0f}s')
                time.sleep(_OPEN_RETRY_PERIOD)

    def _reopen_serial(self) -> None:
        """
        Reopen the serial port — only on actual I/O failure (USB unplug,
        permission revoked, etc.). NEVER on radio failsafe — the line is
        fine then and reopening corrupts byte alignment.
        """
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass
        time.sleep(0.3)
        try:
            self._ser = _open_serial(self._port, self._baud)
            self.get_logger().info('Serial port reopened successfully')
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial reopen failed: {exc}')

    def destroy_node(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()
            self.get_logger().info('Serial port closed')
        super().destroy_node()


# =============================================================================
#  Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = SbusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
