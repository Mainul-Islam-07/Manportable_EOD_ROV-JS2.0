"""
diagnostics.py
==============
CANopen / CiA-402 diagnostic helpers.

Every function here is pure — it reads telemetry attributes from
a Motor_CANopen_Lib object and returns ROS DiagnosticStatus values.
Imported by the heartbeat node (sole publisher of /motor_diagnostics).
"""

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


# ── tiny helpers ────────────────────────────────────────────────────────

def kv(key, value):
    return KeyValue(key=str(key), value="--" if value is None else str(value))


_HEARTBEAT_STATES = {
    0x00: "BOOTUP",
    0x04: "STOPPED",
    0x05: "OPERATIONAL",
    0x7F: "PRE_OPERATIONAL",
}


def humanize_heartbeat_state(raw):
    if not isinstance(raw, int):
        return None
    return _HEARTBEAT_STATES.get(raw, f"UNKNOWN(0x{raw:02X})")


# ── CiA-402 statusword ─────────────────────────────────────────────────

def decode_cia402_state(sw_raw):
    if not isinstance(sw_raw, int):
        return None
    s = sw_raw & 0x6F
    if s & 0x40:                   return "SWITCH_ON_DISABLED"
    if (s & 0x0F) == 0x00:        return "NOT_READY_TO_SWITCH_ON"
    if (s & 0x6F) == 0x21:        return "READY_TO_SWITCH_ON"
    if (s & 0x6F) == 0x23:        return "SWITCHED_ON"
    if (s & 0x6F) == 0x27:        return "OPERATION_ENABLED"
    if (s & 0x6F) == 0x07:        return "QUICK_STOP_ACTIVE"
    if (sw_raw & 0x4F) == 0x0F:   return "FAULT_REACTION_ACTIVE"
    if (sw_raw & 0x4F) == 0x08:   return "FAULT"
    return f"UNKNOWN(0x{sw_raw:04X})"


def humanize_statusword(sw_shared):
    sw_raw = getattr(sw_shared, "raw", None)
    state_obj = getattr(sw_shared, "state", None)
    state_name = (getattr(state_obj, "name", None)
                  if state_obj is not None else None) \
                 or decode_cia402_state(sw_raw)
    flags = []
    sw = getattr(sw_shared, "sw", None)
    if sw is not None:
        if getattr(sw, "operation_enabled",     False): flags.append("OE")
        if getattr(sw, "switched_on",           False): flags.append("SO")
        if getattr(sw, "ready_to_switch_on",    False): flags.append("RTSO")
        if getattr(sw, "voltage_enabled",       False): flags.append("VE")
        if getattr(sw, "fault",                 False): flags.append("FAULT")
        if getattr(sw, "warning",               False): flags.append("WARN")
        if getattr(sw, "target_reached",        False): flags.append("TR")
        if getattr(sw, "switch_on_disabled",    False): flags.append("SOD")
        if not getattr(sw, "quick_stop",         True): flags.append("QS_ACTIVE")
        if getattr(sw, "internal_limit_active", False): flags.append("ILA")
        if getattr(sw, "following_error",       False): flags.append("FE")
    return state_name, (",".join(flags) if flags else "none")


# ── error register / error code ─────────────────────────────────────────

def humanize_errorregister(er):
    p = getattr(er, "parsed", None)
    if p is None:
        return None
    flags = []
    if getattr(p, "generic",        False): flags.append("GENERIC")
    if getattr(p, "current",        False): flags.append("CURRENT")
    if getattr(p, "voltage",        False): flags.append("VOLTAGE")
    if getattr(p, "temperature",    False): flags.append("TEMP")
    if getattr(p, "communication",  False): flags.append("COMM")
    if getattr(p, "device_profile", False): flags.append("DEVICE_PROFILE")
    if getattr(p, "manufacturer",   False): flags.append("MFG")
    return ",".join(flags) if flags else "none"


def humanize_errorcode(ec):
    p = getattr(ec, "parsed", None)
    if p is None:
        return None
    flags = []
    if getattr(p, "software_error_flash", False): flags.append("SW_FLASH")
    if getattr(p, "overvoltage",          False): flags.append("OVERVOLT")
    if getattr(p, "undervoltage",         False): flags.append("UNDERVOLT")
    if getattr(p, "startuperror",         False): flags.append("STARTUP")
    if getattr(p, "speedfeedbackerror",   False): flags.append("SPEED_FB")
    if getattr(p, "overflow",             False): flags.append("OVERFLOW")
    if getattr(p, "encodercommunication", False): flags.append("ENC_COMM")
    if getattr(p, "motor_temp_high",      False): flags.append("MOTOR_TEMP")
    if getattr(p, "board_temp_high",      False): flags.append("BOARD_TEMP")
    return ",".join(flags) if flags else "none"


# ── raw-int flag decoders ───────────────────────────────────────────────
# These decode the SAME flag strings as humanize_statusword / _errorregister
# / _errorcode above, but from a plain integer instead of a parsed bitfield
# object.  They let downstream consumers that only receive the raw hex value
# (e.g. telemetry_udp_bridge) reproduce the human-readable flags, so the wire
# message can omit the derived strings.  Bit positions mirror
# Motor_Telemetry_Feedback_Parser; label set/order mirror the humanize_* fns.

def statusword_flags_from_raw(v):
    """CiA-402 statusword (0x6041) flag list from a raw int."""
    if not isinstance(v, int):
        return "--"
    flags = []
    if v & (1 << 2):       flags.append("OE")
    if v & (1 << 1):       flags.append("SO")
    if v & (1 << 0):       flags.append("RTSO")
    if v & (1 << 4):       flags.append("VE")
    if v & (1 << 3):       flags.append("FAULT")
    if v & (1 << 7):       flags.append("WARN")
    if v & (1 << 10):      flags.append("TR")
    if v & (1 << 6):       flags.append("SOD")
    if not (v & (1 << 5)): flags.append("QS_ACTIVE")
    if v & (1 << 11):      flags.append("ILA")
    if v & (1 << 13):      flags.append("FE")
    return ",".join(flags) if flags else "none"


def errorregister_flags_from_raw(v):
    """CiA-301 error register (0x1001) flag list from a raw int."""
    if not isinstance(v, int):
        return "--"
    flags = []
    if v & (1 << 0): flags.append("GENERIC")
    if v & (1 << 1): flags.append("CURRENT")
    if v & (1 << 2): flags.append("VOLTAGE")
    if v & (1 << 3): flags.append("TEMP")
    if v & (1 << 4): flags.append("COMM")
    if v & (1 << 5): flags.append("DEVICE_PROFILE")
    if v & (1 << 7): flags.append("MFG")
    return ",".join(flags) if flags else "none"


def errorcode_flags_from_raw(v):
    """Vendor error-condition bitfield (0x603F) flag list from a raw int."""
    if not isinstance(v, int):
        return "--"
    flags = []
    if v & (1 << 0):  flags.append("SW_FLASH")
    if v & (1 << 4):  flags.append("OVERVOLT")
    if v & (1 << 5):  flags.append("UNDERVOLT")
    if v & (1 << 1):  flags.append("STARTUP")
    if v & (1 << 2):  flags.append("SPEED_FB")
    if v & (1 << 6):  flags.append("OVERFLOW")
    if v & (1 << 16): flags.append("ENC_COMM")
    if v & (1 << 17): flags.append("MOTOR_TEMP")
    if v & (1 << 18): flags.append("BOARD_TEMP")
    return ",".join(flags) if flags else "none"


# ── health assessment ───────────────────────────────────────────────────

def compute_health(m, cia_state):
    md = m.telemetry.data.metadata
    sw = getattr(md.statusword, "sw", None)
    if sw is None:
        return DiagnosticStatus.STALE, "no statusword yet"
    if getattr(sw, "fault", False):
        return DiagnosticStatus.ERROR, f"FAULT  (state={cia_state or '?'})"
    if getattr(sw, "warning", False):
        return DiagnosticStatus.WARN, f"WARNING bit set  (state={cia_state or '?'})"
    ec_raw = getattr(md.errorcode, "raw", None) or 0
    if ec_raw:
        return DiagnosticStatus.WARN, f"errorcode=0x{ec_raw:04X}  (state={cia_state or '?'})"
    return DiagnosticStatus.OK, cia_state or "Ready"


# ── global fault check ──────────────────────────────────────────────────

def is_motor_faulted(m) -> tuple:
    """Check whether a motor is in a fault state that should stop execution.

    Returns
    -------
    faulted : bool
    reason  : str   (empty when *faulted* is False)
    """
    try:
        md = m.telemetry.data.metadata
    except Exception as e:
        return True, f"telemetry unreadable ({e})"

    sw = getattr(md.statusword, "sw", None)
    if sw is not None and getattr(sw, "fault", False):
        sw_raw = getattr(md.statusword, "raw", None)
        return True, f"statusword FAULT (0x{sw_raw:04X})" if isinstance(sw_raw, int) else "statusword FAULT"

    er_raw = getattr(md.errorregister, "raw", None)
    if isinstance(er_raw, int) and er_raw != 0:
        return True, f"error register 0x{er_raw:02X} ({humanize_errorregister(md.errorregister)})"

    ec_raw = getattr(md.errorcode, "raw", None)
    if isinstance(ec_raw, int) and ec_raw != 0:
        return True, f"error code 0x{ec_raw:04X} ({humanize_errorcode(md.errorcode)})"

    return False, ""


# ── top-level builder ───────────────────────────────────────────────────

def build_diag_status(name, m, bus_label: str = ""):
    """Build a DiagnosticStatus for motor *name* using its
    Motor_CANopen_Lib telemetry."""
    md = m.telemetry.data.metadata
    sw_raw = getattr(md.statusword, "raw", None)
    ec_raw = getattr(md.errorcode, "raw", None)
    er_raw = getattr(md.errorregister, "raw", None)
    hb     = getattr(md, "heartbeat", None)
    hb_state_raw = getattr(hb, "state", None) if hb is not None else None

    # cia_state is needed for the health summary; the derived flag/state
    # strings are NO LONGER put on the wire — consumers recompute them from
    # the raw hex values via the *_from_raw helpers above (saves bandwidth).
    cia_state, _   = humanize_statusword(md.statusword)
    level, message = compute_health(m, cia_state)

    status = DiagnosticStatus()
    status.level       = level
    status.name        = name
    status.message     = message
    status.hardware_id = str(getattr(m, "Node_ID", "?"))
    status.values = [
        kv("bus",                bus_label),
        kv("voltage_raw",        md.voltage),
        kv("current_raw",        md.current),
        kv("coil_temperature",   md.coiltemperature),
        kv("board_temperature",  md.circuittemperature),
        kv("statusword",         f"0x{sw_raw:04X}" if isinstance(sw_raw, int) else None),
        kv("errorregister",      f"0x{er_raw:02X}" if isinstance(er_raw, int) else None),
        kv("errorcode",          f"0x{ec_raw:04X}" if isinstance(ec_raw, int) else None),
        kv("heartbeat_state",    f"0x{hb_state_raw:02X}" if isinstance(hb_state_raw, int) else None),
        kv("heartbeat_count",    getattr(hb, "count", None)),
    ]
    return status
