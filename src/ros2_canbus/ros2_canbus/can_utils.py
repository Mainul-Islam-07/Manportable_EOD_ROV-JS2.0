"""
can_utils.py
============
CAN network lifecycle helpers:  bring-up, safe disarm, disconnect.

CAN bus configuration (interface, channel, bitrate) lives in the JSON
files that the CANopen library reads directly (``Arm_CAN_Config.json``,
``Drive_CAN_Config.json``).  This module never writes or modifies those
files.
"""

import time

from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.CANopen_Network.Network_Lib import CANopen_Network
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.Motor_Lib import Motor_CANopen_Lib
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.Motor_Lib_Heartbeat import Motor_Heartbeat
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Feedback.Motor_Heartbeat_Lib import Heartbeat_Lib
from ros2_canbus.motor_config import load_motor_configs


# ── network lifecycle ──────────────────────────────────────────────────

def create_network(config_name: str, master_role: str,
                   master_node_id: int) -> CANopen_Network:
    """Create, reset and move a CANopen network to pre-operational."""
    net = CANopen_Network(config_name, master_role, master_node_id)
    net.network_reset()
    net.network_preoperational()
    return net


def set_operational(net: CANopen_Network):
    """Transition network to operational state."""
    net.network_operational()
    time.sleep(0.1)


# ── heartbeat scan ─────────────────────────────────────────────────────

def scan_heartbeat(active_names: list, net: CANopen_Network,
                   settings_file: str,
                   wait_s: float, tag: str = "") -> tuple:
    """Create Motor_Heartbeat monitors, wait, return (alive, unavailable).

    Returns
    -------
    alive : list[str]
        Motor names that responded with a heartbeat.
    unavailable : dict[str, str]
        ``{motor_name: reason}`` for motors that did NOT respond.
    """
    prefix = f"[{tag}-HB]" if tag else "[HB]"

    heartbeat = {}
    monitor_fail = []
    for name in active_names:
        try:
            heartbeat[name] = Motor_Heartbeat(name, net, settings_file)
        except Exception as e:
            monitor_fail.append(f"{name}({e})")

    if monitor_fail:
        print(f"{prefix} Monitor FAILED: {monitor_fail}")
    print(f"{prefix} Scanning {len(heartbeat)}/{len(active_names)} motors ({wait_s}s)...")
    time.sleep(wait_s)

    alive = []
    unavailable = {}
    for name in active_names:
        if name not in heartbeat:
            unavailable[name] = "heartbeat monitor creation failed"
            continue
        if heartbeat[name].heartbeat.is_heartbeat:
            alive.append(name)
        else:
            unavailable[name] = "no heartbeat detected"

    print(f"{prefix} Alive: {alive or '(none)'}")
    if unavailable:
        print(f"{prefix} Unavailable: {list(unavailable.keys())}")

    for name, hb_obj in heartbeat.items():
        try:
            hb_obj.heartbeat.close()
        except Exception:
            pass
        # Remove the node from python-canopen network so Motor_CANopen_Lib
        # gets a clean RemoteNode (not a stale one from the scan phase)
        try:
            node_id = hb_obj.settings.Node_ID
            if node_id in net.network:
                del net.network[node_id]
        except Exception:
            pass

    return alive, unavailable


# ── motor initialisation ───────────────────────────────────────────────

def init_motors(names: list, net: CANopen_Network,
                settings_file: str, tag: str = "",
                max_retries: int = 3, retry_delay_s: float = 1.0) -> tuple:
    """Create Motor_CANopen_Lib for each name.

    On a cold boot the Avatar drives may not have latched their mode by
    the time ``Motor_CANopen_Lib.__init__`` reads it back.  If
    ``FAILURE_EXIT`` is set in the motor settings the library calls
    ``sys.exit()`` (raises ``SystemExit``).  This function catches that,
    cleans up the partially-created node, waits, and retries — giving
    the drive time to latch.

    Returns
    -------
    motors : dict[str, Motor_CANopen_Lib]
    failed : dict[str, str]
    """
    prefix = f"[{tag}-INIT]" if tag else "[INIT]"

    # Build name → node_id map so we can clean up partial nodes on retry
    node_ids = {}
    try:
        cfgs = load_motor_configs(settings_file, set(names))
        node_ids = {n: c.node_id for n, c in cfgs.items()}
    except Exception as e:
        print(f"{prefix} WARNING: could not pre-load node_ids ({e}), "
              f"retry cleanup may be incomplete")

    motors = {}
    failed = {}
    for name in names:
        last_error = None
        for attempt in range(1, max_retries + 1):
            try:
                motors[name] = Motor_CANopen_Lib(name, net, settings_file)
                last_error = None
                break  # success
            except SystemExit as e:
                last_error = (f"FAILURE_EXIT on attempt {attempt}/{max_retries}"
                              f" (exit code {e.code})")
                # Remove the partially-created node from the network
                # so the next attempt gets a clean RemoteNode
                nid = node_ids.get(name)
                if nid is not None and nid in net.network:
                    try:
                        del net.network[nid]
                    except Exception:
                        pass
                if attempt < max_retries:
                    print(f"{prefix} {name}: mode mismatch (attempt "
                          f"{attempt}/{max_retries}), retrying in "
                          f"{retry_delay_s}s...")
                    time.sleep(retry_delay_s)
            except Exception as e:
                last_error = str(e)
                break  # non-retryable error

        if last_error:
            failed[name] = last_error

    print(f"{prefix} Init OK: {list(motors.keys()) or '(none)'}")
    if failed:
        print(f"{prefix} Init FAILED: {failed}")
    return motors, failed


# ── heartbeat attachment ───────────────────────────────────────────────

def attach_heartbeat(motors: dict, tag: str = ""):
    """Attach a Heartbeat_Lib to each Motor_CANopen_Lib.

    Motor_CANopen_Lib does not create heartbeat monitoring by default
    (the line is commented out in Motor_Lib.py).  This function manually
    creates a Heartbeat_Lib using the motor's existing node, settings,
    and telemetry objects, then stores it as ``motor.heartbeat``.
    """
    prefix = f"[{tag}-HB-ATTACH]" if tag else "[HB-ATTACH]"
    ok, fail = [], []
    for name, m in motors.items():
        try:
            m.heartbeat = Heartbeat_Lib(
                m.node, m.Node_Name, m.canopen_handle,
                m.settings, m.telemetry,
                int(m.settings.heartbeat_timeout_ms),
            )
            ok.append(name)
        except Exception as e:
            fail.append(f"{name}({e})")
    print(f"{prefix} Attached: {ok or '(none)'}")
    if fail:
        print(f"{prefix} FAILED: {fail}")


# ── mode verification (cold-boot mismatch fix) ────────────────────

def verify_and_fix_modes(motors: dict, tag: str = "",
                         retries: int = 5, delay_s: float = 0.3):
    """Re-check modes_of_operation_display against the intended mode.

    On a cold boot the Avatar drives may not have latched the mode by
    the time Motor_CANopen_Lib reads it back, so Set_Mode_Lib logs a
    "Mismatch" and returns None — leaving operational settings (CSP/CSV/
    CST parameters) unconfigured.

    This function detects that situation and retries the SDO write +
    readback until the drive confirms the correct mode, then re-runs
    ``select_operational_settings`` so that position/velocity/torque
    parameters are actually applied.
    """
    prefix = f"[{tag}-MODE-FIX]" if tag else "[MODE-FIX]"
    for name, m in motors.items():
        intended_mode = m.settings.mode          # int from motor_settings.xlsx
        current_read  = m.telemetry.data.settings.operationmode.read

        # If the library init succeeded, current_read == intended_mode
        if current_read == intended_mode:
            continue

        print(f"{prefix} {name}: mode mismatch detected "
              f"(intended={intended_mode}, got={current_read}). Retrying...")

        fixed = False
        for attempt in range(1, retries + 1):
            try:
                # Re-write the mode
                m.node.sdo['Modes_of_operation'].raw = intended_mode
                time.sleep(delay_s)
                # Re-read display
                confirmed = m.node.sdo['Modes_of_operation_display'].raw
                if confirmed == intended_mode:
                    # Patch the telemetry so the rest of the stack is consistent
                    m.telemetry.data.settings.operationmode.read = confirmed
                    # Re-run operational settings (CSP/CSV/CST parameter setup)
                    m.mode.select_operational_settings(confirmed)
                    print(f"{prefix} {name}: mode FIXED to {confirmed} "
                          f"on attempt {attempt}")
                    fixed = True
                    break
                else:
                    print(f"{prefix} {name}: attempt {attempt} "
                          f"still {confirmed}, retrying...")
            except Exception as e:
                print(f"{prefix} {name}: attempt {attempt} error ({e})")
            time.sleep(delay_s)

        if not fixed:
            print(f"{prefix} {name}: WARNING — could not confirm mode "
                  f"{intended_mode} after {retries} attempts!")


# ── safe shutdown ──────────────────────────────────────────────────────

def safe_disarm_all(motors: dict, tag: str = ""):
    """Best-effort DISARM every motor in *motors*."""
    if not motors:
        return
    prefix = f"[{tag}-SAFE]" if tag else "[SAFE]"
    for name, m in motors.items():
        try:
            m.control.DISARM()
        except Exception as e:
            print(f"{prefix} {name} disarm error ({e})")
    print(f"{prefix} Disarmed {len(motors)} motors")


def safe_disconnect(nets: dict, tag: str = ""):
    """Best-effort disconnect every network in *nets*."""
    if not nets:
        return
    prefix = f"[{tag}-CLEAN]" if tag else "[CLEAN]"
    for net_name, n in nets.items():
        try:
            n.disconnect()
            print(f"{prefix} Disconnected {net_name}.")
        except Exception as e:
            print(f"{prefix} Disconnect error on {net_name}: {e}")