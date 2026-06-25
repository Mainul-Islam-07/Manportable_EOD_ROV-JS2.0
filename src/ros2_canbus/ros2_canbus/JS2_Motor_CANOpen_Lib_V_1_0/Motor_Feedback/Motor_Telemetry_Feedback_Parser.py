import canopen, time
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Logger_Lib import Logger
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Common_Lib import Common
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Mapping.data import statistics, StatuswordShared, errorcode, errorregister, heartbeat
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Mapping.mapping import Avatarrobot_CANopen_Map


class Parser:
    def __init__(self, node: canopen.RemoteNode, node_name: str):
        self.node = node
        self.Node_ID = self.node.id
        self.Node_Name = node_name
        self.common = Common(self.Node_ID, self.Node_Name)
        self.log = Logger(self.Node_Name, self.Node_ID)
        self.data = statistics()
        self._hb_last_ts = None
        self.hbcount = 0


    # ------------------------- Error_register + 603f -------------------------
    def parse_and_flood_error_register_error_code(self, error_register: int, error_code: int):
        """0x1001 (Error_register) + 0x603F (Error condition bitfield)."""
        md = self.data.metadata

        # 0x1001 — standard CiA-301
        er_val = int(error_register) & 0xFF
        md.errorregister.raw = er_val
        # er = md.errorregister.parsed

        md.errorregister.parsed.generic        = bool(er_val & (1 << 0))
        md.errorregister.parsed.current        = bool(er_val & (1 << 1))
        md.errorregister.parsed.voltage        = bool(er_val & (1 << 2))
        md.errorregister.parsed.temperature    = bool(er_val & (1 << 3))
        md.errorregister.parsed.communication  = bool(er_val & (1 << 4))
        md.errorregister.parsed.device_profile = bool(er_val & (1 << 5))
        md.errorregister.parsed.reserved       = bool(er_val & (1 << 6))
        md.errorregister.parsed.manufacturer   = bool(er_val & (1 << 7))
        # self.print_errorregister(er_val, er)

        # 0x603F — vendor bitfield (datasheet: bits 0,1,2,4,5,6,16,17,18)
        ec_val = int(error_code) & 0xFFFFFFFF
        md.errorcode.raw = ec_val
        ec = md.errorcode.parsed

        md.errorcode.parsed.software_error_flash = bool(ec_val & (1 << 0))
        md.errorcode.parsed.startuperror         = bool(ec_val & (1 << 1))
        md.errorcode.parsed.speedfeedbackerror   = bool(ec_val & (1 << 2))
        md.errorcode.parsed.overvoltage          = bool(ec_val & (1 << 4))
        md.errorcode.parsed.undervoltage         = bool(ec_val & (1 << 5))
        md.errorcode.parsed.overflow             = bool(ec_val & (1 << 6))
        md.errorcode.parsed.encodercommunication = bool(ec_val & (1 << 16))
        md.errorcode.parsed.motor_temp_high      = bool(ec_val & (1 << 17))
        md.errorcode.parsed.board_temp_high      = bool(ec_val & (1 << 18))

        # self.print_errorcode(ec_val, ec)
        return md.errorregister, md.errorcode

    def print_errorregister(self, error_register: errorregister):
        error_register_value = error_register.raw
        er = error_register.parsed
        self.log.print(
            f"Error_register=0x{error_register_value:02X} -> "
            f"GEN={er.generic} CUR={er.current} VOL={er.voltage} TEMP={er.temperature} "
            f"COM={er.communication} DEV={er.device_profile} RES={er.reserved} MAN={er.manufacturer}",
            "SDO", "parse_and_flood_error_register"
        )

    def print_errorcode(self, error_code: errorcode):
        error_code_value = error_code.raw
        ec = error_code.parsed
        self.log.print(
            f"Error Code=0x{error_code_value:04X} -> "
            f"SW={ec.software_error_flash} OV={ec.overvoltage} UV={ec.undervoltage} "
            f"SU={ec.startuperror} SFE={ec.speedfeedbackerror} OF={ec.overflow} "
            f"ENC={ec.encodercommunication} MT={ec.motor_temp_high} BT={ec.board_temp_high}",
            "SDO", "parse_and_flood_error_code"
        )

    # ------------------------- Statusword + state machine -------------------------
    def print_statusword(self, shared: StatuswordShared) -> None:
        sw = shared.sw
        # full snapshot
        self.log.print(
            f"Statusword=0x{shared.raw:04X} -> "
            f"RTSO={sw.ready_to_switch_on} SO={sw.switched_on} OE={sw.operation_enabled} FAULT={sw.fault} "
            f"VE={sw.voltage_enabled} QS={sw.quick_stop} SOD={sw.switch_on_disabled} WARN={sw.warning} "
            f"R8={getattr(sw, 'reserved_8', False)} REM={sw.remote} TR={sw.target_reached} "
            f"ILA={sw.internal_limit_active} SPA={sw.set_point_acknowledge} FE={sw.following_error} "
            f"HA={sw.homing_attained} HE={sw.homing_error}",
            "TPDO3", "STATUSWORD"
        )
        # mode-specific
        if shared.mode_bucket == "POSITION":
            self.log.print(
                f"Mode=POSITION (mode={shared.mode}) → "
                f"TargetReached={sw.target_reached}, SetPointAck={sw.set_point_acknowledge}, "
                f"FollowingError={sw.following_error}, InternalLimit={sw.internal_limit_active}, "
                f"QSActive={shared.qs_active}",
                "TPDO3", "STATUSWORD_MODE_BITS", ""
            )
        elif shared.mode_bucket == "VELOCITY":
            self.log.print(
                f"Mode=VELOCITY → VelTargetReached={sw.target_reached}, "
                f"VelAttained={sw.set_point_acknowledge}, OverspeedOrVelError={sw.following_error}, "
                f"InternalLimit={sw.internal_limit_active}, QSActive={shared.qs_active}",
                "TPDO3", "STATUSWORD_MODE_BITS", ""
            )
        elif shared.mode_bucket == "TORQUE":
            self.log.print(
                f"Mode=TORQUE → TorqueTargetReached={sw.target_reached}, "
                f"Bit12(n/a)={sw.set_point_acknowledge}, Bit13(n/a)={sw.following_error}, "
                f"InternalLimit={sw.internal_limit_active}, QSActive={shared.qs_active}",
                "TPDO3", "STATUSWORD_MODE_BITS", ""
            )
        else:
            self.log.print(
                f"Mode=IDLE/UNKNOWN → Bit10={sw.target_reached}, "
                f"Bit12={sw.set_point_acknowledge}, Bit13={sw.following_error}, QSActive={shared.qs_active}",
                "TPDO3", "STATUSWORD_MODE_BITS", ""
            )
        # final state
        self.log.print(
            f"State={getattr(shared.state, 'name', shared.state)} "
            f"(low4=0x{shared.low4:X}, b5={shared.b5}, b8={shared.b8})",
            "TPDO3", "STATE"
        )

    def parse_statusword_state(self, status: int, mode: int):
        """
        Decode 0x6041 and decide CiA-402 state; update metadata.statusword;
        print via StatuswordShared; return (sw, state).
        """
        md = self.data.metadata
        v = int(status) & 0xFFFF
        sw = md.statusword.sw  # parsed bool container

        # bit decode
        sw.ready_to_switch_on    = bool(v & (1 << 0))
        sw.switched_on           = bool(v & (1 << 1))
        sw.operation_enabled     = bool(v & (1 << 2))
        sw.fault                 = bool(v & (1 << 3))
        sw.voltage_enabled       = bool(v & (1 << 4))
        sw.quick_stop            = bool(v & (1 << 5))   # 1 ⇒ QS NOT active
        sw.switch_on_disabled    = bool(v & (1 << 6))
        sw.warning               = bool(v & (1 << 7))
        sw.reserved_8            = bool(v & (1 << 8))
        sw.remote                = bool(v & (1 << 9))
        sw.target_reached        = bool(v & (1 << 10))
        sw.internal_limit_active = bool(v & (1 << 11))
        sw.set_point_acknowledge = bool(v & (1 << 12))
        sw.following_error       = bool(v & (1 << 13))
        sw.homing_attained       = bool(v & (1 << 14))
        sw.homing_error          = bool(v & (1 << 15))

        # helpers
        low4 = ((1 if sw.fault else 0) << 3 |
                (1 if sw.operation_enabled else 0) << 2 |
                (1 if sw.switched_on else 0) << 1 |
                 (1 if sw.ready_to_switch_on else 0))
        b5 = 1 if sw.quick_stop else 0
        b8 = 1 if getattr(sw, "vendor_state_bit_8", getattr(sw, "reserved_8", False)) else 0
        qs_active = not sw.quick_stop

        # mode bucket
        M = Avatarrobot_CANopen_Map.ModesOfOperation
        if mode in {M.CYCLIC_SYNCHRONOUS_POSITION_MODE, M.PROFILE_POSITION_MODE, M.POSITION_INTERPOLATION_MODE}:
            bucket = "POSITION"
        elif mode in {M.CYCLIC_SYNCHRONOUS_VELOCITY_MODE, M.PROFILE_VELOCITY_MODE, M.VELOCITY_MODE}:
            bucket = "VELOCITY"
        elif mode in {M.CYCLIC_SYNCHRONOUS_TORQUE_MODE, M.PROFILE_TORQUE_MODE}:
            bucket = "TORQUE"
        else:
            bucket = "IDLE"

        # CiA-402 state decision
        S = Avatarrobot_CANopen_Map.StateMachineState
        if   low4 == 0xF: st = S.FAULT_REACTION_ACTIVE
        elif low4 == 0x8: st = S.FAULT
        elif low4 == 0x0: st = S.SWITCH_ON_DISABLED if b8 else S.NOT_READY_TO_SWITCH_ON
        elif b5 == 1 and low4 == 0x1: st = S.READY_TO_SWITCH_ON
        elif b5 == 1 and low4 == 0x3: st = S.SWITCHED_ON
        elif b5 == 1 and low4 == 0x7: st = S.OPERATION_ENABLED
        elif b5 == 0 and low4 == 0x7: st = S.QUICK_STOP_ACTIVE
        else:                         st = getattr(S, "UNKNOWN", S.SWITCH_ON_DISABLED)

        # update snapshot in metadata
        md.statusword.raw = v
        md.statusword.mode = mode
        md.statusword.state = st
        md.statusword.low4 = low4
        md.statusword.b5 = b5
        md.statusword.b8 = b8
        md.statusword.qs_active = qs_active
        md.statusword.mode_bucket = bucket

        # print using snapshot
        # self.print_statusword(md.statusword)

        # return parsed bits + state (for callers that rely on the tuple)
        return md.statusword

    def decode_heartbeat(self, data) -> Avatarrobot_CANopen_Map.HeartbeatState:
        """
        Decode CANopen heartbeat producer byte into a HeartbeatState.
        data: int or bytes/bytearray (len>=1)
        Returns: HeartbeatState (for known codes) or the raw int for unknown/vendor codes.
        """
        if isinstance(data, (bytes, bytearray)):
            if not data:
                raise ValueError("empty heartbeat payload")
            val = data[0]

        else:
            val = int(data) & 0xFF

        hb = self.data.metadata.heartbeat

        # before setting hb.interval
        now = time.monotonic_ns()
        last = getattr(self, "_hb_last_ts", None)
        hb.interval = 0 if last is None else max(0, now - last)
        self._hb_last_ts = now

        self.hbcount+=1
        hb.state = Avatarrobot_CANopen_Map.HeartbeatState(val)
        hb.count = self.hbcount
        try:
            return hb
        except ValueError:
            return None  # unknown code; caller can log e.g. f"UNKNOWN(0x{val:02X})"

    def print_heartbeat(self, heartbeat: heartbeat):
        self.log.print(f"Count = {heartbeat.count:,}, State = 0x{heartbeat.state:04X}, Period = {heartbeat.interval:,} ns", "❤️ ","#")