# Parse data
    # It is best to go to "Motor_Feedback/Motor_Telemetry_Feedback_Parser.py" and read it. The following part is boring, and best to just copy it into codes
## Parse status
    """
    Decode 0x6041 and decide CiA-402 state; update metadata.statusword;
    print via StatuswordShared; return (sw, state).
    """
    md = self.data.metadata
    v = int(status) & 0xFFFF
    sw = md.statusword.sw  # parsed bool container
### Parse statusword
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
### Parse state
    # helpers (taken from statusword)
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
    md.statusword.state = st #<<< This is the state of the hardware (output of "CiA-402 state decision")
    md.statusword.low4 = low4
    md.statusword.b5 = b5
    md.statusword.b8 = b8
    md.statusword.qs_active = qs_active
    md.statusword.mode_bucket = bucket

    # print using snapshot
    # self.print_statusword(md.statusword)

    # return parsed bits + state (for callers that rely on the tuple)
    return md.statusword
## Parse Error
    """0x1001 (Error_register) + 0x603F (Error condition bitfield)."""
        md = self.data.metadata
### Parse Error Register
    # 0x1001 — standard CiA-301 error list
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
### Parse Error Code
    # 0x603F — vendor specific error list bitfield (avatarrobot canopen motor datasheet: bits 0,1,2,4,5,6,16,17,18)
    ec_val = int(error_code) & 0xFFFFFFFF
    md.errorcode.raw = ec_val
    ec = md.errorcode.parsed

    md.errorcode.parsed.software_error_flash = bool(ec_val & (1 << 0))
    md.errorcode.parsed.overvoltage          = bool(ec_val & (1 << 1))
    md.errorcode.parsed.undervoltage         = bool(ec_val & (1 << 2))
    md.errorcode.parsed.startuperror         = bool(ec_val & (1 << 4))
    md.errorcode.parsed.speedfeedbackerror   = bool(ec_val & (1 << 5))
    md.errorcode.parsed.overflow             = bool(ec_val & (1 << 6))
    md.errorcode.parsed.encodercommunication = bool(ec_val & (1 << 16))
    md.errorcode.parsed.motor_temp_high      = bool(ec_val & (1 << 17))
    md.errorcode.parsed.board_temp_high      = bool(ec_val & (1 << 18))

    # self.print_errorcode(ec_val, ec)
