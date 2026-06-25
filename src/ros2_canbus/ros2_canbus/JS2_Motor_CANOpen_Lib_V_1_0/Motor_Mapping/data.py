from typing import Optional, Any, Literal
from dataclasses import dataclass, field

# ---------------- normalized mode bucket for bit 10/12/13 semantics ----------------
ModeBucket = Literal["POSITION", "VELOCITY", "TORQUE", "IDLE"]

# ---------------- parsed bitfields for 0x6041 ----------------
class status_parsed:
    def __init__(self):
        self.ready_to_switch_on = False      # bit0
        self.switched_on = False             # bit1
        self.operation_enabled = False       # bit2
        self.fault = False                   # bit3
        self.voltage_enabled = False         # bit4
        self.quick_stop = False              # bit5 (1 ⇒ QS NOT active)
        self.switch_on_disabled = False      # bit6
        self.warning = False                 # bit7
        self.reserved_8 = False              # bit8 (vendor/reserved)
        self.remote = False                  # bit9
        self.target_reached = False          # bit10
        self.internal_limit_active = False   # bit11
        self.set_point_acknowledge = False   # bit12
        self.following_error = False         # bit13
        self.homing_attained = False         # bit14
        self.homing_error = False            # bit15

# ---------------- parser→printer handoff (now the single source of truth) ---------
@dataclass(slots=True)
class StatuswordShared:
    """Immutable-enough snapshot the parser fills and the printer logs."""
    raw: int = 0                              # 0x6041 masked to 16-bit
    mode: int = 0                             # 0x6061 raw mode
    sw: status_parsed = field(default_factory=status_parsed)  # parsed booleans
    state: Any = None                         # final CiA-402 state enum
    low4: int = 0                             # [b3 b2 b1 b0] = [fault, OE, SO, RTSO]
    b5: int = 0                               # bit5 as 0/1 (1 ⇒ QS NOT active)
    b8: int = 0                               # bit8 as 0/1 (vendor/reserved)
    qs_active: bool = False                   # True when quick-stop is active (bit5 == 0)
    mode_bucket: ModeBucket = "IDLE"          # POSITION | VELOCITY | TORQUE | IDLE

# ---------------- other telemetry containers ----------------
class motion:
    def __init__(self):
        self.position: Optional[int] = None
        self.velocity: Optional[int] = None
        self.torque: Optional[int] = None

class direction:
    def __init__(self):
        self.read: Optional[int] = None
        self.write: Optional[int] = None

class settings:
    def __init__(self):
        self.operationmode = direction()
        self.acceleration  = direction()
        self.deceleration  = direction()
        self.torque_slope  = direction()
        self.rated_current  = direction()

class errorregister_parsed:
    def __init__(self):
        self.generic = False          # bit0
        self.current = False          # bit1
        self.voltage = False          # bit2
        self.temperature = False      # bit3
        self.communication = False    # bit4
        self.device_profile = False   # bit5
        self.reserved = False         # bit6
        self.manufacturer = False     # bit7

class errorregister:
    def __init__(self):
        self.raw: Optional[int] = None
        self.parsed = errorregister_parsed()

class errorcode_parsed:
    """
    0x603F — Error condition BITFIELD (per vendor datasheet).
    Bits used: 0,1,2,4,5,6,16,17,18 → exposed below as booleans.
    """
    def __init__(self):
        self.software_error_flash = False  # bit0
        self.overvoltage          = False  # bit1
        self.undervoltage         = False  # bit2
        # bit3 not specified
        self.startuperror         = False  # bit4
        self.speedfeedbackerror   = False  # bit5
        self.overflow             = False  # bit6
        self.encodercommunication = False  # bit16
        self.motor_temp_high      = False  # bit17
        self.board_temp_high      = False  # bit18

class errorcode:
    def __init__(self):
        self.raw: Optional[int] = None
        self.parsed = errorcode_parsed()

class metadata:
    def __init__(self):
        self.current: Optional[int] = None
        self.voltage: Optional[int] = None
        self.power: Optional[int] = None
        self.coiltemperature: Optional[int] = None
        self.circuittemperature: Optional[int] = None

        # statusword snapshot lives here (replaces old statusword() holder)
        self.statusword = StatuswordShared()

        self.errorregister = errorregister()
        self.errorcode = errorcode()

        # derived
        self.state: Optional[int] = None
        self.heartbeat = heartbeat()

class heartbeat:
    def __init__(self):
        self.count :Optional[int] = 0
        self.state :Optional[int] = 0xFF
        self.interval :Optional[int] = 0

class statistics:
    def __init__(self):
        self.command = motion()    # commands
        self.controlword = 0x0     # command
        self.settings = settings() # command + feedback mirrors

        self.feedback = motion()   # feedback
        self.metadata = metadata() # meta + statusword snapshot
