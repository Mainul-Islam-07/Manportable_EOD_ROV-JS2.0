import canopen
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Logger_Lib import Logger
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Common_Lib import Common
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.CANopen_Network.Network_Lib import CANopen_Network
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Settings.Load_Settings_Lib import Load_Settings
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.Motor_PDO_Lib import PDO_Lib
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.Motor_Telemetry_Lib import Motor_Telemetry
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.Motor_PDO_Lib import PDO_Lib

class Position_Setup():
    def __init__(
            self, node: canopen.RemoteNode,
            node_name: str,
            canopen_handle: CANopen_Network,
            settings: Load_Settings,
            telemetry: Motor_Telemetry):
        try:
            self.node = node
            self.Node_ID = self.node.id
            self.Node_Name = node_name
            self.canopen_handle = canopen_handle
            self.settings = settings
            self.telemetry = telemetry
            self.common = Common(self.Node_ID, self.Node_Name)
            self.log = Logger(self.Node_Name, self.Node_ID)
            self.DEBUG_INIT = self.settings.DEBUG_INIT and True
            self.DEBUG_FEEDBACK = self.settings.DEBUG_FEEDBACK and True
            self.log.print(f"Starting Position Profile setup...","INFO", "Position_Setup","__init__", "CYCLIC_SYNCHRONOUS_Position_MODE")
            self.set_position_parameter(self.settings.max_position, self.settings.min_position)
            self.set_position_parameter_success = self.position_limit_settings_IMPORTANT
            self.pdo = PDO_Lib(self.node, self.Node_Name)
            self.configure()

        except Exception as e:
            self.log.print(f"Setup Failure","❌","Position_Setup","__init__",f"{e}")



    def set_position_parameter(self, max_position, min_position):
        try:
            self.position_limit_settings_IMPORTANT = False
            if not self.node:
                raise RuntimeError(f"Node not initialized.")
            if max_position is None or min_position is None:
                raise ValueError(f"Failed to acquire Max and Min position limits for not being a number")
            
            self.node.sdo['Software location limit'][2].raw = max_position
            self.common.delay_SDO()
            self.node.sdo['Software location limit'][1].raw = min_position
            self.common.delay_SDO()
            self.node.sdo['Software location limit'][2].raw = max_position
            self.common.delay_SDO()
            self.DEBUG_INIT and self.log.print(f"Currently requested max_position={max_position} counts, min_position={min_position} counts", "SDO", "POSITION")

            set_max_position = self.node.sdo['Software location limit'][2].raw
            set_min_position = self.node.sdo['Software location limit'][1].raw
            self.DEBUG_INIT and self.log.print(f"Previously configured set_max_position={set_max_position} counts, set_min_position={set_min_position} counts", "SDO", "POSITION")


            if (set_max_position!= max_position or
                set_min_position!= min_position):
                raise RuntimeError(f"Position limits verification failed.")
            else:
                self.DEBUG_INIT and self.log.print(f"Successfully configured max_position={max_position} counts, min_position={min_position} counts", "SDO", "POSITION")
                self.position_limit_settings_IMPORTANT = True

        except Exception as e:
            self.DEBUG_INIT and self.log.print(f"Position limit error: {e}", "❌", "set_position_parameter", "Position_Setup", f"{e}")
            
    def set_position_offset(self, position_offset: int):
        try:
            if not self.node:
                raise RuntimeError(f"Node not initialized.")
            if position_offset is None:
                raise ValueError(f"Failed to acquire position offset for not being a number")
            # Read current values
            A = self.node.sdo["Position_actual_value"].raw   # Position actual value
            B = self.node.sdo["ZeroPositionOffset"].raw   # ZeroPositionOffset
            C = position_offset

            new_offset = int((A + B) - C)

            # Write the new offset
            self.node.sdo["ZeroPositionOffset"].raw = new_offset
            # Read back to verify effect at *same pose*
            A_after = int(self.node.sdo["Position_actual_value"].raw)

            self.DEBUG_INIT and self.log.print(f"[POSITION] After write: 0x2008={new_offset} (set). "
            f"Readback 0x6064={A_after} (should be ≈ {C} at same pose).",
            "SDO", "POSITION")


        except Exception as e:
            self.DEBUG_INIT and self.log.print(f"Position offset error", "ERROR", "set_position_offset", f"{e}")

    def configure(self):
        self.position_rpdo = self.pdo.configure(
            node_id=self.node.id,
            pdo_name="RPDO1",
            variables=["Target_position", "Profile_velocity"],
            trans_type=255,
            event_timer=0,
            enabled=True
        )

    def run(self, target_position: int, profile_velocity: int = 0):
        try:
            if not self.position_rpdo:
                raise RuntimeError(f"position_rpdo 'Target_position' not initialized.")
            
            self.telemetry.data.command.position = target_position
            self.telemetry.data.command.velocity = abs(profile_velocity)
            self.position_rpdo['Target_position'].raw = self.telemetry.data.command.position
            self.position_rpdo['Profile_velocity'].raw = self.telemetry.data.command.velocity
            self.position_rpdo.transmit()
            self.common.delay_PDO()
            self.DEBUG_FEEDBACK and self.log.print(f"Set Target_position = {self.telemetry.data.command.position} count, Profile_velocity = {self.telemetry.data.command.velocity}", "RPDO1", "Target_position-Profile_velocity")
            return self.telemetry.data.command.position, self.telemetry.data.command.velocity
        except Exception as e:
            self.DEBUG_FEEDBACK and self.log.print(f"Position command error", "❌", "run", f"{e}")
            return None
        
    def RUN(self, target_position: int, profile_velocity: int = 0): 
        try:
            if not self.position_rpdo:
                raise RuntimeError(f"position_rpdo 'Target_position' not initialized.")
            
            self.telemetry.data.command.position = target_position
            self.telemetry.data.command.velocity = abs(profile_velocity)
            self.node.sdo['Target_position'].raw = self.telemetry.data.command.position
            self.node.sdo['Profile_velocity'].raw = self.telemetry.data.command.velocity
            self.common.delay_SDO()
            self.DEBUG_FEEDBACK and self.log.print(f"Set Target_position = {self.telemetry.data.command.position} count, Profile_velocity = {self.telemetry.data.command.velocity}", "RPDO1", "Target_position-Profile_velocity")
            return self.telemetry.data.command.position, self.telemetry.data.command.velocity
        except Exception as e:
            self.DEBUG_FEEDBACK and self.log.print(f"Position command error", "❌", "run", f"{e}")
            return None

