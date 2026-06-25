import canopen
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Logger_Lib import Logger
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Common_Lib import Common
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.CANopen_Network.Network_Lib import CANopen_Network
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Settings.Load_Settings_Lib import Load_Settings
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.Motor_PDO_Lib import PDO_Lib
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.Motor_Telemetry_Lib import Motor_Telemetry
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.Motor_PDO_Lib import PDO_Lib

class Velocity_Setup():
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
            self.log.print(f"Starting Velocity Profile setup...","INFO", "Velocity_Setup","__init__", "CYCLIC_SYNCHRONOUS_VELOCITY_MODE")
            self.set_velocity_parameter_success = self.set_velocity_parameter(self.settings.BLDC_max_velocity, 
                                                                              self.settings.max_velocity, 
                                                                              self.settings.current_acceleration, 
                                                                              self.settings.current_deceleration)
            self.pdo = PDO_Lib(self.node, self.Node_Name)
            for i in range(1,6):
                self.log.print(f"PDO Configuration attempt {i}...", "INFO", "Velocity_Setup", "__init__", "PDO")
                is_pdo_configured = self.configure()
                self.common.delay_SDO()
                if is_pdo_configured is not None:
                    break
            # self.configure()

        except Exception as e:
            self.log.print(f"Setup Failure","❌","Velocity_Setup","__init__",f"{e}")



    def set_velocity_parameter(self, max_velocity, profile_velocity, acceleration, deceleration):
        try:
            
            if not self.node:
                raise RuntimeError(f"Node not initialized.")
            if max_velocity is None or profile_velocity is None or acceleration is None or deceleration is None:
                raise ValueError(f"Failed to acquire velocity profile parameters for not being a number")
            
            self.node.sdo['Max_profile_velocity'].raw = max_velocity
            self.common.delay_SDO()
            self.node.sdo['Profile_velocity'].raw = profile_velocity
            self.common.delay_SDO()
            self.node.sdo['Profile_acceleration'].raw = acceleration
            self.common.delay_SDO()
            self.node.sdo['Profile_deceleration'].raw = deceleration
            self.common.delay_SDO()

            if (self.node.sdo['Max_profile_velocity'].raw != max_velocity     or
                self.node.sdo['Profile_velocity'    ].raw != profile_velocity or
                self.node.sdo['Profile_acceleration'].raw != acceleration     or
                self.node.sdo['Profile_deceleration'].raw != deceleration):
                return False
            else:
                self.DEBUG_INIT and self.log.print(f"Set "
                                            f"max_vel={max_velocity} crps, "
                                            f"max_profile_vel={profile_velocity} crps, "
                                            f"accel={acceleration} crpspm, "
                                            f"decel={deceleration} crpspm", 
                                            "SDO", "PROFILE")
                return True
                
        except Exception as e:
            self.DEBUG_INIT and self.log.print(f"Velocity Profile error", "❌","Velocity_Setup", "set_velocity_parameter", f"{e}")
            return False

    def configure(self):
        self.velocity_rpdo = self.pdo.configure_pdo_attempt_multiple(
            node_id=self.node.id,
            pdo_name="RPDO1",
            variables=["Target_velocity"],
            trans_type=255,
            event_timer=0,
            enabled=True
        )
        return self.velocity_rpdo

    def run(self, target_velocity: int = 0):
        try:
            if not self.velocity_rpdo:
                raise RuntimeError(f"velocity_rpdo 'Target_velocity' not initialized.")
            
            self.telemetry.data.command.velocity = target_velocity
            self.velocity_rpdo['Target_velocity'].raw = self.telemetry.data.command.velocity
            self.velocity_rpdo.transmit()
            self.common.delay_PDO()
            self.DEBUG_FEEDBACK and self.log.print(f"Set Target_velocity = {self.telemetry.data.command.velocity} crps", "RPDO1", "Target_velocity")
            return self.telemetry.data.command.velocity
        except Exception as e:
            self.DEBUG_FEEDBACK and self.log.print(f"Velocity command error", "❌", "run", f"{e}")
            return None


    def RUN(self, target_velocity: int = 0):
        try:
            if not self.node:
                raise RuntimeError(f"velocity_object 'Target_velocity' not initialized.")
            
            self.telemetry.data.command.velocity = target_velocity
            self.node.sdo['Target_velocity'].raw = self.telemetry.data.command.velocity
            self.common.delay_SDO()
            self.DEBUG_FEEDBACK and self.log.print(f"Set Target_velocity = {self.telemetry.data.command.velocity} crps", "SDO", "Target_velocity")
            return self.telemetry.data.command.velocity
        except Exception as e:
            self.DEBUG_FEEDBACK and self.log.print(f"Velocity command error", "❌", "run", f"{e}")
            return None