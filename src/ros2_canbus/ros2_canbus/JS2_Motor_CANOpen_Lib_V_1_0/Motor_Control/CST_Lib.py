import canopen
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Logger_Lib import Logger
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Common_Lib import Common
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.CANopen_Network.Network_Lib import CANopen_Network
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Settings.Load_Settings_Lib import Load_Settings
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.Motor_PDO_Lib import PDO_Lib
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.Motor_Telemetry_Lib import Motor_Telemetry
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.Motor_PDO_Lib import PDO_Lib

class Torque_Setup():
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
            self.log.print(f"Starting Torque Profile setup...","INFO", "Torque_Setup","__init__", "CYCLIC_SYNCHRONOUS_TORQUE_MODE")
            self.set_torque_parameter_success = self.set_torque_parameter(self.settings.max_torque, self.settings.torque_current_ratio)
            self.set_velocity_parameter_success = self.set_velocity_parameter(self.settings.BLDC_max_velocity, 
                                                                    self.settings.max_velocity, 
                                                                    self.settings.current_acceleration, 
                                                                    self.settings.current_deceleration)
            self.pdo = PDO_Lib(self.node, self.Node_Name)
            # self.configure()
            for i in range(1,6):
                self.log.print(f"PDO Configuration attempt {i}...", "INFO", "Torque_Setup", "__init__", "PDO")
                is_pdo_configured = self.configure()
                self.common.delay_SDO()
                if is_pdo_configured is not None:
                    break


            self.target_torque_now = 0

        except Exception as e:
            self.log.print(f"Setup Failure","❌","Torque_Setup","__init__",f"{e}")



    def set_torque_parameter(self, max_torque, torque_slope):
        try:
            if not self.node:
                raise RuntimeError(f"Node not initialized.")
            if max_torque is None or torque_slope is None:
                raise ValueError(f"Failed to acquire max torque or torque slope for not being a number")
            self.node.sdo['Max_torque'].raw = max_torque
            self.common.delay_SDO()
            self.node.sdo['Torque_slope'].raw = torque_slope
            self.common.delay_SDO()

            if (self.node.sdo['Max_torque'].raw != max_torque     or
                self.node.sdo['Torque_slope'].raw != torque_slope):
                return False
            else:
                self.DEBUG_INIT and self.log.print(f"Set max_torque = {max_torque} ma (coil torque current), torque_slope = {torque_slope} ratio", "SDO", "PROFILE")
                return True
        except Exception as e:
            self.DEBUG_INIT and self.log.print(f"Torque Profile error", "❌","Torque_Setup", "set_torque_parameter", f"{e}")
            return False

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
            self.DEBUG_INIT and self.log.print(f"Velocity Profile error", "❌","Torque_Setup", "set_velocity_parameter", f"{e}")
            return False

    def configure(self):
        self.torque_rpdo = self.pdo.configure(
            node_id=self.node.id,
            pdo_name="RPDO1",
            variables=["Target_torque"],
            trans_type=255,
            event_timer=0,
            enabled=True
        )
        return self.torque_rpdo

    # def run(self, target_torque: int = 0):
    #     try:
    #         if not self.torque_rpdo:
    #             raise RuntimeError(f"torque_rpdo 'Target_torque' not initialized.")
        
    #         self.telemetry.data.command.torque = target_torque
    #         self.torque_rpdo['Target_torque'].raw = self.telemetry.data.command.torque
    #         self.torque_rpdo.transmit()
    #         self.common.delay_PDO()
    #         self.DEBUG_FEEDBACK and self.log.print(f"Set Target_torque = {self.telemetry.data.command.torque} torque_current(ma)", "RPDO1", "Target_torque")
    #         return self.telemetry.data.command.torque
    #     except Exception as e:
    #         self.DEBUG_FEEDBACK and self.log.print(f"Torque command error" "❌", "run", f"{e}")
    #         return None

    def run(self, target_torque: int = 0, target_velocity: int | None = None):
        try:
            if not self.torque_rpdo:
                raise RuntimeError(f"torque_rpdo 'Target_torque' not initialized.")
            
            # --- simple torque write (Single input)---
            if target_velocity is None or target_torque == 0:
                self.telemetry.data.command.torque = int(target_torque)
                self.torque_rpdo["Target_torque"].raw = self.telemetry.data.command.torque
                self.torque_rpdo.transmit()
                self.common.delay_PDO()
                if self.DEBUG_FEEDBACK:
                    self.log.print(
                        f"Set Target_torque = {self.telemetry.data.command.torque} torque_current(ma)",
                        "RPDO1", "Target_torque"
                    )
                return self.telemetry.data.command.torque
            
            # --- torque - velocity write ---
            # _target_torque = abs(target_torque)
            if target_torque > 0:
                dir = 1
            elif target_torque < 0:
                dir = -1
            else:
                dir = 0
            _target_velocity = dir*abs(target_velocity)
            


            
            current_velocity = self.telemetry.data.feedback.velocity

            if current_velocity < _target_velocity:
                self.target_torque_now+=1
                pass

            elif current_velocity > _target_velocity:
                self.target_torque_now-=1
                pass

            else:

                pass

            if abs(self.target_torque_now) > abs(target_torque):
                self.target_torque_now = dir*abs(target_torque)



            self.telemetry.data.command.torque = self.target_torque_now
            self.telemetry.data.command.velocity = target_velocity
            self.torque_rpdo['Target_torque'].raw = self.telemetry.data.command.torque
            self.torque_rpdo.transmit()
            self.common.delay_PDO()
            self.DEBUG_FEEDBACK and self.log.print(f"Set target_torque_now = {self.telemetry.data.command.torque} torque_current(ma) at goal of {target_torque} torque_current(ma) at {target_velocity} crps", "RPDO1", "Target_torque")
            return self.telemetry.data.command.torque
        except Exception as e:
            self.DEBUG_FEEDBACK and self.log.print(f"Torque command error" "❌", "run", f"{e}")
            return None
        

    def RUN(self, target_torque: int = 0):
        try:
            if not self.node:
                raise RuntimeError(f"velocity_object 'Target_velocity' not initialized.")
            
            self.telemetry.data.command.torque = target_torque
            self.node.sdo['Target_torque'].raw = self.telemetry.data.command.torque
            self.common.delay_SDO()
            self.DEBUG_FEEDBACK and self.log.print(f"Set Target_torque = {self.telemetry.data.command.torque} dN", "SDO", "Target_torque")
            return self.telemetry.data.command.torque
        except Exception as e:
            self.DEBUG_FEEDBACK and self.log.print(f"Torque command error", "❌", "run", f"{e}")
            return None