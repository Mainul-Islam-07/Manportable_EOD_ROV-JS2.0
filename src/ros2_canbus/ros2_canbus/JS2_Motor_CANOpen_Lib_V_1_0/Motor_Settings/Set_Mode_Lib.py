import canopen
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Logger_Lib import Logger
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Common_Lib import Common
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.CANopen_Network.Network_Lib import CANopen_Network
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Settings.Load_Settings_Lib import Load_Settings
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.Motor_Telemetry_Lib import Motor_Telemetry
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.CST_Lib import Torque_Setup
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.CSV_Lib import Velocity_Setup
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.CSP_Lib import Position_Setup
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Mapping.mapping import Avatarrobot_CANopen_Map


class Mode():
    def __init__(
            self, node: canopen.RemoteNode,
            node_name: str,
            canopen_handle: CANopen_Network,
            settings: Load_Settings,
            telemetry: Motor_Telemetry,
            mode: int):
        try:
            self.node = node
            self.Node_ID = self.node.id
            self.Node_Name = node_name
            self.canopen_handle = canopen_handle
            self.settings = settings
            self.failure_exit = self.settings.FAILURE_EXIT
            self.telemetry = telemetry
            self.common = Common(self.Node_ID, self.Node_Name)
            self.log = Logger(self.Node_Name, self.Node_ID)
            self.DEBUG_INIT = self.settings.DEBUG_INIT and True
            self.DEBUG_INIT and self.log.print(f"Starting CiA402 startup protocol...","INFO", "Mode_operation_Configuration->Mode")
            self.telemetry.data.settings.operationmode.write = mode
            self.temp_mode = Avatarrobot_CANopen_Map.ModesOfOperation
            self.setup = self.set_mode_of_operation(mode)
            self.telemetry.data.settings.operationmode.read = self.setup
            self.select_operational_settings_success = self.select_operational_settings(self.telemetry.data.settings.operationmode.read)
            self.log.print(f"Set Mode of Operation with settings: {self.select_operational_settings_success}", "MODE", "SETUP")


        except Exception as e:
            self.log.print(f"Mode Setup Failure","❌","Mode_Lib","__init__",f"{e}")

    def set_mode_of_operation(self, mode: Avatarrobot_CANopen_Map.ModesOfOperation):
        try:
            if not self.node:
                raise RuntimeError(f"Node not initialized.")
            self.node.sdo['Modes_of_operation'].raw = mode
            self.common.delay_SDO()
            mode_check = self.node.sdo['Modes_of_operation_display'].raw
            self.common.delay_SDO()
            if mode_check != mode:
                self.DEBUG_INIT and self.log.print(f"Set mode = {mode} : {mode_check} (Mismatch)","SDO", "MODE")
                if self.failure_exit:
                    self.log.print(f"Exiting due to mode set failure as FAILURE_EXIT is set to True.","❌","Mode_Lib","set_mode_of_operation",f"Exiting program.")
                    exit(1)
            else:
                self.DEBUG_INIT and self.log.print(f"Set mode = {mode} : {mode}","SDO", "MODE")
                return mode_check
            
        except Exception as e:
            self.DEBUG_INIT and self.log.print(f"Set mode error","❌", "set_mode","set_mode_of_operation", f"{e}")

    def get_mode_of_operation(self):
        try:
            if not self.node:
                raise RuntimeError(f"Node not initialized.")
            mode = self.node.sdo['Modes_of_operation_display'].raw
            self.common.delay_SDO()
            self.DEBUG_INIT and self.log.print(f"Got mode={mode}", "SDO", "MODE")
            return mode
        except Exception as e:
            self.DEBUG_INIT and self.log.print(f"Getting mode error", "❌", "set_mode", f"{e}")
            return None
        

    def select_operational_settings(self, mode: Avatarrobot_CANopen_Map.ModesOfOperation):
        try:
            if not self.node:
                raise RuntimeError(f"Node not initialized.")
            success = False
            match mode:

                case self.temp_mode.CYCLIC_SYNCHRONOUS_TORQUE_MODE:
                    self.torque = Torque_Setup(self.node, self.Node_Name, self.canopen_handle, self.settings, self.telemetry)
                    success = self.torque.set_torque_parameter_success
                    return success
                case self.temp_mode.CYCLIC_SYNCHRONOUS_VELOCITY_MODE:
                    self.velocity = Velocity_Setup(self.node, self.Node_Name, self.canopen_handle, self.settings, self.telemetry)
                    success = self.velocity.set_velocity_parameter_success
                    return success
                
                case self.temp_mode.CYCLIC_SYNCHRONOUS_POSITION_MODE:
                    self.position = Position_Setup(self.node, self.Node_Name, self.canopen_handle, self.settings, self.telemetry)
                    success = self.position.set_position_parameter_success
                    return success
                
                case _:
                    self.log.print(f"Unsupported mode selection: {mode}. Only valid modes are CYCLIC_SYNCHRONOUS_TORQUE_MODE, CYCLIC_SYNCHRONOUS_VELOCITY_MODE, CYCLIC_SYNCHRONOUS_POSITION_MODE")
                    return success
        
        except Exception as e:
            self.DEBUG_INIT and self.log.print(f"Selecting operational settings error", "❌", "select_operational_settings", f"{e}")
            return success