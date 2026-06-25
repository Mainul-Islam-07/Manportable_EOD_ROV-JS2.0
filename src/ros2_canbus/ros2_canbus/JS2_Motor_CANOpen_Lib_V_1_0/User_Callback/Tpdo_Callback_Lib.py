import canopen
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Logger_Lib import Logger
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Common_Lib import Common
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.CANopen_Network.Network_Lib import CANopen_Network
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.Motor_Telemetry_Lib import Motor_Telemetry
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Settings.Load_Settings_Lib import Load_Settings
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Feedback.Motor_Telemetry_Feedback_Parser import Parser

class Tpdo_Callback:
    def __init__(
                self,
                node: canopen.RemoteNode,
                node_name: str,
                canopen_handle: CANopen_Network,
                telemetry: Motor_Telemetry,  
                settings: Load_Settings):
        try:
            self.node = node
            self.Node_ID = self.node.id
            self.Node_Name = node_name
            self.canopen_handle = canopen_handle
            self.common = Common(self.Node_ID, self.Node_Name)
            self.log = Logger(self.Node_Name, self.Node_ID)
            self.telemetry = telemetry
            self.settings = settings
            self.parser = Parser(self.node, self.Node_Name)
            self.DEBUG_FEEDBACK = self.settings.DEBUG_FEEDBACK and True
            self.hbcount = 0

            self.udp_payload_tpdo1 = None
            self.udp_payload_TPDO2 = None
            self.udp_payload_TPDO3 = None

        except Exception as e:
            self.log.print(f"Tpdo_Callback Setup Failure","❌","Tpdo_Callback","__init__",f"{e}")

    """TPDO1 Callback handling"""
    """Position_actual_value"""
    """Velocity_actual_value"""
    def TPDO_1_Position_actual_value_Velocity_actual_value_Object_callback(
                                                                        self, 
                                                                        Position_actual_value,
                                                                        Velocity_actual_value):
        try:
            self.TPDO_1_Position_actual_value_Velocity_actual_value_Object_callback_preprocessing(
                                                                        Position_actual_value,
                                                                        Velocity_actual_value)
            self.telemetry.data.feedback.position = Position_actual_value
            self.telemetry.data.feedback.velocity = Velocity_actual_value
            self.DEBUG_FEEDBACK and self.log.print(f"Position: {self.telemetry.data.feedback.position:,}, "
                                                   f"Velocity: {self.telemetry.data.feedback.velocity:,}",
                                                   "TPDO1")

            self.TPDO_1_Position_actual_value_Velocity_actual_value_Object_callback_postprocessing(
                                                                        self.telemetry.data.feedback.position,
                                                                        self.telemetry.data.feedback.velocity)
        except Exception as e:
            self.log.print(f"❌","Tpdo1_Callback", f"{e}")

    """User-defined TPDO1 callback handling."""
    def TPDO_1_Position_actual_value_Velocity_actual_value_Object_callback_preprocessing(self, 
                                                                        Position_actual_value,
                                                                        Velocity_actual_value):
        """User-defined preprocessing before main TPDO1 handling."""

        pass

    def TPDO_1_Position_actual_value_Velocity_actual_value_Object_callback_postprocessing(self, 
                                                                        Position_actual_value,
                                                                        Velocity_actual_value):
        """User-defined postprocessing after main TPDO1 handling."""
        # print(f"TPDO_1_Position_actual_value_Velocity_actual_value_Object_callback_preprocessing placeholder")
        pass

    """TPDO2 Callback handling"""
    """Motor_bus_voltage"""
    """Current_actual_value"""
    """Coil_temperature_value"""
    """Circuit_board_temperature_value"""
    def TPDO_2_Motor_bus_voltage_Current_actual_value_Coil_temperature_value_Circuit_board_temperature_value_Object_callback(
                                                                        self, 
                                                                        Motor_bus_voltage,
                                                                        Current_actual_value,
                                                                        Coil_temperature_value,
                                                                        Circuit_board_temperature_value):
        try:
            self.TPDO_2_Motor_bus_voltage_Current_actual_value_Coil_temperature_value_Circuit_board_temperature_value_Object_callback_preprocessing(
                                                                        Motor_bus_voltage,
                                                                        Current_actual_value,
                                                                        Coil_temperature_value,
                                                                        Circuit_board_temperature_value)
            self.telemetry.data.metadata.voltage = Motor_bus_voltage
            self.telemetry.data.metadata.current = Current_actual_value
            self.telemetry.data.metadata.coiltemperature = Coil_temperature_value
            self.telemetry.data.metadata.circuittemperature = Circuit_board_temperature_value
            self.DEBUG_FEEDBACK and self.log.print(
                                                   f"Bus Voltage: {self.telemetry.data.metadata.voltage:,}, "
                                                   f"Current: {self.telemetry.data.metadata.current:,}, "
                                                   f"Coil Temp: {self.telemetry.data.metadata.coiltemperature:,}, "
                                                   f"Board Temp: {self.telemetry.data.metadata.circuittemperature:,}",
                                                   "TPDO2")

            self.TPDO_2_Motor_bus_voltage_Current_actual_value_Coil_temperature_value_Circuit_board_temperature_value_Object_callback_postprocessing(
                                                                        self.telemetry.data.metadata.voltage,
                                                                        self.telemetry.data.metadata.current,
                                                                        self.telemetry.data.metadata.coiltemperature,
                                                                        self.telemetry.data.metadata.circuittemperature)
        except Exception as e:
            self.log.print(f"❌","Tpdo2_Callback", f"{e}")
        
    """User-defined TPDO2 callback handling."""
    def TPDO_2_Motor_bus_voltage_Current_actual_value_Coil_temperature_value_Circuit_board_temperature_value_Object_callback_preprocessing(self, 
                                                                        Motor_bus_voltage,
                                                                        Current_actual_value,
                                                                        Coil_temperature_value,
                                                                        Circuit_board_temperature_value):
        """User-defined preprocessing before main TPDO2 handling."""
        pass

    def TPDO_2_Motor_bus_voltage_Current_actual_value_Coil_temperature_value_Circuit_board_temperature_value_Object_callback_postprocessing(self, 
                                                                        Motor_bus_voltage,
                                                                        Current_actual_value,
                                                                        Coil_temperature_value,
                                                                        Circuit_board_temperature_value):
        """User-defined postprocessing after main TPDO2 handling."""
        # print(f"TPDO_2_Motor_bus_voltage_Current_actual_value_Coil_temperature_value_Circuit_board_temperature_value_Object_callback_postprocessing placeholder")
        pass

    """TPDO3 Callback handling"""
    """Torque_actual_value"""
    """Statusword"""
    """Error_code"""
    """Error_register"""
    """Modes_of_operation_display"""
    def TPDO_3_Torque_actual_value_Statusword_Error_code_Error_register_Modes_of_operation_display_Object_callback(
                                                                        self, 
                                                                        Torque_actual_value,
                                                                        Statusword,
                                                                        Error_code,
                                                                        Error_register,
                                                                        Modes_of_operation_display):
        try:
            self.TPDO_3_Torque_actual_value_Statusword_Error_code_Error_register_Modes_of_operation_display_Object_callback_preprocessing(
                                                                        Torque_actual_value,
                                                                        Statusword,
                                                                        Error_code,
                                                                        Error_register,
                                                                        Modes_of_operation_display)
            self.telemetry.data.feedback.torque = Torque_actual_value
            self.telemetry.data.metadata.statusword.raw = Statusword
            self.telemetry.data.metadata.errorcode.raw = Error_code
            self.telemetry.data.metadata.errorregister.raw = Error_register
            self.telemetry.data.settings.operationmode.read = Modes_of_operation_display
            self.DEBUG_FEEDBACK and self.log.print(
                                                   f"Torque: {self.telemetry.data.feedback.torque:,}, "
                                                   f"Statusword: 0x{self.telemetry.data.metadata.statusword.raw:04x}, "
                                                   f"Error Code: 0x{self.telemetry.data.metadata.errorcode.raw:04x}, "
                                                   f"Error Register: 0x{self.telemetry.data.metadata.errorregister.raw:02x}, "
                                                   f"Operation Mode: {self.telemetry.data.settings.operationmode.read}",
                                                   "TPDO3")

            self.telemetry.data.metadata.statusword = self.parser.parse_statusword_state(Statusword, Modes_of_operation_display)
            self.DEBUG_FEEDBACK and self.parser.print_statusword(self.telemetry.data.metadata.statusword)
            self.telemetry.data.metadata.errorregister , self.telemetry.data.metadata.errorcode = self.parser.parse_and_flood_error_register_error_code(Error_register, Error_code)
            self.DEBUG_FEEDBACK and self.parser.print_errorcode(self.telemetry.data.metadata.errorcode)
            self.DEBUG_FEEDBACK and self.parser.print_errorregister(self.telemetry.data.metadata.errorregister)

            self.TPDO_3_Torque_actual_value_Statusword_Error_code_Error_register_Modes_of_operation_display_Object_callback_postprocessing(
                                                                        self.telemetry.data.feedback.torque,
                                                                        self.telemetry.data.metadata.statusword.raw , self.telemetry.data.metadata.statusword,
                                                                        self.telemetry.data.metadata.errorcode.raw, self.telemetry.data.metadata.errorcode.parsed,
                                                                        self.telemetry.data.metadata.errorregister.raw, self.telemetry.data.metadata.errorregister.parsed ,
                                                                        self.telemetry.data.settings.operationmode.read)
        except Exception as e:
            self.log.print(f"❌","Tpdo3_Callback", f"{e}")

    """User-defined TPDO3 callback handling."""
    def TPDO_3_Torque_actual_value_Statusword_Error_code_Error_register_Modes_of_operation_display_Object_callback_preprocessing(self, 
                                                                        Torque_actual_value,
                                                                        Statusword,
                                                                        Error_code,
                                                                        Error_register,
                                                                        Modes_of_operation_display):
        """User-defined preprocessing before main TPDO3 handling."""
        pass
    def TPDO_3_Torque_actual_value_Statusword_Error_code_Error_register_Modes_of_operation_display_Object_callback_postprocessing(self, 
                                                                        Torque_actual_value,
                                                                        Statusword, Status_Bucket,
                                                                        Error_code, Error_code_parsed,
                                                                        Error_register, Error_register_parsed,
                                                                        Modes_of_operation_display):
        """User-defined postprocessing after main TPDO3 handling."""
        # print(f"TPDO_3_Torque_actual_value_Statusword_Error_code_Error_register_Modes_of_operation_display_Object_callback_postprocessing placeholder")
        pass

