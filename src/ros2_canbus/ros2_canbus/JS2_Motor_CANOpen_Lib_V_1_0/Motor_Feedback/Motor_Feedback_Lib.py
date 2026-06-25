import canopen
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Logger_Lib import Logger
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Common_Lib import Common
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.CANopen_Network.Network_Lib import CANopen_Network
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Settings.Load_Settings_Lib import Load_Settings
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.Motor_PDO_Lib import PDO_Lib
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.Motor_Telemetry_Lib import Motor_Telemetry
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.User_Callback.Tpdo_Callback_Lib import Tpdo_Callback
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.Motor_Telemetry_Lib import Motor_Telemetry

class Feedback_Lib:
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
            self.FAILURE_EXIT = self.settings.FAILURE_EXIT
            self.telemetry = telemetry
            self.common = Common(self.Node_ID, self.Node_Name)
            self.log = Logger(self.Node_Name, self.Node_ID)
            self.pdo = PDO_Lib(
                                self.node, 
                                self.Node_Name)
            
            self.callback = Tpdo_Callback(
                                        self.node,
                                        self.Node_Name,
                                        self.canopen_handle,
                                        self.telemetry,
                                        self.settings)

            self.trans_type=254
            self.DEBUG_FEEDBACK = self.settings.DEBUG_FEEDBACK
            self.time_period_ms = {
                "tpdo1": 2*9,
                "tpdo2": 5*1000,
                "tpdo3": 2*11
            }
            
            self.tpdo_id = {
                "1": canopen_handle.get_canopen_id(self.Node_ID, "tpdo1"),
                "2": canopen_handle.get_canopen_id(self.Node_ID, "tpdo2"),
                "3": canopen_handle.get_canopen_id(self.Node_ID, "tpdo3")
            }
            self.canopen_handle.network.subscribe(self.tpdo_id["1"], self.TPDO_1_Position_actual_value_Velocity_actual_value_Object_interrupt)
            self.canopen_handle.network.subscribe(self.tpdo_id["2"], self.TPDO_2_Motor_bus_voltage_Current_actual_value_Coil_temperature_value_Circuit_board_temperature_value_Object_interrupt)
            self.canopen_handle.network.subscribe(self.tpdo_id["3"], self.TPDO_3_Torque_actual_value_Statusword_Error_code_Error_register_Modes_of_operation_display_Object_interrupt)
            
            self.configure()



        except Exception as e:
            self.log.print(f"Feedback Setup Failure","❌","Feedback_Lib","__init__",f"{e}")


    def configure(self):
        try:
            self.tpdo = {
                "Position_actual_value_Velocity_actual_value":    self.pdo.configure_pdo_attempt_multiple(
                                                                            node_id=self.node.id,
                                                                            pdo_name="TPDO1",
                                                                            variables=["Position_actual_value",
                                                                                    "Velocity_actual_value"],
                                                                            trans_type=self.trans_type,
                                                                            event_timer=self.time_period_ms["tpdo1"],
                                                                            enabled=True
                                                                        ),
                "Motor_bus_voltage_Current_actual_value_Coil_temperature_value_Circuit_board_temperature_value": self.pdo.configure_pdo_attempt_multiple(
                                                                            node_id=self.node.id,
                                                                            pdo_name="TPDO2",
                                                                            variables=["Motor_bus_voltage",
                                                                                    "Current_actual_value",
                                                                                    "Coil_temperature_value",
                                                                                    "Circuit_board_temperature_value"],
                                                                            trans_type=self.trans_type,
                                                                            event_timer=self.time_period_ms["tpdo2"],
                                                                            enabled=True
                                                                        ),   
                "Torque_actual_value_Statusword_Error_code_Error_register_Modes_of_operation_display": self.pdo.configure_pdo_attempt_multiple(
                                                                            node_id=self.node.id,
                                                                            pdo_name="TPDO3",
                                                                            variables=["Torque_actual_value",
                                                                                    "Statusword", "Error_code",
                                                                                    "Error_register",
                                                                                    "Modes_of_operation_display"],
                                                                            trans_type=self.trans_type,
                                                                            event_timer=self.time_period_ms["tpdo3"],
                                                                            enabled=True
                                                                        )                                                    

            }
        except Exception as e:
            self.log.print(f"Feedback PDO Configuration Failure","❌","Feedback_Lib","configure",f"{e}")
            if self.FAILURE_EXIT:
                self.log.print(f"Exiting due to configuration failure as FAILURE_EXIT is set to True.","❌","Feedback_Lib","configure",f"Exiting program.")
                exit(1)
            return self.tpdo


    def TPDO_1_Position_actual_value_Velocity_actual_value_Object_interrupt(self, cob_id, data, ts):
        try:
            self.callback.TPDO_1_Position_actual_value_Velocity_actual_value_Object_callback(
                self.tpdo["Position_actual_value_Velocity_actual_value"]['Position_actual_value'].raw,
                self.tpdo["Position_actual_value_Velocity_actual_value"]['Velocity_actual_value'].raw)

        except Exception as e:
            self.DEBUG_FEEDBACK and self.log.print(
                                                f"TPDO1 interrupt error",
                                                "❌", 
                                                "Feedback_Lib",
                                                f"{e}")
            

    def TPDO_2_Motor_bus_voltage_Current_actual_value_Coil_temperature_value_Circuit_board_temperature_value_Object_interrupt(self, cob_id, data, ts):
        try:
            self.callback.TPDO_2_Motor_bus_voltage_Current_actual_value_Coil_temperature_value_Circuit_board_temperature_value_Object_callback(
                self.tpdo["Motor_bus_voltage_Current_actual_value_Coil_temperature_value_Circuit_board_temperature_value"]['Motor_bus_voltage'].raw,
                self.tpdo["Motor_bus_voltage_Current_actual_value_Coil_temperature_value_Circuit_board_temperature_value"]['Current_actual_value'].raw,
                self.tpdo["Motor_bus_voltage_Current_actual_value_Coil_temperature_value_Circuit_board_temperature_value"]['Coil_temperature_value'].raw,
                self.tpdo["Motor_bus_voltage_Current_actual_value_Coil_temperature_value_Circuit_board_temperature_value"]['Circuit_board_temperature_value'].raw)
        except Exception as e:
            self.DEBUG_FEEDBACK and self.log.print(
                                                f"TPDO2 interrupt error",
                                                "❌", 
                                                "Feedback_Lib",
                                                f"{e}")

    def TPDO_3_Torque_actual_value_Statusword_Error_code_Error_register_Modes_of_operation_display_Object_interrupt(self, cob_id, data, ts):
        try:
            self.callback.TPDO_3_Torque_actual_value_Statusword_Error_code_Error_register_Modes_of_operation_display_Object_callback(
                self.tpdo["Torque_actual_value_Statusword_Error_code_Error_register_Modes_of_operation_display"]['Torque_actual_value'].raw,
                self.tpdo["Torque_actual_value_Statusword_Error_code_Error_register_Modes_of_operation_display"]['Statusword'].raw,
                self.tpdo["Torque_actual_value_Statusword_Error_code_Error_register_Modes_of_operation_display"]['Error_code'].raw,
                self.tpdo["Torque_actual_value_Statusword_Error_code_Error_register_Modes_of_operation_display"]['Error_register'].raw,
                self.tpdo["Torque_actual_value_Statusword_Error_code_Error_register_Modes_of_operation_display"]['Modes_of_operation_display'].raw)
        except Exception as e:
            self.DEBUG_FEEDBACK and self.log.print(
                                                f"TPDO3 interrupt error",
                                                "❌", 
                                                "Feedback_Lib",
                                                f"{e}")

