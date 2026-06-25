# **User callback**
    This is user callback for heartbeat. If you need to take actions per heartbeat reception, you modify the following code
## Heartbeat Callback
    # Go to "JS2_Motor_CANOpen_Lib_V_1_0/User_Callback/Heartbeat_Callback_Lib.py"
### Add your intended actions to be executed in the event of heartbeat reception/timeout interrupt
    def heartbeat_timeout_callback(self):
        try:
            self.DEBUG_HEARTBEAT and self.log.print(f"Heartbeat timeout detected for Node {self.Node_Name} (ID: {self.Node_ID})", "💔", "Callback_Lib", "heartbeat_timeout_callback")
            # You can add additional handling here, such as setting flags, attempting recovery, etc, or turning off motor
        except Exception as e:
            self.DEBUG_HEARTBEAT and self.log.print(f"Heartbeat timeout callback error", "❌", "Callback_Lib","heartbeat_timeout_callback", f"{e}")
### Add your intended actions to be handled at heartbeat reception
    # Preprocessing
    def heartbeat_callback_preprocessing(self, heartbeat: heartbeat):
        """User-defined preprocessing before main heartbeat handling."""
        # print(f"heartbeat_callback_preprocessing placeholder")

        pass

    # Postprocessing
    def heartbeat_callback_postprocessing(self, heartbeat: heartbeat):
        """User-defined postprocessing after main heartbeat handling."""
        # print(f"heartbeat_callback_postprocessing placeholder")

        pass
## TPDO Callback
    # Go to "JS2_Motor_CANOpen_Lib_V_1_0/User_Callback/Tpdo_Callback_Lib.py"
### **Add your intended actions to be executed in the event of a TPDO reception interrupt**
### TPDO1: Position, Velocity
    # Preprocessing
    def TPDO_1_Position_actual_value_Velocity_actual_value_Object_callback_preprocessing(self, 
                                                                        Position_actual_value,
                                                                        Velocity_actual_value):
        """User-defined preprocessing before main TPDO1 handling."""

        pass
    # Postprocessing
    def TPDO_1_Position_actual_value_Velocity_actual_value_Object_callback_postprocessing(self, 
                                                                        Position_actual_value,
                                                                        Velocity_actual_value):
        """User-defined postprocessing after main TPDO1 handling."""
        # print(f"TPDO_1_Position_actual_value_Velocity_actual_value_Object_callback_preprocessing placeholder")
        pass
### TPDO2: Voltage, Current, Coil temperature, Circuit Temperature
    # Preprocessing
    def TPDO_2_Motor_bus_voltage_Current_actual_value_Coil_temperature_value_Circuit_board_temperature_value_Object_callback_preprocessing(self, 
                                                                        Motor_bus_voltage,
                                                                        Current_actual_value,
                                                                        Coil_temperature_value,
                                                                        Circuit_board_temperature_value):
        """User-defined preprocessing before main TPDO2 handling."""
        pass

    # Postprocessing
    def TPDO_2_Motor_bus_voltage_Current_actual_value_Coil_temperature_value_Circuit_board_temperature_value_Object_callback_postprocessing(self, 
                                                                        Motor_bus_voltage,
                                                                        Current_actual_value,
                                                                        Coil_temperature_value,
                                                                        Circuit_board_temperature_value):
        """User-defined postprocessing after main TPDO2 handling."""
        # print(f"TPDO_2_Motor_bus_voltage_Current_actual_value_Coil_temperature_value_Circuit_board_temperature_value_Object_callback_postprocessing placeholder")
        pass
### TPDO3: Torque_actual_value, Statusword, Error_code, Error_register, Modes_of_operation_display
    # Preprocessing
    def TPDO_3_Torque_actual_value_Statusword_Error_code_Error_register_Modes_of_operation_display_Object_callback_preprocessing(self, 
                                                                        Torque_actual_value,
                                                                        Statusword,
                                                                        Error_code,
                                                                        Error_register,
                                                                        Modes_of_operation_display):
        """User-defined preprocessing before main TPDO3 handling."""
        pass
    # Postprocessing
    def TPDO_3_Torque_actual_value_Statusword_Error_code_Error_register_Modes_of_operation_display_Object_callback_postprocessing(self, 
                                                                        Torque_actual_value,
                                                                        Statusword, Status_Bucket,
                                                                        Error_code, Error_code_parsed,
                                                                        Error_register, Error_register_parsed,
                                                                        Modes_of_operation_display):
        """User-defined postprocessing after main TPDO3 handling."""
        # print(f"TPDO_3_Torque_actual_value_Statusword_Error_code_Error_register_Modes_of_operation_display_Object_callback_postprocessing placeholder")
        pass