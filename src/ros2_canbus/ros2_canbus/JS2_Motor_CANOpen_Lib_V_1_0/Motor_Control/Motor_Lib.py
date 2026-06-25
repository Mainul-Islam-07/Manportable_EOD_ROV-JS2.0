import os, canopen, time
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.CANopen_Network.Network_Lib import CANopen_Network
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Logger_Lib import Logger
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Settings.Load_Settings_Lib import Load_Settings
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.Motor_Telemetry_Lib import Motor_Telemetry
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Feedback.Motor_Feedback_Lib import Feedback_Lib
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Common_Lib import Common
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Feedback.Motor_Heartbeat_Lib import Heartbeat_Lib
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Settings.Set_Mode_Lib import Mode
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.Controlword import Controlword_Setup

class Motor_CANopen_Lib:
    def __init__(self, 
                node_name: str,
                canopen_handle: CANopen_Network,
                settings_file: str):
        try:
            # print(f"🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧")
            # print(f"Initiated Loading settings for {node_name} from file: {settings_file}")
            # print(f"🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧🔧")
            
            self.Node_Name = node_name
            self.canopen_handle = canopen_handle
            self.settings_file = settings_file
            self.Node_ID = -1
            self.common = Common(self.Node_ID, self.Node_Name)
            self.log = Logger(self.Node_Name, self.Node_ID)
            if self.canopen_handle is None:
                raise RuntimeError("canopen_handle is invalid")

            else:

                self.settings = Load_Settings(settings_file, self.Node_Name)
                self.common = Common(self.Node_ID, self.Node_Name)
                self.log = Logger(self.Node_Name, self.settings.Node_ID)
                self.add_node_to_network(self.settings.Node_ID, self.settings.eds_file)
                # self.node.nmt.state = 'RESET'
                # self.node.nmt.state = 'PRE-OPERATIONAL'

                self.telemetry = Motor_Telemetry(self.node, self.Node_Name)
                # self.telemetry.get_print_all_telemetry()
                self.feedback = Feedback_Lib(self.node, self.Node_Name, self.canopen_handle, self.settings, self.telemetry)
                # self.heartbeat = Heartbeat_Lib(self.node, self.Node_Name, self.canopen_handle, self.settings, self.telemetry, 0)
                # print(f"Mode selected: {self.settings.mode}")
                self.mode = Mode(self.node, self.Node_Name, self.canopen_handle, self.settings, self.telemetry, self.settings.mode)
                self.control = Controlword_Setup(self.node, self.Node_Name, self.canopen_handle, self.settings, self.telemetry, self.mode)
                
                
                
                # self.node.nmt.state = 'OPERATIONAL'
                self.common.delay_SDO()
                #self.telemetry.get_print_all_telemetry()


                self.log.print(f"Completed Loading settings for {node_name}","✅", "Motor_Lib", "INIT")
                
        except Exception as e:
            
            self.log.print(f"Motor Setup Failure","❌","Motor_CANopen_Lib","__init__",f"{e}")
            self.log.print(f"❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌")

        

    def add_node_to_network(self, node_id, eds_file):
        try:
            
            self.motor_eds_file = self.common.file_navigator("Motor_Mapping", eds_file)
            self.node = canopen.RemoteNode(node_id, self.motor_eds_file)
            self.canopen_handle.network.add_node(self.node)
            # self.log.print(f"Added EDS file", "MOTOR", "ADD")
        except Exception as e:
            self.log.print(f"Failed to add EDS file: {e}", "❌", "MOTOR", "ADD")

    # def add_master_to_UDP(self, udp_file: str):
    #     try:
    #         self.udp_settings_json_file = self.common.file_navigator("Network", udp_file)
    #     except Exception as e:
    #         self.log.print(f"Failed to add UDP settings file: {e}", "❌", "MOTOR", "ADD")

