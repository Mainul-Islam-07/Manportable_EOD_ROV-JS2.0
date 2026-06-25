import canopen
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Logger_Lib import Logger
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Common_Lib import Common
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.CANopen_Network.Network_Lib import CANopen_Network
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.Motor_Telemetry_Lib import Motor_Telemetry
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Settings.Load_Settings_Lib import Load_Settings
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Mapping.data import heartbeat
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Feedback.Motor_Telemetry_Feedback_Parser import Parser

class Heartbeat_Callback:
    def __init__(
                self,
                node: canopen.RemoteNode,
                node_name: str,
                canopen_handle: CANopen_Network,
                telemetry: Motor_Telemetry,  
                settings: Load_Settings,
                ):
        try:
            self.node = node
            self.Node_ID = self.node.id
            self.Node_Name = node_name
            self.canopen_handle = canopen_handle
            self.common = Common(self.Node_ID, self.Node_Name)
            self.log = Logger(self.Node_Name, self.Node_ID)
            self.telemetry = telemetry
            self.settings = settings
            self.DEBUG_HEARTBEAT = self.settings.DEBUG_HEARTBEAT and True
            self.hbcount = 0
            self.parser = Parser(self.node, self.Node_Name)



            self.udp_payload_Heartbeat = None


        except Exception as e:
            self.log.print(f"Heartbeat Setup Failure","❌","Callback_Lib","__init__",f"{e}")


    def heartbeat_callback(self,data, hbcount):
        try:
            # self.telemetry.data.metadata.heartbeat.count = hbcount
            self.telemetry.data.metadata.heartbeat = self.parser.decode_heartbeat(data)
            self.heartbeat_callback_preprocessing(self.telemetry.data.metadata.heartbeat)

            self.DEBUG_HEARTBEAT and self.parser.print_heartbeat(self.telemetry.data.metadata.heartbeat)



            self.heartbeat_callback_postprocessing(self.telemetry.data.metadata.heartbeat)
        except Exception as e:
            self.DEBUG_HEARTBEAT and self.log.print(f"Callback error", "❌", "Callback","heartbeat_callback", f"{e}")

    def heartbeat_timeout_callback(self):
        try:
            self.DEBUG_HEARTBEAT and self.log.print(f"Heartbeat timeout detected for Node {self.Node_Name} (ID: {self.Node_ID})", "💔", "Callback_Lib", "heartbeat_timeout_callback")
            # You can add additional handling here, such as setting flags, attempting recovery, etc, or turning off motor
        except Exception as e:
            self.DEBUG_HEARTBEAT and self.log.print(f"Heartbeat timeout callback error", "❌", "Callback_Lib","heartbeat_timeout_callback", f"{e}")

    """User-defined heartbeat callback handling."""

    def heartbeat_callback_preprocessing(self, heartbeat: heartbeat):
        """User-defined preprocessing before main heartbeat handling."""
        # print(f"heartbeat_callback_preprocessing placeholder")

        pass

    def heartbeat_callback_postprocessing(self, heartbeat: heartbeat):
        """User-defined postprocessing after main heartbeat handling."""
        # print(f"heartbeat_callback_postprocessing placeholder")

        pass

