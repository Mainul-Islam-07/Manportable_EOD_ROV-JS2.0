# rpdo3_controlword_sender.py
import can
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Logger_Lib import Logger
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Common_Lib import Common
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Mapping.mapping import Avatarrobot_CANopen_Map


class RPDO3ControlwordSender:
    """
    Minimal fire-and-forget RPDO3 Controlword sender using `cansend`.

    - RPDO3 COB-ID defaults to (0x400 + node_id) per CiA-301.
    - Pass `rpdo3_cobid` to target a shared/group COB-ID (all nodes listen on the same ID).
    - Always sends 2 bytes (little-endian) because RPDO3 is mapped to 0x6040:00 (16 bits).
    """

    def __init__(self,
                 node_name: str,
                 node_id: int,
                 iface: any):
        try:
            self.Node_ID = node_id
            self.Node_Name = node_name

            self.common = Common(self.Node_ID, self.Node_Name)
            self.log = Logger(self.Node_Name, self.Node_ID)
            self.controlword_id = 0x400 + self.Node_ID  # Default RPDO3 COB-ID
            self.bus = iface

        except Exception as e:
            self.log.print(f"Universal Controlword Setup Failure","❌","RPDO3ControlwordSender","__init__",f"{e}")

    
    def send_controlword(self, controlword: int) -> int:
        try:
            if self.bus is None:
                self.log.print("CAN bus is not initialized", "CANopen", "SEND", "❌")
                return -1
            msg = can.Message(
            arbitration_id=self.controlword_id,
            data=(controlword & 0xFFFF).to_bytes(2, 'little'),
            is_extended_id=False
            )
            self.bus.send(msg)
            self.common.delay_PDO()
            self.log.print(f"Sent controlword {controlword:04x} in {self.bus.channel_info} to all node", "CANopen", "SEND", "UNIVERSAL")
            return controlword
        except Exception as e:
            self.log.print(f"Failed to send controlword {controlword:04x}", "❌", "UNIVERSAL", f"{e}")
            return -1
        
    def arm(self):
        try:
            self.send_controlword(Avatarrobot_CANopen_Map.ControlWord.SHUT_DOWN)
            self.send_controlword(Avatarrobot_CANopen_Map.ControlWord.SWITCH_ON)
            self.send_controlword(Avatarrobot_CANopen_Map.ControlWord.ENABLE_VOLTAGE)
            self.log.print(f"Sent ARM sequence to all nodes", "CANopen", "ARM", "UNIVERSAL")

        except Exception as e:
            self.log.print(f"Failed to send ARM sequence", "❌", "ARM", f"{e}")

    def disarm(self):
        try:
            self.send_controlword(Avatarrobot_CANopen_Map.ControlWord.SHUT_DOWN)
            self.send_controlword(Avatarrobot_CANopen_Map.ControlWord.RESET_FAULT)
            self.send_controlword(Avatarrobot_CANopen_Map.ControlWord.DISABLE_VOLTAGE)
            self.log.print(f"Sent DISARM command to all nodes", "CANopen", "DISARM", "UNIVERSAL")

        except Exception as e:
            self.log.print(f"Failed to send DISARM sequence", "❌", "DISARM", f"{e}")