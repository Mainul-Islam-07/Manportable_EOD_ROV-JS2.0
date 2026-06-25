# CANopen_Network_Setup.py
import os, json, canopen, json, re
from typing import Optional

from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Logger_Lib import Logger
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Common_Lib import Common
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.CANopen_Network.Universal_Controlword import RPDO3ControlwordSender
# from UDP_Network.UDP_Lib import StatelessUDP

class CANopen_Network:
    def __init__(self, can_json_file: str, Node_Name: str = "Master", Node_ID: int = 1):
        try:
            self.can_json_file = can_json_file
            self.Node_Name = Node_Name
            self.Node_ID = Node_ID
            self.log = Logger(self.Node_Name, self.Node_ID)
            self.common = Common(self.Node_Name, self.Node_ID)
            self.CAN_network_settings_json_file = self.common.file_navigator("/home/jontro_soinik_2_0-2/ros2_ws/src/ros2_canbus/ros2_canbus/JS2_Motor_CANOpen_Lib_V_1_0/Network", can_json_file)
            self.settings = self.load_settings_from_json(self.CAN_network_settings_json_file)
            self.setup_network()
            self.control = RPDO3ControlwordSender(self.Node_Name, self.Node_ID, self.bus)

            # self.network.sync.start(0.1)  # 10 ms sync interval

            


        except Exception as e:
            self.log.print(f"network Setup Failure","❌","CANopen_Network_Setup","__init__",f"{e}")

    def load_settings_from_json(self, file_absolute_path: str):
        """Load settings from the CAN JSON file."""
        try:
            can_settings = None
            with open(file_absolute_path, 'r') as f:
                can_settings = json.load(f)
                for key, value in can_settings.items():
                    self.log.print(f"{key}: {value}")
            return can_settings
        except Exception as e:
            self.log.print(f"network Setup Failure","❌","CANopen_Network_Setup","load_settings_from_json",f"{e}")
    
    def setup_network(self):
        try:
            if self.settings is None:
                raise ValueError("Settings not loaded properly")
            network_settings = self.settings['network']
            self.network = canopen.Network()
            self.network.connect(
                interface=network_settings['interface'],
                channel=network_settings['channel'],
                bitrate=network_settings['bitrate']
            )
            self.bus = self.network.bus
            if self.network is not None:
                self.log.print(f"Connected to CANopen network on {network_settings['interface']} with channel {network_settings['channel']} and bitrate {network_settings['bitrate']}",
                "NETWORK", "CONNECT", "")
            else:
                raise ValueError(f"failed to setup {self.can_json_file}")
        except Exception as e:
            self.log.print(f"network Setup Failure","❌","CANopen_Network_Setup","setup_network",f"{e}")

    def network_operational(self):
        try:
            if self.network is None:
                raise RuntimeError("CANopen network not initialized")
            self.network.nmt.state = 'OPERATIONAL'
            self.log.print(f"Network is now OPERATIONAL", "NETWORK", "OPERATIONAL", "")
        except Exception as e:
            self.log.print(f"Failed to set network to OPERATIONAL","❌","CANopen_Network_Setup","network_go_live",f"{e}")

    def network_preoperational(self):
        try:
            if self.network is None:
                raise RuntimeError("CANopen network not initialized")
            self.network.nmt.state = 'PRE-OPERATIONAL'
            self.log.print(f"Network is now PRE-OPERATIONAL", "NETWORK", "PRE-OPERATIONAL", "")
        except Exception as e:
            self.log.print(f"Failed to set network to OPERATIONAL","❌","CANopen_Network_Setup","network_go_live",f"{e}")

    def network_reset(self):
        try:
            if self.network is None:
                raise RuntimeError("CANopen network not initialized")
            self.network.nmt.state = 'RESET'
            self.log.print(f"Network is now RESET", "NETWORK", "RESET", "")
        except Exception as e:
            self.log.print(f"Failed to reset network","❌","CANopen_Network_Setup","network_reset",f"{e}")

    def get_canopen_id(self, node_id: Optional[int], message_type: str) -> int:
        """
        Return the 11-bit CANopen arbitration ID for a given message type.
        node_id: 1..127 for node-scoped types; ignored for 'nmt', 'sync', 'time'.
        message_type (case/space/underscore insensitive):
            - 'tpdo1'..'tpdo4', 'rpdo1'..'rpdo4'
            - 'sdo_req' / 'sdoreq'  (0x600 + node)
            - 'sdo_res' / 'sdoresp' (0x580 + node)
            - 'emcy'       (0x080 + node)
            - 'heartbeat' / 'nmtstate' / 'bootup' (0x700 + node)
            - 'sync' (0x080), 'time' (0x100), 'nmt' (0x000)
        """
        s = re.sub(r"[\s_-]+", "", str(message_type).lower())

        # IDs that do NOT include node_id in the arbitration ID
        fixed_no_node = {
            "nmt":  0x000,
            "sync": 0x080,
            "time": 0x100,
        }
        if s in fixed_no_node:
            return fixed_no_node[s]

        # Validate node_id for everything else
        if not isinstance(node_id, int) or not (1 <= node_id <= 127):
            raise ValueError("node_id must be an int in 1..127 for this message type")

        # SDO / EMCY / Heartbeat families
        fixed_with_node = {
            "sdoreq":   0x600,
            "sdotx":    0x600,
            "sdotonode":0x600,
            "sdores":   0x580,
            "sdorx":    0x580,
            "sdofromnode": 0x580,
            "emcy":     0x080,
            "heartbeat":0x700,
            "nmtstate": 0x700,
            "bootup":   0x700,  # same ID as heartbeat; payload distinguishes
        }
        if s in fixed_with_node:
            return fixed_with_node[s] + node_id

        # PDOs
        m = re.fullmatch(r"([tr])pdo([1-4])", s)
        if m:
            n = int(m.group(2))
            if m.group(1) == "t":  # TPDO
                bases = {1: 0x180, 2: 0x280, 3: 0x380, 4: 0x480}
            else:                   # RPDO
                bases = {1: 0x200, 2: 0x300, 3: 0x400, 4: 0x500}
            return bases[n] + node_id

        raise ValueError(f"Unknown message type: {message_type!r}")


    def recovery(self):
        try:
            self.control.disarm()
            self.log.print("Recovary Disarm triggered", "CANopen", "DISARM", "UNIVERSAL")
        except Exception as e:
            self.log.print("Disarm failure - Bus Offline","Error","recovery", "CANopen_Network", f"{e}")


    def disconnect(self):
        """Gracefully disconnect from the CAN network."""
        if self.network:
            self.control.disarm()
            self.network.nmt.state = 'PRE-OPERATIONAL'
            self.network.disconnect()
            self.log.print(f"Disconnected {self.can_json_file} from CANopen network", "NETWORK", "DISCONNECT", "")

