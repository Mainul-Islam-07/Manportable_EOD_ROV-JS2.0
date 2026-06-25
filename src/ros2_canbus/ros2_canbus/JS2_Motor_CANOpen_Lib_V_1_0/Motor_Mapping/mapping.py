from enum import IntEnum
from dataclasses import dataclass
from typing import Optional

class Avatarrobot_CANopen_Map():
    """Class containing all CANopen control structures, states, and descriptions for Avatarrobot."""

    # Function to generate descriptions dynamically
    @staticmethod
    def generate_descriptions(enum_class):
        return {key: key.name.replace("_", " ").title() for key in enum_class}

    # Modes of Operation
    class ModesOfOperation(IntEnum):
        IDLE_TIME = 0
        PROFILE_POSITION_MODE = 1
        VELOCITY_MODE = 2
        PROFILE_VELOCITY_MODE = 3
        PROFILE_TORQUE_MODE = 4
        POSITION_INTERPOLATION_MODE = 7
        CYCLIC_SYNCHRONOUS_POSITION_MODE = 8
        CYCLIC_SYNCHRONOUS_VELOCITY_MODE = 9
        CYCLIC_SYNCHRONOUS_TORQUE_MODE = 10
        CONTINUE_TO_HAVE = 11

    class ControlSequence(IntEnum):
        UNKNOWN = 0
        SHUT_DOWN = 1
        SWITCH_ON = 2
        DISABLE_VOLTAGE = 3
        QUICK_STOP = 4
        DISABLE_OPERATION = 5
        ENABLE_OPERATION = 6
        FAULT_RESET = 7

    #Control Words
    class ControlWord(IntEnum):
        SHUT_DOWN = 0x06
        SWITCH_ON = 0x07
        ENABLE_VOLTAGE = 0x0F
        DISABLE_VOLTAGE = 0x00
        QUICK_STOP = 0x02
        QUICK_STOP_ACTIVE = 0x0B
        RESET_FAULT = 0x80
        SET_ABSOLUTE_POSITION = 0x1F
        TRIGER_ABSOLUTE_POSITION = 0x3F
        LATCH_ABSOLUTE_POSITION_IMMEDIATELY = 0x2F
        SET_RELATIVE_POSITION = 0x5F
        TRIGGER_RELATIVE_POSITION = 0x7F

    # CiA402 State Machine States
    class StateMachineState(IntEnum):
        UNKNOWN = 0
        NOT_READY_TO_SWITCH_ON = 1
        SWITCH_ON_DISABLED = 2
        READY_TO_SWITCH_ON = 3
        SWITCHED_ON = 4
        OPERATION_ENABLED = 5
        QUICK_STOP_ACTIVE = 6
        FAULT_REACTION_ACTIVE = 7
        FAULT = 8
        
    # NMT States
    class NMTState(IntEnum):
        ENTER_OPERATIONAL = 0x01 #Start Remote Node (Enter Operational)
        ENTER_STOPPED = 0x02 #Stop Remote Node (Enter Stopped)
        ENTER_PRE_OPERATIONAL = 0x80 #Enter Pre-Operational State
        RESET_NODE = 0x81 #Reset Node
        RESET_COMMUNICATION = 0x82 #Reset Communication
    
    # Heartbeat states for the node
    class HeartbeatState(IntEnum):
        UNKNOWN = 0xFF
        BOOT_UP = 0x00
        STOPPED = 0x04
        OPERATIONAL = 0x05
        PRE_OPERATIONAL = 0x7F

    class NodeGuardState(IntEnum):
        INITIALIZING = 0x00
        DISCONNECTED = 0x01
        CONNECTING = 0x02
        PRE_OPERATIONAL_1 = 0x03
        STOPPED = 0x04
        OPERATIONAL = 0x05
        PRE_OPERATIONAL_2 = 0x7F



    # Process Data Object (PDO) Mappings
    class PDOMapping(IntEnum):
        TX_PDO1 = 0x180
        RX_PDO1 = 0x200
        TX_PDO2 = 0x280
        RX_PDO2 = 0x300
        TX_PDO3 = 0x380
        RX_PDO3 = 0x400
        TX_PDO4 = 0x480
        RX_PDO4 = 0x500

    class CommunicationObjects(IntEnum):
        NMT_ID = 0x000  # Node ID for NMT (Network Management)
        HEARTBEAT_ID = 0x700  # Heartbeat ID for the node
        SYNC_ID = 0x080  # Sync ID for synchronization messages
        EMCY_ID = 0x080  # Emergency message ID



        @classmethod
        def describe(cls, error_word: int) -> dict:
            """
            Parse a raw error word (0x603F) into a mapping of ErrorCode members to descriptions.
            """
            descriptions = {}
            for code in cls:
                if error_word & code:
                    descriptions[code] = code.name.replace("_", " ").title()
            return descriptions



    @classmethod
    def generate_all_descriptions(cls):
        """Automatically generate descriptions for all IntEnum subclasses."""
        descriptions = {}  # Temporary dictionary to hold new attributes
        
        for attribute_name, attribute in list(cls.__dict__.items()):  # Use list() to avoid runtime changes
            if isinstance(attribute, type) and issubclass(attribute, IntEnum):
                descriptions[f"{attribute_name}_DESCRIPTIONS"] = cls.generate_descriptions(attribute)

        # Apply changes after iteration to avoid modifying __dict__ while iterating
        for key, value in descriptions.items():
            setattr(cls, key, value)

# Run description generation at the end of the class definition
Avatarrobot_CANopen_Map.generate_all_descriptions()