# ------------------------------------------------------------------
#                          SPEX ROVER 2025
# ------------------------------------------------------------------
# file name     : CAN_constants.py
# purpose       : contains constants for communication between the teensy4.1 and rover.
#                   
# authors       : Tyler Halifax
# created on    : 12/4/2025 - Tyler
# last modified : 12/4/2025 - Tyler
# ------------------------------------------------------------------

from enum import IntEnum

class CHANNEL(IntEnum):
    JETSON        = 0
    MAIN_BODY     = 1
    SCIENCE_BOARD = 2
    ARM_BOARD     = 3

TOPIC_RANGES = {
    CHANNEL.MAIN_BODY: (0, 4),
    CHANNEL.ARM_BOARD: (10, 18),
    CHANNEL.SCIENCE_BOARD: (20, 30),
}

TOPICS = {
    0: {
        "id": 0,
        "name": "E_STOP",
        "buf": bytearray(8),
        "channel": CHANNEL.MAIN_BODY,
    },
    1: {
        "id": 1,
        "name": "TARGET_VELOCITY",
        "buf": bytearray(8),
        "channel": CHANNEL.MAIN_BODY,
    },
    2: {
        "id": 2,
        "name": "CURRENT_VELOCITY",
        "buf": bytearray(8),
        "channel": CHANNEL.MAIN_BODY,
    },
    3: {
        "id": 3,
        "name": "DRIVE_POWER",
        "buf": bytearray(8),
        "channel": CHANNEL.MAIN_BODY,
    },
    10: {
        "id": 10,
        "name": "ARM_E_STOP",
        "buf": bytearray(8),
        "channel": CHANNEL.ARM_BOARD,
    },
    11: {
        "id": 11,
        "name": "MOVE_BASE",
        "buf": bytearray(8),
        "channel": CHANNEL.ARM_BOARD,
    },
    12: {
        "id": 12,
        "name": "MOVE_SHOULDER",
        "buf": bytearray(8),
        "channel": CHANNEL.ARM_BOARD,
    },
    13: {
        "id": 13,
        "name": "MOVE_ELBOW",
        "buf": bytearray(8),
        "channel": CHANNEL.ARM_BOARD,
    },
    14: {
        "id": 14,
        "name": "BEND_WRIST",
        "buf": bytearray(8),
        "channel": CHANNEL.ARM_BOARD,
    },
    15: {
        "id": 15,
        "name": "TWIST_WRIST",
        "buf": bytearray(8),
        "channel": CHANNEL.ARM_BOARD,
    },
    16: {
        "id": 16,
        "name": "MOVE_CLAW",
        "buf": bytearray(8),
        "channel": CHANNEL.ARM_BOARD,
    },
    17: {
        "id": 17,
        "name": "MOVE_SOLINOID",
        "buf": bytearray(8),
        "channel": CHANNEL.ARM_BOARD
    }
}

# New topic name base for ROS topics

TEENSY_TOPIC_NAME = "TEENSY_CAN"

class Subsystems_Names:
    ALL     = "ALL"
    CHASSIS = "CHASSIS"
    ARM     = "ARM"
    SCIENCE = "SCIENCE"

from typing import Any

# Define wrapper class for data types
class DATA_TYPE:
    def __init__(self, num_bits: int, id: int) -> None:
        self.num_bits = num_bits
        self.id = id

# Define data types for can bus communication
class DATA_TYPES:
    # enum of data types in bits
    UINT_8          = DATA_TYPE(8,  0x00)
    UINT_16         = DATA_TYPE(16, 0x01)
    UINT_32         = DATA_TYPE(32, 0x02)

class Signal:
    """Class representing a signal with a name and type."""
    def __init__(self, type: DATA_TYPE, default_value: Any = 0):
        self.__type = type
        self.__default_value = default_value
        self.__value = self.__default_value
    @property
    def type(self) -> DATA_TYPE:
        """Get the type of the signal."""
        return self.__type
    @property
    def value(self) -> Any:
        """Get the current value of the signal."""
        return self.__value
    @property
    def default_value(self) -> Any:
        """Get the default value of the signal."""
        return self.__default_value

    @value.setter
    def value(self, new_value: Any) -> None:
        """Set the current value of the signal."""
        self.__value = new_value

    def toString(self) -> str:
        """Get the string representation of the signal."""
        return f"Signal(type={self.__type}, value={self.__value}, default_value={self.__default_value})"

class Message:
    """Class representing a message with an ID, name, and signals."""
    def __init__(self, id: int, name: str, subsystem: str, signals: dict[str, Signal]):
        self.__id = id
        self.__name = name
        self.__subsystem = subsystem
        self.__signals = signals

    @property
    def id(self) -> int:
        """Get the ID of the message."""
        return self.__id

    @property
    def name(self) -> str:
        """Get the name of the message."""
        return self.__name

    @property
    def signals(self) -> dict[str, Signal]:
        """Get the signals of the message."""
        return self.__signals
    
    @property
    def get_topic_name(self) -> str:
        """Get the topic name of the message."""
        return TEENSY_TOPIC_NAME + "/" + self.__subsystem + "/" + self.__name

class SignalTopic(Signal):
    """Class wrapping a signal with additional feature to work with topics."""
    def __init__(self, signal: Signal, name: str, topic_src: str):
        self.__name = name
        super().__init__(signal.type, signal.default_value)
        self.__topic_name = topic_src + "/" + name

    @property
    def name(self) -> str:
        """Get the name of the signal topic."""
        return self.__name

    @property
    def topic_name(self) -> str:
        """Get the topic name of the signal topic."""
        return self.__topic_name

# From this point onward, data should be auto generated from interface sheet
class ArmState(IntEnum):
    Active = 0
    Stop = 1

class ArmDirection(IntEnum):
    Forward = 0
    Backward = 1

TEENSY_CAN_MESSAGES = {
    0: Message(
        id=0,
        subsystem=Subsystems_Names.ALL,
        name="E_STOP",
        signals={
            "E_STOP": Signal(DATA_TYPES.UINT_8, 0)
        }
    ),
    10: Message(
        id=10,
        subsystem=Subsystems_Names.ARM,
        name="ENABLE_ARM",
        signals={
            "enable": Signal(DATA_TYPES.UINT_8, 0)
        }
    ),
    11: Message(
        id=11,
        subsystem=Subsystems_Names.ARM,
        name="MOVE_BASE",
        signals={
            "state":     Signal(DATA_TYPES.UINT_8, ArmState.Stop),
            "direction": Signal(DATA_TYPES.UINT_8, ArmDirection.Forward)
        }
    ),
    12: Message(
        id=12,
        subsystem=Subsystems_Names.ARM,
        name="MOVE_SHOULDER",
        signals={
            "state":     Signal(DATA_TYPES.UINT_8, ArmState.Stop),
            "direction": Signal(DATA_TYPES.UINT_8, ArmDirection.Forward)
        }
    ),
    13: Message(
        id=13,
        subsystem=Subsystems_Names.ARM,
        name="MOVE_ELBOW",
        signals={
            "state":     Signal(DATA_TYPES.UINT_8, ArmState.Stop),
            "direction": Signal(DATA_TYPES.UINT_8, ArmDirection.Forward)
        }
    ),
    14: Message(
        id=14,
        subsystem=Subsystems_Names.ARM,
        name="BEND_WRIST",
        signals={
            "state":     Signal(DATA_TYPES.UINT_8, ArmState.Stop),
            "direction": Signal(DATA_TYPES.UINT_8, ArmDirection.Forward)
        }
    ),
    15: Message(
        id=15,
        subsystem=Subsystems_Names.ARM,
        name="TWIST_WRIST",
        signals={
            "state":     Signal(DATA_TYPES.UINT_8, ArmState.Stop),
            "direction": Signal(DATA_TYPES.UINT_8, ArmDirection.Forward)
        }
    ),
    16: Message(
        id=16,
        subsystem=Subsystems_Names.ARM,
        name="MOVE_CLAW",
        signals={
            "state":     Signal(DATA_TYPES.UINT_8, ArmState.Stop),
            "direction": Signal(DATA_TYPES.UINT_8, ArmDirection.Forward)
        }
    ),
    17: Message(
        id=17,
        subsystem=Subsystems_Names.ARM,
        name="MOVE_SOLINOID",
        signals={
            "state":     Signal(DATA_TYPES.UINT_8, ArmState.Stop),
            "direction": Signal(DATA_TYPES.UINT_8, ArmDirection.Forward)
        }
    )
}

class SIGNALS_TOPICS:
    Enable_Arm              = SignalTopic(TEENSY_CAN_MESSAGES[10].signals["enable"],    "enable",    TEENSY_CAN_MESSAGES[10].get_topic_name)
    Move_Base_State         = SignalTopic(TEENSY_CAN_MESSAGES[11].signals["state"],     "state",     TEENSY_CAN_MESSAGES[11].get_topic_name)
    Move_Base_Direction     = SignalTopic(TEENSY_CAN_MESSAGES[11].signals["direction"], "direction", TEENSY_CAN_MESSAGES[11].get_topic_name)
    Move_Shoulder_State     = SignalTopic(TEENSY_CAN_MESSAGES[12].signals["state"],     "state",     TEENSY_CAN_MESSAGES[12].get_topic_name)
    Move_Shoulder_Direction = SignalTopic(TEENSY_CAN_MESSAGES[12].signals["direction"], "direction", TEENSY_CAN_MESSAGES[12].get_topic_name)
    Move_Elbow_State        = SignalTopic(TEENSY_CAN_MESSAGES[13].signals["state"],     "state",     TEENSY_CAN_MESSAGES[13].get_topic_name)
    Move_Elbow_Direction    = SignalTopic(TEENSY_CAN_MESSAGES[13].signals["direction"], "direction", TEENSY_CAN_MESSAGES[13].get_topic_name)
    Bend_Wrist_State        = SignalTopic(TEENSY_CAN_MESSAGES[14].signals["state"],     "state",     TEENSY_CAN_MESSAGES[14].get_topic_name)
    Bend_Wrist_Direction    = SignalTopic(TEENSY_CAN_MESSAGES[14].signals["direction"], "direction", TEENSY_CAN_MESSAGES[14].get_topic_name)
    Twist_Wrist_State       = SignalTopic(TEENSY_CAN_MESSAGES[15].signals["state"],     "state",     TEENSY_CAN_MESSAGES[15].get_topic_name)
    Twist_Wrist_Direction   = SignalTopic(TEENSY_CAN_MESSAGES[15].signals["direction"], "direction", TEENSY_CAN_MESSAGES[15].get_topic_name)
    Move_Claw_State         = SignalTopic(TEENSY_CAN_MESSAGES[16].signals["state"],     "state",     TEENSY_CAN_MESSAGES[16].get_topic_name)
    Move_Claw_Direction     = SignalTopic(TEENSY_CAN_MESSAGES[16].signals["direction"], "direction", TEENSY_CAN_MESSAGES[16].get_topic_name)
    Move_Solenoid_State     = SignalTopic(TEENSY_CAN_MESSAGES[17].signals["state"],     "state",     TEENSY_CAN_MESSAGES[17].get_topic_name)
    Move_Solenoid_Direction = SignalTopic(TEENSY_CAN_MESSAGES[17].signals["direction"], "direction", TEENSY_CAN_MESSAGES[17].get_topic_name)

# self.__messages = { # Note: dictationaries are ordered in Python 3.7+
#             CONSTANTS.COMPACT_MESSAGES.HEARTBEAT_ID: # byte 0
#             {
#                 "name": CONSTANTS.HEARTBEAT.NAME,
#                 "values": {
#                             # byte 1-2
#                             CONSTANTS.HEARTBEAT.TIMESTAMP_MESSAGE: Signal(CONSTANTS.COMPACT_MESSAGES.UINT_16)} # bits 0-15
#             },
#             CONSTANTS.COMPACT_MESSAGES.N64_ID: { # byte 0
#                 "name": CONSTANTS.N64.NAME,
#                 "values": {
#                             # byte 1
#                             CONSTANTS.N64.BUTTON.A_STR:        Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 0-1
#                             CONSTANTS.N64.BUTTON.B_STR:        Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 2-3
#                             CONSTANTS.N64.BUTTON.L_STR:        Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 4-5
#                             CONSTANTS.N64.BUTTON.R_STR:        Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 6-7

#                             # byte 2
#                             CONSTANTS.N64.BUTTON.C_UP_STR:     Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 0-1
#                             CONSTANTS.N64.BUTTON.C_DOWN_STR:   Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 2-3
#                             CONSTANTS.N64.BUTTON.C_LEFT_STR:   Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 4-5
#                             CONSTANTS.N64.BUTTON.C_RIGHT_STR:  Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 6-7

#                             # byte 3
#                             CONSTANTS.N64.BUTTON.DP_UP_STR:    Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 0-1
#                             CONSTANTS.N64.BUTTON.DP_DOWN_STR:  Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 2-3
#                             CONSTANTS.N64.BUTTON.DP_LEFT_STR:  Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 4-5
#                             CONSTANTS.N64.BUTTON.DP_RIGHT_STR: Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 6-7

#                             # byte 4
#                             CONSTANTS.N64.BUTTON.Z_STR:        Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False)} # bits 0-1
#             },
#             CONSTANTS.COMPACT_MESSAGES.XBOX_ID: { # byte 0
#                 "name": CONSTANTS.XBOX.NAME,
#                 "values": {
#                             # byte 1
#                             CONSTANTS.XBOX.JOYSTICK.AXIS_LY_STR:    Signal(CONSTANTS.COMPACT_MESSAGES.UINT_8_JOYSTICK, CONSTANTS.XBOX.JOYSTICK.NEUTRAL_FLOAT), # bits 0-7

#                             # byte 2
#                             CONSTANTS.XBOX.JOYSTICK.AXIS_RY_STR:    Signal(CONSTANTS.COMPACT_MESSAGES.UINT_8_JOYSTICK, CONSTANTS.XBOX.JOYSTICK.NEUTRAL_FLOAT), # bits 0-7

#                             # byte 3
#                             CONSTANTS.XBOX.BUTTON.A_STR:            Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bit 0-1
#                             CONSTANTS.XBOX.BUTTON.B_STR:            Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bit 2-3
#                             CONSTANTS.XBOX.BUTTON.X_STR:            Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bit 4-5
#                             CONSTANTS.XBOX.BUTTON.Y_STR:            Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bit 6-7

#                             # byte 4
#                             CONSTANTS.XBOX.BUTTON.LEFT_BUMPER_STR:  Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bit 0-1
#                             CONSTANTS.XBOX.BUTTON.RIGHT_BUMPER_STR: Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bit 2-3
#                             CONSTANTS.XBOX.TRIGGER.AXIS_LT_STR:     Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bit 4-5
#                             CONSTANTS.XBOX.TRIGGER.AXIS_RT_STR:     Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False)} # bit 6-7
#             },
#             CONSTANTS.COMPACT_MESSAGES.QUIT_ID: { # byte 0
#                 "name": CONSTANTS.QUIT.NAME,
#                 "values": {
#                             # byte 1
#                             CONSTANTS.QUIT.NAME: Signal(CONSTANTS.COMPACT_MESSAGES.BOOLEAN, CONSTANTS.QUIT.VALUE)}, # bit 0
#             }
