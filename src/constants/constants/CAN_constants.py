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
from CAN_structs import DATA_TYPE, Signal, Message
from CAN_enums import CAN_MESSAGE_IDS, ArmState, ArmDirection

class Subsystems_Names:
    ALL     = "ALL"
    CHASSIS = "CHASSIS"
    ARM     = "ARM"
    SCIENCE = "SCIENCE"

# Define data types for can bus communication
class DATA_TYPES:
    UINT_8          = DATA_TYPE(8,  0x00)
    UINT_16         = DATA_TYPE(16, 0x01)
    UINT_32         = DATA_TYPE(32, 0x02)
    FLOAT_32        = DATA_TYPE(32, 0x03)

TEENSY_CAN_MESSAGES = {
    CAN_MESSAGE_IDS.E_STOP: Message(
        id=CAN_MESSAGE_IDS.E_STOP,
        subsystem=Subsystems_Names.ALL,
        name="E_STOP",
        signals={
            "E_STOP": Signal(DATA_TYPES.UINT_8, 1)
        },
        isforJetson=False
    ),
    CAN_MESSAGE_IDS.ROS_HEARTBEAT: Message(
        id=CAN_MESSAGE_IDS.ROS_HEARTBEAT,
        subsystem=Subsystems_Names.ALL,
        name="Ros-Heartbeat",
        signals={
            "timestamp": Signal(DATA_TYPES.UINT_32, 0)
        },
        isforJetson=False
    ),
    CAN_MESSAGE_IDS.DRIVE_POWER: Message(
        id=CAN_MESSAGE_IDS.DRIVE_POWER,
        subsystem=Subsystems_Names.ALL,
        name="Drive_Power",
        signals={
            "left": Signal(DATA_TYPES.UINT_8, 0),
            "right": Signal(DATA_TYPES.UINT_8, 0)
        },
        isforJetson=False
    ),
    CAN_MESSAGE_IDS.ENABLE_ARM: Message(
        id=CAN_MESSAGE_IDS.ENABLE_ARM,
        subsystem=Subsystems_Names.ARM,
        name="ENABLE_ARM",
        signals={
            "enable": Signal(DATA_TYPES.UINT_8, 0)
        },
        isforJetson=False
    ),
    CAN_MESSAGE_IDS.MOVE_BASE: Message(
        id=CAN_MESSAGE_IDS.MOVE_BASE,
        subsystem=Subsystems_Names.ARM,
        name="MOVE_BASE",
        signals={
            "state":     Signal(DATA_TYPES.UINT_8, ArmState.Stop),
            "direction": Signal(DATA_TYPES.UINT_8, ArmDirection.Forward)
        },
        isforJetson=False
    ),
    CAN_MESSAGE_IDS.MOVE_SHOULDER: Message(
        id=CAN_MESSAGE_IDS.MOVE_SHOULDER,
        subsystem=Subsystems_Names.ARM,
        name="MOVE_SHOULDER",
        signals={
            "state":     Signal(DATA_TYPES.UINT_8, ArmState.Stop),
            "direction": Signal(DATA_TYPES.UINT_8, ArmDirection.Forward)
        },
        isforJetson=False
    ),
    CAN_MESSAGE_IDS.MOVE_ELBOW: Message(
        id=CAN_MESSAGE_IDS.MOVE_ELBOW,
        subsystem=Subsystems_Names.ARM,
        name="MOVE_ELBOW",
        signals={
            "state":     Signal(DATA_TYPES.UINT_8, ArmState.Stop),
            "direction": Signal(DATA_TYPES.UINT_8, ArmDirection.Forward)
        },
        isforJetson=False
    ),
    CAN_MESSAGE_IDS.BEND_WRIST: Message(
        id=CAN_MESSAGE_IDS.BEND_WRIST,
        subsystem=Subsystems_Names.ARM,
        name="BEND_WRIST",
        signals={
            "state":     Signal(DATA_TYPES.UINT_8, ArmState.Stop),
            "direction": Signal(DATA_TYPES.UINT_8, ArmDirection.Forward)
        },
        isforJetson=False
    ),
    CAN_MESSAGE_IDS.TWIST_WRIST: Message(
        id=CAN_MESSAGE_IDS.TWIST_WRIST,
        subsystem=Subsystems_Names.ARM,
        name="TWIST_WRIST",
        signals={
            "state":     Signal(DATA_TYPES.UINT_8, ArmState.Stop),
            "direction": Signal(DATA_TYPES.UINT_8, ArmDirection.Forward)
        },
        isforJetson=False
    ),
    CAN_MESSAGE_IDS.MOVE_CLAW: Message(
        id=CAN_MESSAGE_IDS.MOVE_CLAW,
        subsystem=Subsystems_Names.ARM,
        name="MOVE_CLAW",
        signals={
            "state":     Signal(DATA_TYPES.UINT_8, ArmState.Stop),
            "direction": Signal(DATA_TYPES.UINT_8, ArmDirection.Forward)
        },
        isforJetson=False
    ),
    CAN_MESSAGE_IDS.MOVE_SOLENOID: Message(
        id=CAN_MESSAGE_IDS.MOVE_SOLENOID,
        subsystem=Subsystems_Names.ARM,
        name="MOVE_SOLENOID",
        signals={
            "state":     Signal(DATA_TYPES.UINT_8, ArmState.Stop),
            "direction": Signal(DATA_TYPES.UINT_8, ArmDirection.Forward)
        },
        isforJetson=False
    )
}