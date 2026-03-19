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
from constants.CAN_enums import CAN_MESSAGE_IDS, ArmState, ArmDirection, SubsystemsIDs
from constants.CAN_structs import DATA_TYPE, Signal, Message


# Define subsystem names
class Subsystems_Names:
    GENERAL = "GENERAL"
    CHASSIS = "CHASSIS"
    ARM = "ARM"
    SCIENCE = "SCIENCE"


# Define data types for can bus communication
# See the following for format details
# https://docs.python.org/3/library/struct.html#struct-alignment
class DATA_TYPES:
    UINT_8 = DATA_TYPE(0x00, 8, "B")
    UINT_16 = DATA_TYPE(0x01, 16, "H")
    UINT_32 = DATA_TYPE(0x02, 32, "I")
    FLOAT_32 = DATA_TYPE(0x03, 32, "f")
    INT_8 = DATA_TYPE(0x04, 8, "b")
    INT_16 = DATA_TYPE(0x05, 16, "h")
    INT_32 = DATA_TYPE(0x06, 32, "i")


TEENSY_CAN_MESSAGES = {
    CAN_MESSAGE_IDS.E_STOP: Message(
        id=CAN_MESSAGE_IDS.E_STOP,
        subsystem=Subsystems_Names.GENERAL,
        name="E_STOP",
        signals={"E_STOP": Signal(DATA_TYPES.UINT_8, 1)},
        isforJetson=False,
    ),
    CAN_MESSAGE_IDS.ROS_HEARTBEAT: Message(
        id=CAN_MESSAGE_IDS.ROS_HEARTBEAT,
        subsystem=Subsystems_Names.GENERAL,
        name="ROS_Heartbeat",
        signals={
            "source": Signal(DATA_TYPES.UINT_8, SubsystemsIDs.GENERAL),
            "timestamp": Signal(DATA_TYPES.UINT_32, 0),
            "enabled": Signal(DATA_TYPES.UINT_8, 1),
        },
        isforJetson=False,
    ),
    CAN_MESSAGE_IDS.TEENSY_HEARTBEAT: Message(
        id=CAN_MESSAGE_IDS.TEENSY_HEARTBEAT,
        subsystem=Subsystems_Names.GENERAL,
        name="Teensy_Heartbeat",
        signals={
            "source": Signal(DATA_TYPES.UINT_8, SubsystemsIDs.GENERAL),
            "timestamp": Signal(DATA_TYPES.UINT_32, 0),
            "enabled": Signal(DATA_TYPES.UINT_8, 0),
        },
        isforJetson=True,
    ),
    CAN_MESSAGE_IDS.ENABLE_CHASSIS: Message(
        id=CAN_MESSAGE_IDS.ENABLE_CHASSIS,
        subsystem=Subsystems_Names.CHASSIS,
        name="ENABLE_CHASSIS",
        signals={"enable": Signal(DATA_TYPES.UINT_8, 0)},
        isforJetson=False,
    ),
    CAN_MESSAGE_IDS.DRIVE_POWER: Message(
        id=CAN_MESSAGE_IDS.DRIVE_POWER,
        subsystem=Subsystems_Names.CHASSIS,
        name="Drive_Power",
        signals={
            "left": Signal(DATA_TYPES.FLOAT_32, 0),
            "right": Signal(DATA_TYPES.FLOAT_32, 0),
        },
        isforJetson=False,
    ),
    CAN_MESSAGE_IDS.ENABLE_ARM: Message(
        id=CAN_MESSAGE_IDS.ENABLE_ARM,
        subsystem=Subsystems_Names.ARM,
        name="ENABLE_ARM",
        signals={"enable": Signal(DATA_TYPES.UINT_8, 0)},
        isforJetson=False,
    ),
    # CAN_MESSAGE_IDS.MOVE_BASE: Message(
    #     id=CAN_MESSAGE_IDS.MOVE_BASE,
    #     subsystem=Subsystems_Names.ARM,
    #     name="MOVE_BASE",
    #     signals={
    #         "state":     Signal(DATA_TYPES.UINT_8, ArmState.Stop),
    #         "direction": Signal(DATA_TYPES.UINT_8, ArmDirection.Forward)
    #     },
    #     isforJetson=False
    # ),
    # CAN_MESSAGE_IDS.MOVE_SHOULDER: Message(
    #     id=CAN_MESSAGE_IDS.MOVE_SHOULDER,
    #     subsystem=Subsystems_Names.ARM,
    #     name="MOVE_SHOULDER",
    #     signals={
    #         "state":     Signal(DATA_TYPES.UINT_8, ArmState.Stop),
    #         "direction": Signal(DATA_TYPES.UINT_8, ArmDirection.Forward)
    #     },
    #     isforJetson=False
    # ),
    # CAN_MESSAGE_IDS.MOVE_ELBOW: Message(
    #     id=CAN_MESSAGE_IDS.MOVE_ELBOW,
    #     subsystem=Subsystems_Names.ARM,
    #     name="MOVE_ELBOW",
    #     signals={
    #         "state":     Signal(DATA_TYPES.UINT_8, ArmState.Stop),
    #         "direction": Signal(DATA_TYPES.UINT_8, ArmDirection.Forward)
    #     },
    #     isforJetson=False
    # ),
    CAN_MESSAGE_IDS.BEND_WRIST: Message(
        id=CAN_MESSAGE_IDS.BEND_WRIST,
        subsystem=Subsystems_Names.ARM,
        name="BEND_WRIST",
        signals={
            "Position": Signal(DATA_TYPES.FLOAT_32, 0),
        },
        isforJetson=False,
    ),
    CAN_MESSAGE_IDS.TWIST_WRIST: Message(
        id=CAN_MESSAGE_IDS.TWIST_WRIST,
        subsystem=Subsystems_Names.ARM,
        name="TWIST_WRIST",
        signals={
            "Position": Signal(DATA_TYPES.FLOAT_32, 0),
        },
        isforJetson=False,
    ),
    CAN_MESSAGE_IDS.MOVE_CLAW: Message(
        id=CAN_MESSAGE_IDS.MOVE_CLAW,
        subsystem=Subsystems_Names.ARM,
        name="MOVE_CLAW",
        signals={
            "State": Signal(DATA_TYPES.UINT_8, 0),
            "Position": Signal(DATA_TYPES.FLOAT_32, 0),
        },
        isforJetson=False,
    ),
    CAN_MESSAGE_IDS.MOVE_SOLENOID: Message(
        id=CAN_MESSAGE_IDS.MOVE_SOLENOID,
        subsystem=Subsystems_Names.ARM,
        name="MOVE_SOLENOID",
        signals={
            "Enabled": Signal(DATA_TYPES.UINT_8, 0),
        },
        isforJetson=False,
    ),
    CAN_MESSAGE_IDS.READ_WRIST_BEND: Message(
        id=CAN_MESSAGE_IDS.READ_WRIST_BEND,
        subsystem=Subsystems_Names.ARM,
        name="READ_WRIST_BEND",
        signals={
            "Position": Signal(DATA_TYPES.FLOAT_32, 0),
        },
        isforJetson=True,
    ),
    CAN_MESSAGE_IDS.READ_WRIST_TWIST: Message(
        id=CAN_MESSAGE_IDS.READ_WRIST_TWIST,
        subsystem=Subsystems_Names.ARM,
        name="READ_WRIST_TWIST",
        signals={
            "Position": Signal(DATA_TYPES.FLOAT_32, 0),
        },
        isforJetson=True,
    ),
    CAN_MESSAGE_IDS.READ_CLAW: Message(
        id=CAN_MESSAGE_IDS.READ_CLAW,
        subsystem=Subsystems_Names.ARM,
        name="READ_CLAW",
        signals={
            "State": Signal(DATA_TYPES.UINT_8, 0),
            "Position": Signal(DATA_TYPES.FLOAT_32, 0),
        },
        isforJetson=True,
    ),
    CAN_MESSAGE_IDS.ENABLE_SCIENCE: Message(
        id=CAN_MESSAGE_IDS.ENABLE_SCIENCE,
        subsystem=Subsystems_Names.SCIENCE,
        name="ENABLE_SCIENCE",
        signals={
            "Enabled": Signal(DATA_TYPES.UINT_8, 0),
        },
        isforJetson=False,
    ),
    CAN_MESSAGE_IDS.MOVE_AUGER: Message(
        id=CAN_MESSAGE_IDS.MOVE_AUGER,
        subsystem=Subsystems_Names.SCIENCE,
        name="MOVE_AUGER",
        signals={
            "position": Signal(DATA_TYPES.INT_32, 0),
            "home": Signal(DATA_TYPES.UINT_8, 0),
        },
        isforJetson=False,
    ),
    CAN_MESSAGE_IDS.ENABLE_DRILL: Message(
        id=CAN_MESSAGE_IDS.ENABLE_DRILL,
        subsystem=Subsystems_Names.SCIENCE,
        name="ENABLE_DRILL",
        signals={
            "Enabled": Signal(DATA_TYPES.UINT_8, 0),
        },
        isforJetson=False,
    ),
    CAN_MESSAGE_IDS.MOVE_SLIDE: Message(
        id=CAN_MESSAGE_IDS.MOVE_SLIDE,
        subsystem=Subsystems_Names.SCIENCE,
        name="MOVE_SLIDE",
        signals={
            "stage": Signal(DATA_TYPES.UINT_8, 0),
        },
        isforJetson=False,
    ),
    CAN_MESSAGE_IDS.ENABLE_PUMP1: Message(
        id=CAN_MESSAGE_IDS.ENABLE_PUMP1,
        subsystem=Subsystems_Names.SCIENCE,
        name="ENABLE_PUMP1",
        signals={
            "Enabled": Signal(DATA_TYPES.UINT_8, 0),
        },
        isforJetson=False,
    ),
    CAN_MESSAGE_IDS.ENABLE_PUMP2: Message(
        id=CAN_MESSAGE_IDS.ENABLE_PUMP2,
        subsystem=Subsystems_Names.SCIENCE,
        name="ENABLE_PUMP2",
        signals={
            "Enabled": Signal(DATA_TYPES.UINT_8, 0),
        },
        isforJetson=False,
    ),
    CAN_MESSAGE_IDS.ENABLE_PUMP3: Message(
        id=CAN_MESSAGE_IDS.ENABLE_PUMP3,
        subsystem=Subsystems_Names.SCIENCE,
        name="ENABLE_PUMP3",
        signals={
            "Enabled": Signal(DATA_TYPES.UINT_8, 0),
        },
        isforJetson=False,
    ),
    CAN_MESSAGE_IDS.ENABLE_PUMP4: Message(
        id=CAN_MESSAGE_IDS.ENABLE_PUMP4,
        subsystem=Subsystems_Names.SCIENCE,
        name="ENABLE_PUMP4",
        signals={
            "Enabled": Signal(DATA_TYPES.UINT_8, 0),
        },
        isforJetson=False,
    ),
    CAN_MESSAGE_IDS.MOVE_SPECTROMETER_SLIDE: Message(
        id=CAN_MESSAGE_IDS.MOVE_SPECTROMETER_SLIDE,
        subsystem=Subsystems_Names.SCIENCE,
        name="MOVE_SPECTROMETER_SLIDE",
        signals={
            "Stage": Signal(DATA_TYPES.UINT_8, 0),
            "Home": Signal(DATA_TYPES.UINT_8, 0),
        },
        isforJetson=False,
    ),
    CAN_MESSAGE_IDS.MOVE_FLOROMETER_SLIDE: Message(
        id=CAN_MESSAGE_IDS.MOVE_FLOROMETER_SLIDE,
        subsystem=Subsystems_Names.SCIENCE,
        name="MOVE_FLOROMETER_SLIDE",
        signals={
            "Stage": Signal(DATA_TYPES.UINT_8, 0),
            "Home": Signal(DATA_TYPES.UINT_8, 0),
        },
        isforJetson=False,
    ),
    CAN_MESSAGE_IDS.ENABLE_FLOROMETER_MICRO_PUMP: Message(
        id=CAN_MESSAGE_IDS.ENABLE_FLOROMETER_MICRO_PUMP,
        subsystem=Subsystems_Names.SCIENCE,
        name="ENABLE_FLOROMETER_MICRO_PUMP",
        signals={
            "Enabled": Signal(DATA_TYPES.UINT_8, 0),
        },
        isforJetson=False,
    ),
    CAN_MESSAGE_IDS.ENABLE_PRIMER: Message(
        id=CAN_MESSAGE_IDS.ENABLE_PRIMER,
        subsystem=Subsystems_Names.SCIENCE,
        name="ENABLE_PRIMER",
        signals={
            "Enabled": Signal(DATA_TYPES.UINT_8, 0),
        },
        isforJetson=False,
    ),
    CAN_MESSAGE_IDS.ENABLE_VIBRATOR: Message(
        id=CAN_MESSAGE_IDS.ENABLE_VIBRATOR,
        subsystem=Subsystems_Names.SCIENCE,
        name="ENABLE_VIBRATOR",
        signals={
            "Enabled": Signal(DATA_TYPES.UINT_8, 0),
        },
        isforJetson=False,
    ),
    CAN_MESSAGE_IDS.READ_AUGER: Message(
        id=CAN_MESSAGE_IDS.READ_AUGER,
        subsystem=Subsystems_Names.SCIENCE,
        name="READ_AUGER",
        signals={
            "Position": Signal(DATA_TYPES.INT_32, 0),
        },
        isforJetson=True,
    ),
    CAN_MESSAGE_IDS.READ_SLIDE: Message(
        id=CAN_MESSAGE_IDS.READ_SLIDE,
        subsystem=Subsystems_Names.SCIENCE,
        name="READ_SLIDE",
        signals={
            "Stage": Signal(DATA_TYPES.UINT_8, 0),
            "Position": Signal(DATA_TYPES.INT_32, 0),
            "Limit_Switch": Signal(DATA_TYPES.UINT_8, 0),
        },
        isforJetson=True,
    ),
    CAN_MESSAGE_IDS.READ_DRILL: Message(
        id=CAN_MESSAGE_IDS.READ_DRILL,
        subsystem=Subsystems_Names.SCIENCE,
        name="READ_DRILL",
        signals={
            "Enabled": Signal(DATA_TYPES.UINT_8, 0),
        },
        isforJetson=True,
    ),
    CAN_MESSAGE_IDS.READ_PUMPS: Message(
        id=CAN_MESSAGE_IDS.READ_PUMPS,
        subsystem=Subsystems_Names.SCIENCE,
        name="READ_PUMPS",
        signals={
            "Pump1_Enabled": Signal(DATA_TYPES.UINT_8, 0),
            "Pump2_Enabled": Signal(DATA_TYPES.UINT_8, 0),
            "Pump3_Enabled": Signal(DATA_TYPES.UINT_8, 0),
            "Pump4_Enabled": Signal(DATA_TYPES.UINT_8, 0),
        },
        isforJetson=True,
    ),
    CAN_MESSAGE_IDS.READ_SPECTROMETER_SLIDE: Message(
        id=CAN_MESSAGE_IDS.READ_SPECTROMETER_SLIDE,
        subsystem=Subsystems_Names.SCIENCE,
        name="READ_SPECTROMETER_SLIDE",
        signals={
            "Stage": Signal(DATA_TYPES.UINT_8, 0),
            "Position": Signal(DATA_TYPES.INT_32, 0),
            "Limit_Switch": Signal(DATA_TYPES.UINT_8, 0),
        },
        isforJetson=True,
    ),
    CAN_MESSAGE_IDS.READ_FLOROMETER_SLIDE: Message(
        id=CAN_MESSAGE_IDS.READ_FLOROMETER_SLIDE,
        subsystem=Subsystems_Names.SCIENCE,
        name="READ_FLOROMETER_SLIDE",
        signals={
            "Stage": Signal(DATA_TYPES.UINT_8, 0),
            "Position": Signal(DATA_TYPES.INT_32, 0),
            "Limit_Switch": Signal(DATA_TYPES.UINT_8, 0),
        },
        isforJetson=True,
    ),
    CAN_MESSAGE_IDS.READ_SPECTROMETER_LIGHT: Message(
        id=CAN_MESSAGE_IDS.READ_SPECTROMETER_LIGHT,
        subsystem=Subsystems_Names.SCIENCE,
        name="READ_SPECTROMETER_LIGHT",
        signals={
            "wavelength": Signal(DATA_TYPES.FLOAT_32, 0),
        },
        isforJetson=True,
    ),
    CAN_MESSAGE_IDS.READ_FLOROMETER_COLOR_SENSOR: Message(
        id=CAN_MESSAGE_IDS.READ_FLOROMETER_COLOR_SENSOR,
        subsystem=Subsystems_Names.SCIENCE,
        name="READ_FLOROMETER_COLOR_SENSOR",
        signals={
            "red": Signal(DATA_TYPES.INT_16, 0),
            "green": Signal(DATA_TYPES.INT_16, 0),
            "blue": Signal(DATA_TYPES.INT_16, 0),
            "violet": Signal(DATA_TYPES.INT_16, 0),
        },
        isforJetson=True,
    ),
    CAN_MESSAGE_IDS.SEND_BASE: Message(
        id=CAN_MESSAGE_IDS.SEND_BASE,
        subsystem=Subsystems_Names.ARM,
        name="SEND_BASE",
        signals={
            "MESSAGE_TYPE": Signal(
                DATA_TYPES.UINT_8, int(0x2B)
            ),  # Default with disable message
            "OPCODE_LSB": Signal(
                DATA_TYPES.UINT_8, int(0x40)
            ),  # Default with disable message
            "OPCODE_MSB": Signal(
                DATA_TYPES.UINT_8, int(0x60)
            ),  # Default with disable message
            "EMPTY": Signal(DATA_TYPES.UINT_8, 0),
            "DATA1": Signal(DATA_TYPES.UINT_8, 0),
            "DATA2": Signal(DATA_TYPES.UINT_8, 0),
            "DATA3": Signal(DATA_TYPES.UINT_8, 0),
            "DATA4": Signal(DATA_TYPES.UINT_8, 0),
        },
        isforJetson=False,
    ),
    CAN_MESSAGE_IDS.SEND_SHOULDER: Message(
        id=CAN_MESSAGE_IDS.SEND_SHOULDER,
        subsystem=Subsystems_Names.ARM,
        name="SEND_SHOULDER",
        signals={
            "MESSAGE_TYPE": Signal(
                DATA_TYPES.UINT_8, int(0x2B)
            ),  # Default with disable message
            "OPCODE_LSB": Signal(
                DATA_TYPES.UINT_8, int(0x40)
            ),  # Default with disable message
            "OPCODE_MSB": Signal(
                DATA_TYPES.UINT_8, int(0x60)
            ),  # Default with disable message
            "EMPTY": Signal(DATA_TYPES.UINT_8, 0),
            "DATA1": Signal(DATA_TYPES.UINT_8, 0),
            "DATA2": Signal(DATA_TYPES.UINT_8, 0),
            "DATA3": Signal(DATA_TYPES.UINT_8, 0),
            "DATA4": Signal(DATA_TYPES.UINT_8, 0),
        },
        isforJetson=False,
    ),
    CAN_MESSAGE_IDS.SEND_ELBOW: Message(
        id=CAN_MESSAGE_IDS.SEND_ELBOW,
        subsystem=Subsystems_Names.ARM,
        name="SEND_ELBOW",
        signals={
            "MESSAGE_TYPE": Signal(
                DATA_TYPES.UINT_8, int(0x2B)
            ),  # Default with disable message
            "OPCODE_LSB": Signal(
                DATA_TYPES.UINT_8, int(0x40)
            ),  # Default with disable message
            "OPCODE_MSB": Signal(
                DATA_TYPES.UINT_8, int(0x60)
            ),  # Default with disable message
            "EMPTY": Signal(DATA_TYPES.UINT_8, 0),
            "DATA1": Signal(DATA_TYPES.UINT_8, 0),
            "DATA2": Signal(DATA_TYPES.UINT_8, 0),
            "DATA3": Signal(DATA_TYPES.UINT_8, 0),
            "DATA4": Signal(DATA_TYPES.UINT_8, 0),
        },
        isforJetson=False,
    ),
    CAN_MESSAGE_IDS.READ_BASE: Message(
        id=CAN_MESSAGE_IDS.READ_BASE,
        subsystem=Subsystems_Names.ARM,
        name="READ_BASE",
        signals={
            "MESSAGE_TYPE": Signal(DATA_TYPES.UINT_8, int(0x43)),
            "OPCODE_LSB": Signal(DATA_TYPES.UINT_8, 0),
            "OPCODE_MSB": Signal(DATA_TYPES.UINT_8, int(0x60)),
            "EMPTY": Signal(DATA_TYPES.UINT_8, 0),
            "DATA1": Signal(DATA_TYPES.UINT_8, 0),
            "DATA2": Signal(DATA_TYPES.UINT_8, 0),
            "DATA3": Signal(DATA_TYPES.UINT_8, 0),
            "DATA4": Signal(DATA_TYPES.UINT_8, 0),
        },
        isforJetson=True,
    ),
    CAN_MESSAGE_IDS.READ_SHOULDER: Message(
        id=CAN_MESSAGE_IDS.READ_SHOULDER,
        subsystem=Subsystems_Names.ARM,
        name="READ_SHOULDER",
        signals={
            "MESSAGE_TYPE": Signal(DATA_TYPES.UINT_8, int(0x43)),
            "OPCODE_LSB": Signal(DATA_TYPES.UINT_8, 0),
            "OPCODE_MSB": Signal(DATA_TYPES.UINT_8, int(0x60)),
            "EMPTY": Signal(DATA_TYPES.UINT_8, 0),
            "DATA1": Signal(DATA_TYPES.UINT_8, 0),
            "DATA2": Signal(DATA_TYPES.UINT_8, 0),
            "DATA3": Signal(DATA_TYPES.UINT_8, 0),
            "DATA4": Signal(DATA_TYPES.UINT_8, 0),
        },
        isforJetson=True,
    ),
    CAN_MESSAGE_IDS.READ_ELBOW: Message(
        id=CAN_MESSAGE_IDS.READ_ELBOW,
        subsystem=Subsystems_Names.ARM,
        name="READ_ELBOW",
        signals={
            "MESSAGE_TYPE": Signal(DATA_TYPES.UINT_8, int(0x43)),
            "OPCODE_LSB": Signal(DATA_TYPES.UINT_8, 0),
            "OPCODE_MSB": Signal(DATA_TYPES.UINT_8, int(0x60)),
            "EMPTY": Signal(DATA_TYPES.UINT_8, 0),
            "DATA1": Signal(DATA_TYPES.UINT_8, 0),
            "DATA2": Signal(DATA_TYPES.UINT_8, 0),
            "DATA3": Signal(DATA_TYPES.UINT_8, 0),
            "DATA4": Signal(DATA_TYPES.UINT_8, 0),
        },
        isforJetson=True,
    ),
}
