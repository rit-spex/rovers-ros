# ------------------------------------------------------------------
#                          SPEX ROVER 2025
# ------------------------------------------------------------------
# file name     : CAN_enums.py
# purpose       : contains enums for can communication
#
# authors       : Tyler Halifax
# created on    : 1/14/2026 - Tyler
# last modified : 1/14/2026 - Tyler
# ------------------------------------------------------------------
from enum import IntEnum

class ArmState(IntEnum):
    Active = 0
    Stop = 1

class ArmDirection(IntEnum):
    Forward = 0
    Backward = 1

class CAN_MESSAGE_IDS(IntEnum):
    E_STOP = 0
    ROS_HEARTBEAT = 1
    TEENSY_HEARTBEAT = 2
    ENABLE_CHASSIS = 10
    DRIVE_POWER = 13
    ENABLE_ARM = 20
    # MOVE_BASE = 21
    # MOVE_SHOULDER = 22
    # MOVE_ELBOW = 23
    BEND_WRIST = 24
    TWIST_WRIST = 25
    MOVE_CLAW = 26
    MOVE_SOLENOID = 27
    # READ_BASE = 30
    # READ_SHOULDER = 31
    # READ_ELBOW = 32
    READ_WRIST = 33
    READ_CLAW = 34
    ENABLE_SCIENCE = 40
    MOVE_AUGER = 41
    ENABLE_DRILL = 42
    MOVE_SLIDE = 43
    ENABLE_PUMP1 = 44
    ENABLE_PUMP2 = 45
    ENABLE_PUMP3 = 46
    ENABLE_PUMP4 = 47
    MOVE_SPECTROMETER_SLIDE = 48
    MOVE_FLOROMETER_SLIDE = 49
    ENABLE_FLOROMETER_MICRO_PUMP = 50
    ENABLE_PRIMER = 51
    ENABLE_VIBRATOR = 52
    READ_AUGER = 60
    READ_SLIDE = 61
    READ_DRILL = 62
    READ_PUMPS = 63
    READ_SPECTROMETER_SLIDE = 64
    READ_FLOROMETER_SLIDE = 65
    READ_SPECTROMETER_LIGHT = 66
    READ_FLOROMETER_COLOR_SENSOR = 67

    # OPENCAN
    READ_BASE = 1409 # 0x581
    READ_SHOULDER = 1410 # 0x582
    READ_ELBOW = 1411 # 0x583

    SEND_BASE = 1537 # 0x601
    SEND_SHOULDER = 1538 # 0x602
    SEND_ELBOW = 1539 # 0x603
