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
    DRIVE_POWER = 13
    ENABLE_ARM = 20
    MOVE_BASE = 21
    MOVE_SHOULDER = 22
    MOVE_ELBOW = 23
    BEND_WRIST = 24
    TWIST_WRIST = 25
    MOVE_CLAW = 26
    MOVE_SOLENOID = 27
    ARM_STATUS = 30