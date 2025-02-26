from enum import IntEnum


class CHANNEL(IntEnum):
    JETSON = 0
    MAIN_BODY = 1
    SCIENCE_BOARD = 2
    ARM_BOARD = 3

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
        "id": 15,
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
