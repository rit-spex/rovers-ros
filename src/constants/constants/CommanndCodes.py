class CONSTANTS:
    # message to send to show start of new values
    START_MESSAGE = b"\xDE"

    # message order:
    # byte Axis
    # 2 bits Trigger
    # 2 bits Buttons

    # number of processed buttons and axes
    NUM_AXES = 4
    NUM_USED_AXES = 2
    NUM_TRIGGER = 2
    NUM_BUTTONS = 8  # this number is including the triggers

    class INPUT_TYPE:
        # enum for input type
        IS_BUTTON = 0
        IS_AXIS = 1
        IS_TRIGGER = 2

    class JOYSTICK:
        MIN_VALUE = 0
        NEUTRAL_HEX = b"\x64"
        NEUTRAL_INT = 100
        MAX_VALUE = 200

        AXIS_LX = 0
        AXIS_LY = 1
        AXIS_RX = 2
        AXIS_RY = 3

    # these are treated like buttons for transfer msgs but are classified as axis
    class TRIGGER:
        AXIS_LT = 4
        AXIS_RT = 5

    class BUTTONS:
        SIZE_BUTTON_IN_BITS = 2
        NUM_BUTTONS_PER_BYTE = 8 / SIZE_BUTTON_IN_BITS

        # I choose 2 to represent ON b/c it equals the bit value of 10
        # this means if it error and one of the bit was flipped then it would ignore it.
        # this would make it so 2 bit would need to be changed to produce the wrong result
        ON = 2
        OFF = 1

        A = 0
        B = 1
        X = 2
        Y = 3
        LEFT_BUMPER = 4
        RIGHT_BUMPER = 5
        SELECT = 6
        START = 7
        LEFT_STICK = 8
        RIGHT_STICK = 9
        HOME = 10

    class JOYPAD:
        UP = (0, 1)
        DOWN = (0, -1)
        LEFT = (1, 0)  # Needs to be checked
        RIGHT = (-1, 0)  # Needs to be checked


TOPICS_JOYSTICK = {
    CONSTANTS.JOYSTICK.AXIS_LX: {
        "id": CONSTANTS.JOYSTICK.AXIS_LX,
        "name": "AXIS_LX",
        "val": float,
    },
    CONSTANTS.JOYSTICK.AXIS_LY: {
        "id": CONSTANTS.JOYSTICK.AXIS_LY,
        "name": "AXIS_LY",
        "val": float,
    },
    CONSTANTS.JOYSTICK.AXIS_RX: {
        "id": CONSTANTS.JOYSTICK.AXIS_RX,
        "name": "AXIS_RX",
        "val": float,
    },
    CONSTANTS.JOYSTICK.AXIS_RY: {
        "id": CONSTANTS.JOYSTICK.AXIS_RY,
        "name": "AXIS_RY",
        "val": float,
    },
}

TOPICS_BUTTON = {
    CONSTANTS.BUTTONS.A: {
        "id": CONSTANTS.BUTTONS.A,
        "name": "A",
        "val": bool,
    },
    CONSTANTS.BUTTONS.B: {
        "id": CONSTANTS.BUTTONS.B,
        "name": "B",
        "val": bool,
    },
    CONSTANTS.BUTTONS.X: {
        "id": CONSTANTS.BUTTONS.X,
        "name": "X",
        "val": bool,
    },
    CONSTANTS.BUTTONS.Y: {
        "id": CONSTANTS.BUTTONS.Y,
        "name": "Y",
        "val": bool,
    },
    CONSTANTS.BUTTONS.LEFT_BUMPER: {
        "id": CONSTANTS.BUTTONS.LEFT_BUMPER,
        "name": "LEFT_BUMPER",
        "val": bool,
    },
    CONSTANTS.BUTTONS.RIGHT_BUMPER: {
        "id": CONSTANTS.BUTTONS.RIGHT_BUMPER,
        "name": "RIGHT_BUMPER",
        "val": bool,
    },
    CONSTANTS.BUTTONS.SELECT: {
        "id": CONSTANTS.BUTTONS.SELECT,
        "name": "SELECT",
        "val": bool,
    },
    CONSTANTS.BUTTONS.START: {
        "id": CONSTANTS.BUTTONS.START,
        "name": "START",
        "val": bool,
    },
    CONSTANTS.BUTTONS.LEFT_STICK: {
        "id": CONSTANTS.BUTTONS.LEFT_STICK,
        "name": "LEFT_STICK",
        "val": bool,
    },
    CONSTANTS.BUTTONS.RIGHT_STICK: {
        "id": CONSTANTS.BUTTONS.RIGHT_STICK,
        "name": "RIGHT_STICK",
        "val": bool,
    },
    CONSTANTS.BUTTONS.HOME: {
        "id": CONSTANTS.BUTTONS.HOME,
        "name": "HOME",
        "val": bool,
    },
}
