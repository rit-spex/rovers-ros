from std_msgs.msg import Float32, Bool


class CONSTANTS:
    # message to send to show start of new values
    START_MESSAGE = b"\xDE"
    QUIT_MESSAGE = b"\xFE"

    class INPUT_TYPE:
        IS_BUTTON = 0
        IS_AXIS = 1
        IS_TRIGGER = 2

    class XBOX:
        # number of processed buttons and axes
        NUM_AXES = 6
        NUM_USED_AXES = 2
        NUM_TRIGGER = 2
        NUM_BUTTONS = 8  # this number is including the triggers

        class JOYSTICK:
            MIN_VALUE = 0
            NEUTRAL_HEX = b"\x64"
            NEUTRAL_INT = 100
            MAX_VALUE = 200

            AXIS_LX = 0
            AXIS_LY = 1
            AXIS_RX = 3
            AXIS_RY = 4

            LIST_OF_AXIS = [AXIS_LY, AXIS_RY]

        # these are treated like buttons for transfer msgs but are classified as axis
        class TRIGGER:
            AXIS_LT = 2
            AXIS_RT = 5

            LIST_OF_TRIGGERS = [AXIS_LT, AXIS_RT]

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
            LEFT_STICK = 9
            RIGHT_STICK = 10
            HOME = 8

            LIST_OF_BUTTONS = [A, B, X, Y, LEFT_BUMPER, RIGHT_BUMPER, START, SELECT]

        class JOYPAD:
            UP = (0, 1)
            DOWN = (0, -1)
            LEFT = (1, 0)  # Needs to be checked
            RIGHT = (-1, 0)  # Needs to be checked

            LIST_OF_JOYPAD = [UP, DOWN, LEFT, RIGHT]

    class N64:
        NUM_AXES = 2
        NUM_USED_AXES = 0
        NUM_TRIGGER = 0
        NUM_BUTTONS = 14

        class JOYSTICK:
            MIN_VALUE = 0
            NEUTRAL_HEX = b"\x64"
            NEUTRAL_INT = 100
            MAX_VALUE = 200

            AXIS_X = 0
            AXIS_Y = 1

        class BUTTONS:
            SIZE_BUTTON_IN_BITS = 2
            NUM_BUTTONS_PER_BYTE = 8 / SIZE_BUTTON_IN_BITS

            ON = 2
            OFF = 1

            A = 1
            B = 2
            C_UP = 9
            C_DOWN = 0
            C_LEFT = 3
            C_RIGHT = 8
            L = 4
            R = 5
            Z = 6
            START = 12
            DP_UP = 20
            DP_DOWN = 21
            DP_LEFT = 22
            DP_RIGHT = 23

        class JOYPAD:
            UP = (0, 1)
            DOWN = (0, -1)
            LEFT = (-1, 0)
            RIGHT = (1, 0)


TOPICS_JOYSTICK = {
    CONSTANTS.XBOX.JOYSTICK.AXIS_LX: {
        "id": CONSTANTS.XBOX.JOYSTICK.AXIS_LX,
        "name": "LX",
        "val": Float32,
    },
    CONSTANTS.XBOX.JOYSTICK.AXIS_LY: {
        "id": CONSTANTS.XBOX.JOYSTICK.AXIS_LY,
        "name": "LY",
        "val": Float32,
    },
    CONSTANTS.XBOX.JOYSTICK.AXIS_RX: {
        "id": CONSTANTS.XBOX.JOYSTICK.AXIS_RX,
        "name": "RX",
        "val": Float32,
    },
    CONSTANTS.XBOX.JOYSTICK.AXIS_RY: {
        "id": CONSTANTS.XBOX.JOYSTICK.AXIS_RY,
        "name": "RY",
        "val": Float32,
    },
    CONSTANTS.XBOX.TRIGGER.AXIS_LT: {
        "id": CONSTANTS.XBOX.TRIGGER.AXIS_LT,
        "name": "LT",
        "val": Bool,
    },
    CONSTANTS.XBOX.TRIGGER.AXIS_RT: {
        "id": CONSTANTS.XBOX.TRIGGER.AXIS_RT,
        "name": "RT",
        "val": Bool,
    },
}

# TOPICS_TRIGGER = {
#     CONSTANTS.XBOX.TRIGGER.AXIS_LT: {
#         "id": CONSTANTS.XBOX.TRIGGER.AXIS_LT,
#         "name": "LT",
#         "val": Float32
#     },
#     CONSTANTS.XBOX.TRIGGER.AXIS_RT: {
#         "id": CONSTANTS.XBOX.TRIGGER.AXIS_RT,
#         "name": "RT",
#         "val": Float32
#     }
# }

TOPICS_BUTTON = {
    CONSTANTS.XBOX.BUTTONS.A: {
        "id": CONSTANTS.XBOX.BUTTONS.A,
        "name": "A",
        "val": Bool,
    },
    CONSTANTS.XBOX.BUTTONS.B: {
        "id": CONSTANTS.XBOX.BUTTONS.B,
        "name": "B",
        "val": Bool,
    },
    CONSTANTS.XBOX.BUTTONS.X: {
        "id": CONSTANTS.XBOX.BUTTONS.X,
        "name": "X",
        "val": Bool,
    },
    CONSTANTS.XBOX.BUTTONS.Y: {
        "id": CONSTANTS.XBOX.BUTTONS.Y,
        "name": "Y",
        "val": Bool,
    },
    CONSTANTS.XBOX.BUTTONS.LEFT_BUMPER: {
        "id": CONSTANTS.XBOX.BUTTONS.LEFT_BUMPER,
        "name": "LEFT_BUMPER",
        "val": Bool,
    },
    CONSTANTS.XBOX.BUTTONS.RIGHT_BUMPER: {
        "id": CONSTANTS.XBOX.BUTTONS.RIGHT_BUMPER,
        "name": "RIGHT_BUMPER",
        "val": Bool,
    },
    CONSTANTS.XBOX.BUTTONS.SELECT: {
        "id": CONSTANTS.XBOX.BUTTONS.SELECT,
        "name": "LT",
        "val": Bool,
    },
    CONSTANTS.XBOX.BUTTONS.START: {
        "id": CONSTANTS.XBOX.BUTTONS.START,
        "name": "RT",
        "val": Bool,
    },
    CONSTANTS.XBOX.BUTTONS.LEFT_STICK: {
        "id": CONSTANTS.XBOX.BUTTONS.LEFT_STICK,
        "name": "LEFT_STICK",
        "val": Bool,
    },
    CONSTANTS.XBOX.BUTTONS.RIGHT_STICK: {
        "id": CONSTANTS.XBOX.BUTTONS.RIGHT_STICK,
        "name": "RIGHT_STICK",
        "val": Bool,
    },
    CONSTANTS.XBOX.BUTTONS.HOME: {
        "id": CONSTANTS.XBOX.BUTTONS.HOME,
        "name": "HOME",
        "val": Bool,
    },
}
