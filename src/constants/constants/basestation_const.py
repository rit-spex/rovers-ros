# ------------------------------------------------------------------
#                          SPEX ROVER 2025
# ------------------------------------------------------------------
# file name     : basestation_const.py
# purpose       : contains constants for communication between the basestation and rover.
#                   The first half of the file contains classes that define the structure of the data.
#                   The second half of the file contains the actual data used by the basestation.
# authors       : Tyler Halifax
# created on    : 11/12/2025 - Tyler
# last modified : 11/12/2025 - Tyler
# ------------------------------------------------------------------

from typing import Any

#*******************************************************************************************************************#
#***************************           Classes to define structure                   *******************************#
#*******************************************************************************************************************#

# Enum-like class to represent data types, and their properties.
class DATA_TYPE:
    def __init__(self, num_bits: int, id: int) -> None:
        self.__num_bits = num_bits
        self.__id = id
    @property
    def get_num_bits(self) -> int:
        """Get the number of bits for the data type."""
        return self.__num_bits
    @property
    def get_id(self) -> int:
        """Get the ID of the data type."""
        return self.__id

# Enum-like class to represent message formats.
# class MessageFormat():
#     def __init__(self, name: str, id: int, values: dict[str, Any]) -> None:
#         self.__name = name
#         self.__id = id
#         self.__values = values
#     @property
#     def get_name(self) -> str:
#         """Get the name of the message format."""
#         return self.__name
#     @property
#     def get_id(self) -> int:
#         """Get the ID of the message format."""
#         return self.__id
#     @property
#     def get_values(self) -> dict[str, Any]:
#         """Get the values dictionary of the message format."""
#         return self.__values

# Class to represent a signal within a message.
class Signal:
    """Class representing a signal with a name and type."""
    def __init__(self, type: DATA_TYPE, default_value: Any = 0): # TODO: allow it to support it native types
        self.__type = type
        self.__default_value = default_value
        self.__value = self.__default_value
    @property
    def get_type(self) -> DATA_TYPE:
        """Get the type of the signal."""
        return self.__type
    @property
    def get_value(self) -> Any:
        """Get the current value of the signal."""
        return self.__value

    @property
    def default_value(self) -> Any:
        """Get the default value of the signal."""
        return self.__default_value

    def toString(self) -> str:
        """Get the string representation of the signal."""
        return f"Signal(type={self.__type}, value={self.__value}, default_value={self.__default_value})"

#*******************************************************************************************************************#
#***************************           Classes that hold the data                    *******************************#
#*******************************************************************************************************************#

# enum of data types in bits
class DATA_TYPES:
    UINT_2_BOOL     = DATA_TYPE(2,  0x01) # 01 = false, 10 = true
    UINT_8          = DATA_TYPE(8,  0x02)
    UINT_16         = DATA_TYPE(16, 0x03)
    UINT_8_JOYSTICK = DATA_TYPE(8,  0x04) # convert to float on decoding
    BOOLEAN         = DATA_TYPE(1,  0x05)

# this is made to make sure the messages IDs are not used twice
class MessageIDs:
    HEARTBEAT_ID = int(0xAA)
    N64_ID       = int(0xDF)   # The Id for the N64
    XBOX_ID      = int(0xF0)   # The Id for the Xbox
    QUIT_ID      = int(0xFE)   # QUIT_MESSAGE

class ControllerConfig:
    class N64:
        NAME = "n64"
        NUM_AXES = 2
        NUM_USED_AXES = 0
        NUM_TRIGGER = 0
        NUM_BUTTONS = 14

        class INPUT_TYPE:
            IS_BUTTON = 0
            IS_AXIS = 1
            IS_TRIGGER = 2

        class JOYSTICK:
            MIN_VALUE = 0
            NEUTRAL_HEX = b'\x64'
            NEUTRAL_INT   = 100
            MAX_VALUE = 200

            AXIS_X = 0
            AXIS_X_STR = "AXIS_X"
            AXIS_Y = 1
            AXIS_Y_STR = "AXIS_Y"

        class BUTTON:
            SIZE_BUTTON_IN_BITS = 2
            NUM_BUTTONS_PER_BYTE = 8 / SIZE_BUTTON_IN_BITS

            ON       = 2
            OFF      = 1

            A        = 1
            A_STR    = "A"
            B        = 2
            B_STR    = "B"
            C_UP     = 9
            C_UP_STR = "C_UP"
            C_DOWN   = 0
            C_DOWN_STR = "C_DOWN"
            C_LEFT   = 3
            C_LEFT_STR = "C_LEFT"
            C_RIGHT  = 8
            C_RIGHT_STR = "C_RIGHT"
            L        = 4
            L_STR    = "L"
            R        = 5
            R_STR    = "R"
            Z        = 6
            Z_STR    = "Z"
            START    = 12
            START_STR = "START"
            DP_UP    = 20
            DP_UP_STR = "DP_UP"
            DP_DOWN  = 21
            DP_DOWN_STR = "DP_DOWN"
            DP_LEFT  = 22
            DP_LEFT_STR = "DP_LEFT"
            DP_RIGHT = 23
            DP_RIGHT_STR = "DP_RIGHT"

        class JOYPAD:
            UP = (0, 1)
            DOWN = (0, -1)
            LEFT = (-1, 0)
            RIGHT = (1, 0)


class MessageFormats:
    # Heartbeat consists of 1 byte identifier + 2 bytes timestamp
    class HEARTBEAT:
        NAME = "HEARTBEAT"
        ID = MessageIDs.HEARTBEAT_ID

        INTERVAL = 1_000_000_000  # 1 second heartbeat interval

    class QUIT:
        NAME = "QUIT"
        ID = MessageIDs.QUIT_ID

        # signal to indicate quitting
        DATA_TYPE = DATA_TYPES.BOOLEAN
        VALUE = 1

    class XBOX:
        # number of processed buttons and axes
        NAME = "XBOX"
        ID = MessageIDs.XBOX_ID
        NUM_AXES = 6
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
            NEUTRAL_HEX = b'\x64'
            NEUTRAL_INT   = 100
            NEUTRAL_FLOAT = 0.0
            MAX_VALUE = 200

            AXIS_LX = 0
            AXIS_LX_STR = "AXIS_LX"
            AXIS_LY = 1
            AXIS_LY_STR = "AXIS_LY"
            AXIS_RX = 2
            AXIS_RX_STR = "AXIS_RX"
            AXIS_RY = 3
            AXIS_RY_STR = "AXIS_RY"

        # these are treated like buttons for transfer msgs but are classified as axis
        class TRIGGER:
            AXIS_LT = 5
            AXIS_LT_STR = "AXIS_LT"
            AXIS_RT = 4
            AXIS_RT_STR = "AXIS_RT"

        class BUTTON:
            SIZE_BUTTON_IN_BITS = 2
            NUM_BUTTONS_PER_BYTE = 8 / SIZE_BUTTON_IN_BITS

            # I choose 2 to represent ON b/c it equals the bit value of 10
            # this means if it error and one of the bit was flipped then it would ignore it.
            # this would make it so 2 bit would need to be changed to produce the wrong result
            ON           = 2
            OFF          = 1

            A            = 0
            A_STR       = "A"
            B            = 1
            B_STR       = "B"
            X            = 2
            X_STR       = "X"
            Y            = 3
            Y_STR       = "Y"
            LEFT_BUMPER  = 4
            LEFT_BUMPER_STR = "LEFT_BUMPER"
            RIGHT_BUMPER = 5
            RIGHT_BUMPER_STR = "RIGHT_BUMPER"
            SELECT       = 6
            SELECT_STR   = "SELECT"
            START        = 7
            START_STR    = "START"
            LEFT_STICK   = 9
            LEFT_STICK_STR = "LEFT_STICK"
            RIGHT_STICK  = 10
            RIGHT_STICK_STR = "RIGHT_STICK" 
            HOME         = 8
            HOME_STR     = "HOME"

        class JOYPAD:
            UP = (0, 1)
            DOWN = (0, -1)
            LEFT = (-1, 0)
            RIGHT = (1, 0)

    class N64:
        NAME = "n64"
        NUM_AXES = 2
        NUM_USED_AXES = 0
        NUM_TRIGGER = 0
        NUM_BUTTONS = 14

        class INPUT_TYPE:
            IS_BUTTON = 0
            IS_AXIS = 1
            IS_TRIGGER = 2

        class JOYSTICK:
            MIN_VALUE = 0
            NEUTRAL_HEX = b'\x64'
            NEUTRAL_INT   = 100
            MAX_VALUE = 200

            AXIS_X = 0
            AXIS_X_STR = "AXIS_X"
            AXIS_Y = 1
            AXIS_Y_STR = "AXIS_Y"

        class BUTTON:
            SIZE_BUTTON_IN_BITS = 2
            NUM_BUTTONS_PER_BYTE = 8 / SIZE_BUTTON_IN_BITS

            ON       = 2
            OFF      = 1

            A        = 1
            A_STR    = "A"
            B        = 2
            B_STR    = "B"
            C_UP     = 9
            C_UP_STR = "C_UP"
            C_DOWN   = 0
            C_DOWN_STR = "C_DOWN"
            C_LEFT   = 3
            C_LEFT_STR = "C_LEFT"
            C_RIGHT  = 8
            C_RIGHT_STR = "C_RIGHT"
            L        = 4
            L_STR    = "L"
            R        = 5
            R_STR    = "R"
            Z        = 6
            Z_STR    = "Z"
            START    = 12
            START_STR = "START"
            DP_UP    = 20
            DP_UP_STR = "DP_UP"
            DP_DOWN  = 21
            DP_DOWN_STR = "DP_DOWN"
            DP_LEFT  = 22
            DP_LEFT_STR = "DP_LEFT"
            DP_RIGHT = 23
            DP_RIGHT_STR = "DP_RIGHT"

        class JOYPAD:
            UP = (0, 1)
            DOWN = (0, -1)
            LEFT = (-1, 0)
            RIGHT = (1, 0)


# basestation_messages = { # Note: dictationaries are ordered in Python 3.7+
#     COMPACT_MESSAGES.HEARTBEAT_ID: # byte 0
#     {
#         "name": HEARTBEAT.NAME,
#         "values": {
#                     # byte 1-2
#                     HEARTBEAT.TIMESTAMP_MESSAGE: Signal(COMPACT_MESSAGES.UINT_16)} # bits 0-15
#     },
#     COMPACT_MESSAGES.N64_ID: { # byte 0
#         "name": N64.NAME,
#         "values": {
#                     # byte 1
#                     N64.BUTTON.A_STR:        Signal(COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 0-1
#                     N64.BUTTON.B_STR:        Signal(COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 2-3
#                     N64.BUTTON.L_STR:        Signal(COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 4-5
#                     N64.BUTTON.R_STR:        Signal(COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 6-7

#                     # byte 2
#                     N64.BUTTON.C_UP_STR:     Signal(COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 0-1
#                     N64.BUTTON.C_DOWN_STR:   Signal(COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 2-3
#                     N64.BUTTON.C_LEFT_STR:   Signal(COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 4-5
#                     N64.BUTTON.C_RIGHT_STR:  Signal(COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 6-7

#                     # byte 3
#                     N64.BUTTON.DP_UP_STR:    Signal(COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 0-1
#                     N64.BUTTON.DP_DOWN_STR:  Signal(COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 2-3
#                     N64.BUTTON.DP_LEFT_STR:  Signal(COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 4-5
#                     N64.BUTTON.DP_RIGHT_STR: Signal(COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 6-7

#                     # byte 4
#                     N64.BUTTON.Z_STR:        Signal(COMPACT_MESSAGES.UINT_2_BOOL, False)} # bits 0-1
#     },
#     COMPACT_MESSAGES.XBOX_ID: { # byte 0
#         "name": XBOX.NAME,
#         "values": {
#                     # byte 1
#                     XBOX.JOYSTICK.AXIS_LY_STR:    Signal(COMPACT_MESSAGES.UINT_8_JOYSTICK, XBOX.JOYSTICK.NEUTRAL_FLOAT), # bits 0-7

#                     # byte 2
#                     XBOX.JOYSTICK.AXIS_RY_STR:    Signal(COMPACT_MESSAGES.UINT_8_JOYSTICK, XBOX.JOYSTICK.NEUTRAL_FLOAT), # bits 0-7

#                     # byte 3
#                     XBOX.BUTTON.A_STR:            Signal(COMPACT_MESSAGES.UINT_2_BOOL, False), # bit 0-1
#                     XBOX.BUTTON.B_STR:            Signal(COMPACT_MESSAGES.UINT_2_BOOL, False), # bit 2-3
#                     XBOX.BUTTON.X_STR:            Signal(COMPACT_MESSAGES.UINT_2_BOOL, False), # bit 4-5
#                     XBOX.BUTTON.Y_STR:            Signal(COMPACT_MESSAGES.UINT_2_BOOL, False), # bit 6-7

#                     # byte 4
#                     XBOX.BUTTON.LEFT_BUMPER_STR:  Signal(COMPACT_MESSAGES.UINT_2_BOOL, False), # bit 0-1
#                     XBOX.BUTTON.RIGHT_BUMPER_STR: Signal(COMPACT_MESSAGES.UINT_2_BOOL, False), # bit 2-3
#                     XBOX.TRIGGER.AXIS_LT_STR:     Signal(COMPACT_MESSAGES.UINT_2_BOOL, False), # bit 4-5
#                     XBOX.TRIGGER.AXIS_RT_STR:     Signal(COMPACT_MESSAGES.UINT_2_BOOL, False)} # bit 6-7
#     },
#     COMPACT_MESSAGES.QUIT_ID: { # byte 0
#         "name": QUIT.NAME,
#         "values": {
#                     # byte 1
#                     QUIT.NAME: Signal(COMPACT_MESSAGES.BOOLEAN, QUIT.VALUE)}, # bit 0
#     }
# }

# class COMPACT_MESSAGES:
        # Reserved message IDs (DO NOT USE)
        # CONTROLLER_DATA = int(0xDE)  # START_MESSAGE
        # N64_ID = int(0xDF)   # The Id for the N64
        # XBOX_ID = int(0xF0)  # The Id for the Xbox
        # QUIT_ID = int(0xFE)  # QUIT_MESSAGE
        # HEARTBEAT_ID = int(0xAA)  # Heartbeat

        # Available message IDs for custom messages
        # STATUS = 0xB0  # System status update
        # ERROR = 0xE0  # Error codes
        # GPS = 0xC0  # GPS position data
        # SENSOR = 0xD0  # Sensor readings


# class COMMUNICATION:
#     # Communication settings
#     DEFAULT_PORT = "COM9"
#     DEFAULT_BAUD_RATE = 230400
#     FALLBACK_BAUD_RATE = 921600
#     REMOTE_XBEE_ADDRESS = "0013A200423A7DDD"
    
#     # UDP settings for simulation mode
#     UDP_HOST = "127.0.0.1" # localhost
#     UDP_BASESTATION_PORT = 5000 # Port for basestation to send from
#     UDP_ROVER_PORT = 5005 # Port to send rover commands to