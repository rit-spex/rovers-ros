class CONSTANTS:
    # Set to True to run simulator with UDP, False for actual XBee communication
    SIMULATION_MODE = True  # Change this to False when trying to communicate with real rover
    
    # message to send to show start of new values
    START_MESSAGE = b'\xDE'
    QUIT_MESSAGE = b'\xFE'

    # message order:
    # byte Axis
    # 2 bits Trigger
    # 2 bits Buttons

    class CONVERSION:
        NS_PER_MS = 1_000_000
        NS_PER_S = 1_000_000_000
        
        ONE_HUNDRED_MS_TO_NS = 100_000_000
        FIVE_HUNDRED_MS_TO_NS = 500_000_000
    
    class HEARTBEAT:
        NAME = "heartbeat"

        MESSAGE = b'\xAA'  # Heartbeat signal identifier
        MESSAGE_LENGTH = 3  # Length of heartbeat message in bytes

        TIMESTAMP_MESSAGE = "timestamp"

        INTERVAL = 1_000_000_000  # 1 second heartbeat interval
        # Heartbeat consists of 1 byte identifier + 2 bytes timestamp
    
    class COMPACT_MESSAGES:
        # Reserved message IDs (DO NOT USE)
        CONTROLLER_DATA = int(0xDE)  # START_MESSAGE
        N64_ID = int(0xDF)   # The Id for the N64
        XBOX_ID = int(0xF0)  # The Id for the Xbox
        QUIT_ID = int(0xFE)  # QUIT_MESSAGE
        HEARTBEAT_ID = int(0xAA)  # Heartbeat

        class DATA_TYPE:
            def __init__(self, num_bits: int, id: int) -> None:
                self.num_bits = num_bits
                self.id = id

        # enum of data types in bits
        UINT_2_BOOL     = DATA_TYPE(2,  0x01)
        UINT_8          = DATA_TYPE(8,  0x02)
        UINT_16         = DATA_TYPE(16, 0x03)
        UINT_8_JOYSTICK = DATA_TYPE(8,  0x04) # convert to float on decoding
        BOOLEAN         = DATA_TYPE(1,  0x05)

        # Available message IDs for custom messages
        STATUS = 0xB0  # System status update
        ERROR = 0xE0  # Error codes
        GPS = 0xC0  # GPS position data
        SENSOR = 0xD0  # Sensor readings
        
        # Add your own here! Pick any unused byte value (0x00-0xFF)
        # Examples:
        # CAMERA_CONTROL = 0xCA
        # ARM_POSITION = 0xA0
        # SCIENCE_DATA = 0xDD
        # DRIVE_MODE = 0xDB
    
    class TIMING:
        # Timing constants for various operations (in nanoseconds)
        UPDATE_FREQUENCY = 40_000_000  # 40ms update frequency
        DEADBAND_THRESHOLD = 0.10  # Controller deadband threshold
    
    class COMMUNICATION:
        # Communication settings
        DEFAULT_PORT = "COM9"
        DEFAULT_BAUD_RATE = 230400
        FALLBACK_BAUD_RATE = 921600
        REMOTE_XBEE_ADDRESS = "0013A200423A7DDD"
        
        # UDP settings for simulation mode
        UDP_HOST = "127.0.0.1" # localhost
        UDP_BASESTATION_PORT = 5000 # Port for basestation to send from
        UDP_ROVER_PORT = 5005 # Port to send rover commands to
        UDP_TELEMETRY_PORT = 5002 # Port to receive telemetry data

    class CONTROLLER_MODES:
        # Controller mode multipliers
        NORMAL_MULTIPLIER = 100
        CREEP_MULTIPLIER = 20
        REVERSE_MULTIPLIER = -100  # Will be applied to normal/creep multiplier

    class QUIT:
        NAME = "QUIT"
        VALUE = 1

    class XBOX:
        # number of processed buttons and axes
        NAME = "XBOX"
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

            
        class MODES:
            CONTROL_MODE = 100
            CONTROL_MODE_STR = "CONTROL_MODE"

            CONTROL_MODE_MEANING = ["MANUAL_MODE", "GPS_MODE", "ARCUO_MODE", "OBJDECT_MODE", "KEYB_MODE"]

            MANUAL_MODE  = 101
            MANUAL_MODE_STR = "MANUAL_MODE"
            GPS_MODE     = 102
            GPS_MODE_STR = "GPS_MODE"
            ARCUO_MODE   = 103
            ARCUO_MODE_STR = "ARCUO_MODE"
            OBJDECT_MODE = 104
            OBJDECT_MODE_STR = "OBJDECT_MODE"
            KEYB_MODE    = 105
            KEYB_MODE_STR = "KEYB_MODE"

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

