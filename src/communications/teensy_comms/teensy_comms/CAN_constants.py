class CONVERSION:
    NS_PER_MS = 1_000_000
    NS_PER_S = 1_000_000_000
    
    ONE_HUNDRED_MS_TO_NS = 100_000_000
    FIVE_HUNDRED_MS_TO_NS = 500_000_000

class CAN_CONSTANTS:
    
    class DATA_TYPE:
        def __init__(self, num_bits: int, id: int) -> None:
            self.num_bits = num_bits
            self.id = id

    BOOL            = DATA_TYPE(2,  1)
    UINT_8          = DATA_TYPE(8,  2)
    UINT_16         = DATA_TYPE(16, 3)
    BOOLEAN         = DATA_TYPE(1,  4)

    class GENERIC:
        EStop     = 1
        Heartbeat = 2
    class CHASSIS:
        ENABLE      = 10
        TARGET_RPM  = 11
        CURRENT_RPM = 12
        DRIVE_POWER = 13
    class ARM:
        ENABLE = 20
        MOVE_BASE = 21
        MOVE_SHOULDER = 22
        MOVE_ELBOW = 23
        BEND_WRIST = 24
        TWIST_WRIST = 25
        MOVE_CLAW = 26
        MOVE_SOLENOID = 27
        BASE_POSITION = 28

    class SCIENCE:
        ENABLE = 30
        MOVE_AUGER = 31
        HOME_AUGER = 32
        ENABLE_DRILL = 33
        MOVE_SLIDE = 34

    class COMMUNICATION:
        # UDP settings for simulation mode
        UDP_HOST = "127.0.0.1" # localhost
        UDP_BASESTATION_PORT = 5000 # Port for basestation to send from
        UDP_ROVER_PORT = 5005 # Port to send rover commands to
