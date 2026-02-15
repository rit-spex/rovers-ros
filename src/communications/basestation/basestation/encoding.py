from typing import Any
from command_codes import CONSTANTS

class Signal:
    """Class representing a signal with a name and type."""
    def __init__(self, type: CONSTANTS.COMPACT_MESSAGES.DATA_TYPE, default_value: Any = 0): # TODO: allow it to support it native types
        self.__type = type
        self.__default_value = default_value
        self.__value = self.__default_value
    @property
    def get_type(self) -> CONSTANTS.COMPACT_MESSAGES.DATA_TYPE:
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

class BaseStationCommunication:

    """Module for encoding and decoding data for XBee communication."""
    def __init__(self):
        self.__messages = { # Note: dictationaries are ordered in Python 3.7+
            CONSTANTS.COMPACT_MESSAGES.HEARTBEAT_ID: # byte 0
            {
                "name": CONSTANTS.HEARTBEAT.NAME,
                "values": {
                            # byte 1-2
                            CONSTANTS.HEARTBEAT.TIMESTAMP_MESSAGE: Signal(CONSTANTS.COMPACT_MESSAGES.UINT_16)} # bits 0-15
            },
            CONSTANTS.COMPACT_MESSAGES.N64_ID: { # byte 0
                "name": CONSTANTS.N64.NAME,
                "values": {
                            # byte 1
                            CONSTANTS.N64.BUTTON.A_STR:        Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 0-1
                            CONSTANTS.N64.BUTTON.B_STR:        Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 2-3
                            CONSTANTS.N64.BUTTON.L_STR:        Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 4-5
                            CONSTANTS.N64.BUTTON.R_STR:        Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 6-7

                            # byte 2
                            CONSTANTS.N64.BUTTON.C_UP_STR:     Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 0-1
                            CONSTANTS.N64.BUTTON.C_DOWN_STR:   Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 2-3
                            CONSTANTS.N64.BUTTON.C_LEFT_STR:   Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 4-5
                            CONSTANTS.N64.BUTTON.C_RIGHT_STR:  Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 6-7

                            # byte 3
                            CONSTANTS.N64.BUTTON.DP_UP_STR:    Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 0-1
                            CONSTANTS.N64.BUTTON.DP_DOWN_STR:  Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 2-3
                            CONSTANTS.N64.BUTTON.DP_LEFT_STR:  Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 4-5
                            CONSTANTS.N64.BUTTON.DP_RIGHT_STR: Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bits 6-7

                            # byte 4
                            CONSTANTS.N64.BUTTON.Z_STR:        Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False)} # bits 0-1
            },
            CONSTANTS.COMPACT_MESSAGES.XBOX_ID: { # byte 0
                "name": CONSTANTS.XBOX.NAME,
                "values": {
                            # byte 1
                            CONSTANTS.XBOX.JOYSTICK.AXIS_LY_STR:    Signal(CONSTANTS.COMPACT_MESSAGES.UINT_8_JOYSTICK, CONSTANTS.XBOX.JOYSTICK.NEUTRAL_FLOAT), # bits 0-7

                            # byte 2
                            CONSTANTS.XBOX.JOYSTICK.AXIS_RY_STR:    Signal(CONSTANTS.COMPACT_MESSAGES.UINT_8_JOYSTICK, CONSTANTS.XBOX.JOYSTICK.NEUTRAL_FLOAT), # bits 0-7

                            # byte 3
                            CONSTANTS.XBOX.BUTTON.A_STR:            Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bit 0-1
                            CONSTANTS.XBOX.BUTTON.B_STR:            Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bit 2-3
                            CONSTANTS.XBOX.BUTTON.X_STR:            Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bit 4-5
                            CONSTANTS.XBOX.BUTTON.Y_STR:            Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bit 6-7

                            # byte 4
                            CONSTANTS.XBOX.BUTTON.LEFT_BUMPER_STR:  Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bit 0-1
                            CONSTANTS.XBOX.BUTTON.RIGHT_BUMPER_STR: Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bit 2-3
                            CONSTANTS.XBOX.TRIGGER.AXIS_LT_STR:     Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bit 4-5
                            CONSTANTS.XBOX.TRIGGER.AXIS_RT_STR:     Signal(CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL, False), # bit 6-7

                            # byte 5
                            CONSTANTS.XBOX.MODES.CONTROL_MODE_STR:  Signal(CONSTANTS.COMPACT_MESSAGES.UINT_8, 100)} # bits 0-7
            },
            CONSTANTS.COMPACT_MESSAGES.QUIT_ID: { # byte 0
                "name": CONSTANTS.QUIT.NAME,
                "values": {
                            # byte 1
                            CONSTANTS.QUIT.NAME: Signal(CONSTANTS.COMPACT_MESSAGES.BOOLEAN, CONSTANTS.QUIT.VALUE)}, # bit 0
            }
        }

    def get_messages(self) -> dict[int, dict[str, Any]]:
        """
        Get the message definitions.

        Returns:
            dict: The message definitions.
        """
        return self.__messages

    def __convert_native_to_int(self, type: CONSTANTS.COMPACT_MESSAGES.DATA_TYPE, value: Any) -> int:
        """
        Convert a native type to an integer representation.

        Args:
            type (int): The native type.

        Returns:
            int: The integer representation.
        """
        if type == CONSTANTS.COMPACT_MESSAGES.BOOLEAN:
            return value # convert bool to 1/2 for 2 bit storage

        elif type == CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL:
            return value+1 # convert bool to 1/2 for 2 bit storage

        elif type == CONSTANTS.COMPACT_MESSAGES.UINT_8_JOYSTICK:
            # convert float joystick value (-1.0-1.0) to int (0-200)
            value = max(-1.0, min(1.0, value))
            value = value * 100 + 100
            return int(value)

        else:
            return int(value)
        
    def __convert_int_to_native(self, type: CONSTANTS.COMPACT_MESSAGES.DATA_TYPE, value: int) -> Any:
        """
        Convert an integer representation to a native type.

        Args:
            type (int): The native type.

        Returns:
            Any: The native representation.
        """
        if type == CONSTANTS.COMPACT_MESSAGES.BOOLEAN:
            return bool(value) # convert 1/2 back to bool

        elif type == CONSTANTS.COMPACT_MESSAGES.UINT_2_BOOL:
            return bool(value - 1) # convert 1/2 back to bool

        elif type == CONSTANTS.COMPACT_MESSAGES.UINT_8_JOYSTICK:
            # convert int (0-200) back to float joystick value (-1.0-1.0)
            return (float(value) - 100.0) / 100.0

        else:
            return value


    def encode_data(self, data: dict[str, Any], ID: int) -> bytes:
        """
        Encode the given data dictionary into bytes for transmission.

        Args:
            data (dict): The data to encode.
            ID (int): The identifier for the data.

        Returns:
            bytes: The encoded data.
        """

        bytes_data = b''

        if ID is bytes:
            ID = int.from_bytes(ID, byteorder='big')

        # the first byte is always the ID
        bytes_data += ID.to_bytes(1, byteorder='big')

        # Calculate the number of bits used for current byte
        bitsRemaining = 8
        current_byte = 0

        # Implement encoding logic here
        for key, signal in self.__messages[ID]["values"].items():

            # store how many bits we need to store for this key
            signal_bits_size = signal.get_type.num_bits

            # default the value to the signal's default value
            signal_value = signal.default_value

            if key in data:
                if isinstance(data[key], bytes):
                    signal_value = int.from_bytes(data[key], byteorder='big')
                else:
                    signal_value = data[key]
            
            # convert to int representation
            signal_value = self.__convert_native_to_int(signal.get_type, signal_value)

            # repeat in case of 16 bit values or larger
            while(signal_bits_size > 0):
                if bitsRemaining - signal_bits_size < 0:
                    # use the remaining bits in the current byte
                    signal_bits_size -= bitsRemaining
                    current_byte |= (signal_value >> (signal_bits_size)) & ((1 << bitsRemaining) - 1)
                    bytes_data += current_byte.to_bytes(1, byteorder='big')
                    bitsRemaining = 8
                    current_byte = 0
                else:
                    # the signal fits in the current byte
                    current_byte |= (signal_value & ((1 << signal_bits_size) - 1)) << (bitsRemaining - signal_bits_size)
                    bitsRemaining -= signal_bits_size
                    signal_value = signal_value >> signal_bits_size
                    signal_bits_size = 0

                    # reset the byte if full
                    if bitsRemaining == 0:
                        bytes_data += current_byte.to_bytes(1, byteorder='big')
                        bitsRemaining = 8
                        current_byte = 0

        # if there are remaining bits in the current byte, add it
        if bitsRemaining < 8:
            bytes_data += current_byte.to_bytes(1, byteorder='big')


        return bytes_data


    def decode_data(self, data: bytes) -> tuple[dict[str, Any], int]:
        """
        Decode the received bytes into a data dictionary.

        Args:
            data (bytes): The data to decode.

        Returns:
            dict: The decoded data.
            int: The identifier for the data.
        """

        # the first byte is always the ID
        ID = int.from_bytes(data[0:1], byteorder='big')

        # start decoding from byte index 1
        byte_index = 1
        bitsRemaining = 8

        # Implement decoding logic here
        decoded_data = {}

        # Implement encoding logic here
        for key, signal in self.__messages[ID]["values"].items():

            # store how many bits we need to store for this key
            signal_bits_size = signal.get_type.num_bits

            # set the value to zero for parsing
            signal_value = 0

            # repeat in case of 16 bit values or larger
            while(signal_bits_size > 0):
                if bitsRemaining - signal_bits_size < 0:
                    # use the remaining bits in the current byte
                    signal_bits_size -= bitsRemaining
                    signal_value <<= bitsRemaining
                    signal_value |= int.from_bytes(data[byte_index:byte_index + 1], byteorder='big') & ((1 << bitsRemaining) - 1)
                    byte_index += 1
                    bitsRemaining = 8
                else:
                    # the signal fits in the current byte
                    signal_value <<= signal_bits_size
                    signal_value |= (int.from_bytes(data[byte_index:byte_index + 1], byteorder='big') >> (bitsRemaining - signal_bits_size)) & ((1 << signal_bits_size) - 1)
                    bitsRemaining -= signal_bits_size
                    signal_bits_size = 0

                    # reset the byte if full
                    if bitsRemaining == 0:
                        bitsRemaining = 8
                        byte_index += 1

            # convert back to native type
            signal_value = self.__convert_int_to_native(signal.get_type, signal_value)
            decoded_data[key] = signal_value

        return decoded_data, ID


if __name__ == "__main__":
    comm = BaseStationCommunication()

    # test_data = { # byte 0
    #                 "ly": 100, # byte 1
    #                 "ry": 100, # byte 2
    #                 "A": 2, "B": 1, "X": 2, "Y": 1, # byte 3
    #                 "LB": 2, "RB": 1, "LT": 2, "RT": 1 # byte 4
    #         }

    # test invalid data
    test_data = {}

    encoded = comm.encode_data(test_data, CONSTANTS.COMPACT_MESSAGES.XBOX_ID)
    print(f"Encoded Data: {encoded}, ID: {hex(CONSTANTS.COMPACT_MESSAGES.XBOX_ID)}")

    decoded, ID = comm.decode_data(encoded)
    print(f"Decoded Data: {decoded}, ID: {hex(ID)}")