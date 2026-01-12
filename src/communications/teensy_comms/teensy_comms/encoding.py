from typing import Any
from constants.CAN_Constants import Signal, TEENSY_CAN_MESSAGES, Message

class TeensyCommunication:

    """Module for encoding and decoding data for Teensy communication."""


    def encode_data(self, data: Message) -> bytes:
        """
        Encode the given data dictionary into bytes for transmission.

        Args:
            data (dict): The data to encode.
            ID (int): The identifier for the data.

        Returns:
            bytes: The encoded data.
        """

        bytes_data = b''

        if Message.id is bytes:
            ID = int.from_bytes(ID, byteorder='big')

        # the first byte is always the ID
        bytes_data += ID.to_bytes(1, byteorder='big')

        # Calculate the number of bits used for current byte
        bitsRemaining = 8
        current_byte = 0

        # Implement encoding logic here
        for key, signal in TEENSY_CAN_MESSAGES[ID].signals.items():

            # store how many bits we need to store for this key
            signal_bits_size = signal.type.num_bits

            # default the value to the signal's default value
            signal_value = signal.default_value

            if key in data:
                if isinstance(data[key], bytes):
                    signal_value = int.from_bytes(data[key], byteorder='big')
                else:
                    signal_value = data[key]
            
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