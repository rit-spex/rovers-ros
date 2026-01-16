from typing import Any
from constants.CAN_constants import TEENSY_CAN_MESSAGES
from constants.CAN_enums import CAN_MESSAGE_IDS
from constants.CAN_structs import Message, DATA_TYPE
from custom_interfaces.msg import Can

#from CAN_constants import CAN_CONSTANTS
class TeensyCommunication:

    @staticmethod
    def __convert_native_to_int(type: DATA_TYPE, value: Any) -> int:
        """
        Convert a native type to an integer representation.

        Args:
            type (int): The native type.

        Returns:
            int: The integer representation.
        """
        return int(value)

    @staticmethod
    def __convert_int_to_native(type: DATA_TYPE, value: int) -> Any:
        """
        Convert an integer representation to a native type.

        Args:
            type (int): The native type.

        Returns:
            Any: The native representation.
        """
        # if type == DATA_TYPES.BOOLEAN:
        #     return bool(value) # convert 1/2 back to bool

        # elif type == DATA_TYPES.UINT_2_BOOL:
        #     return bool(value - 1) # convert 1/2 back to bool

        # elif type == DATA_TYPES.UINT_8_JOYSTICK:
        #     # convert int (0-200) back to float joystick value (-1.0-1.0)
        #     return (float(value) - 100.0) / 100.0

        # else:
        return value


    @staticmethod
    def encode_can_message(CANMessage: Message | None) -> Can | None:
        """
        Encode the given data dictionary into bytes for transmission.

        Args:
            CANMessage (Message): The CAN message to encode.

        Returns:
            Can | None: The encoded CAN message or None if encoding failed.
        """

    
        if CANMessage is None:
            return None

        result_can = Can()

        # assign the id
        result_can.id = CANMessage.id

        # Calculate the number of bits used for current byte that will are fill currently
        bitsRemaining = 8
        current_byte = 0

        # the byte index
        byte_index = 0

        # Implement encoding logic here
        for signal_name, signal in CANMessage.signals.items():

            # store how many bits we need to store for this key
            signal_bits_size = signal.type.num_bits
            
            # this is value we can modify
            signal_value = signal.value
            
            # repeat in case of 16 bit values or larger
            while(signal_bits_size > 0):
                if bitsRemaining - signal_bits_size < 0:
                    # use the remaining bits in the current byte
                    signal_bits_size -= bitsRemaining
                    current_byte |= (signal_value >> (signal_bits_size)) & ((1 << bitsRemaining) - 1)
                    result_can.buf[byte_index] = current_byte
                    byte_index += 1
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
                        result_can.buf[byte_index] = current_byte
                        byte_index += 1
                        bitsRemaining = 8
                        current_byte = 0

        # if there are remaining bits in the current byte, add it
        if bitsRemaining < 8:
            result_can.buf[byte_index] = current_byte
            byte_index += 1

        return result_can

    @staticmethod
    def decode_can_packet(CANPacket: Can | None) -> Message | None:
        """
        Decode the given CAN packet into a Message.

        Args:
            CANPacket (Can | None): The CAN packet to decode.

        Returns:
            Message | None: The decoded message or None if decoding failed.
        """

        # Check if the CANPacket is valid
        if CANPacket is None:
            return None

        # Find what message the can packet is for
        result_message = TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS(CANPacket.id)]

        # If no matching message for the id is found end parsing
        if result_message is None:
            return None

        # Calculate the number of bits used for current byte of the data section
        byte_index = 0
        bitsRemaining = 8

        # Implement encoding logic here
        for signal_name, signal in result_message.signals.items():

            # store how many bits we need to store for this signal
            signal_bits_size = signal.type.num_bits

            # set the value to zero for parsing
            signal_value = 0

            # repeat in case of 16 bit values or larger
            while(signal_bits_size > 0):
                if bitsRemaining - signal_bits_size < 0:
                    # use the remaining bits in the current byte
                    signal_bits_size -= bitsRemaining
                    signal_value <<= bitsRemaining
                    signal_value |= (CANPacket.buf[byte_index] & ((1 << bitsRemaining) - 1))
                    byte_index += 1
                    bitsRemaining = 8
                else:
                    # the signal fits in the current byte
                    signal_value <<= signal_bits_size
                    signal_value |= ((CANPacket.buf[byte_index] >> (bitsRemaining - signal_bits_size)) & ((1 << signal_bits_size) - 1))
                    bitsRemaining -= signal_bits_size
                    signal_bits_size = 0

                    # reset the byte if full
                    if bitsRemaining == 0:
                        bitsRemaining = 8
                        byte_index += 1

            # convert back to native type
            signal.set_value(TeensyCommunication.__convert_int_to_native(signal.type, signal_value))

        return result_message


if __name__ == "__main__":
    comm = TeensyCommunication()

    # test packing routine
    test_data = TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS.E_STOP]

    test_data.signals["E_STOP"].set_value(1)

    CANPacket = comm.encode_can_message(test_data)

    print(f"Encoded Data: {CANPacket}, ID: {test_data.id}")

    CANMessage = comm.decode_can_packet(CANPacket)

    if(CANMessage is not None):
        print(f"Decoded Data: {CANMessage.toString()}, ID: {CANMessage.id}")