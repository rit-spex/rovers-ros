from typing import Any
from constants.CAN_constants import TEENSY_CAN_MESSAGES, ODRIVE_CAN_MESSAGES, DATA_TYPES
from constants.CAN_enums import CAN_MESSAGE_IDS, ODRIVE_MESSAGE_IDS
from constants.CAN_structs import Message, DATA_TYPE
from custom_interfaces.msg import Can

import struct
import ctypes

class TeensyCommunication:

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

        signal_values = []
        pack_format = "="

        # Combine the signals together into one list of values
        for (signal_id, signal) in CANMessage.signals.items():
            signal_values.append(signal.value)
            pack_format += signal.type.pack_format

        # create a buffer to pack into
        bufTemp = ctypes.create_string_buffer(8)

        # pack the values into the buffer
        struct.pack_into(pack_format, bufTemp, 0, *signal_values)

        # enter into the can packet
        for x in range(0, 8):
            result_can.buf[x] = int.from_bytes(bufTemp[x], byteorder='big')

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

        signal_values = []
        pack_format = "="

        # Combine the signals together into one list of values
        for (signal_id, signal) in result_message.signals.items():
            signal_values.append(signal.value)
            pack_format += signal.type.pack_format

        unpacked_list = list(struct.unpack_from(pack_format, bytes(CANPacket.buf), 0))

        signal_index = 0
        for (signal_id, signal) in result_message.signals.items():
            signal.set_value(unpacked_list[signal_index])
            signal_index += 1

        return result_message


class OdriveCommunication:

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

        signal_values = []
        pack_format = "="

        # Combine the signals together into one list of values
        for (signal_id, signal) in CANMessage.signals.items():
            signal_values.append(signal.value)
            pack_format += signal.type.pack_format

        # create a buffer to pack into
        bufTemp = ctypes.create_string_buffer(8)

        # pack the values into the buffer
        struct.pack_into(pack_format, bufTemp, 0, *signal_values)

        # enter into the can packet
        for x in range(0, 8):
            result_can.buf[x] = int.from_bytes(bufTemp[x], byteorder='little')

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
        result_message = ODRIVE_CAN_MESSAGES[ODRIVE_MESSAGE_IDS(CANPacket.id)]

        # If no matching message for the id is found end parsing
        if result_message is None:
            return None

        signal_values = []
        pack_format = "="

        # Combine the signals together into one list of values
        for (signal_id, signal) in result_message.signals.items():
            signal_values.append(signal.value)
            pack_format += signal.type.pack_format

        unpacked_list = list(struct.unpack_from(pack_format, bytes(CANPacket.buf), 0))

        signal_index = 0
        for (signal_id, signal) in result_message.signals.items():
            signal.set_value(unpacked_list[signal_index])
            signal_index += 1

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


    comm_o = OdriveCommunication()

    # test packing routine
    test_data = ODRIVE_CAN_MESSAGES[ODRIVE_MESSAGE_IDS.FRONT_L_SET_VEL]

    test_data.signals["cmd_vel"].set_value(1.0)

    CANPacket = comm.encode_can_message(test_data)

    print(f"Encoded Data: {CANPacket}, ID: {test_data.id}")

    CANMessage = comm.decode_can_packet(CANPacket)

    if(CANMessage is not None):
        print(f"Decoded Data: {CANMessage.toString()}, ID: {CANMessage.id}")