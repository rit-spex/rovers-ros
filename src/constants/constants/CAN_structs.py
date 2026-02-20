# ------------------------------------------------------------------
#                          SPEX ROVER 2025
# ------------------------------------------------------------------
# file name     : CAN_structs.py
# purpose       : contains classes for can communication
#                   
# authors       : Tyler Halifax
# created on    : 1/14/2026 - Tyler
# last modified : 1/14/2026 - Tyler
# ------------------------------------------------------------------

from typing import Any
from constants.CAN_enums import CAN_MESSAGE_IDS

# New topic name base for ROS topics
RX_TOPIC_NAME = "CAN/RX"
TX_TOPIC_NAME = "CAN/TX"

# Define wrapper class for data types
class DATA_TYPE:
    def __init__(self, id: int, num_bits: int, pack_format: str) -> None:
        self.__num_bits = num_bits
        self.__id = id
        self.__pack_format = pack_format

    @property
    def id(self) -> int:
        """Get the ID of the data type."""
        return self.__id

    @property
    def num_bits(self) -> int:
        """Get the number of bits of the data type."""
        return self.__num_bits

    @property
    def pack_format(self) -> str:
        """Get the pack format of the data type."""
        return self.__pack_format

class Signal:
    """Class representing a signal with a name and type."""
    def __init__(self, type: DATA_TYPE, default_value: Any = 0):
        self.__type = type
        self.__default_value = default_value
        self.__value = self.__default_value

        # these values are meant to be updated by the message init
        self.__name = "invalid"
        self.__topic_src = "invalid"
    @property
    def type(self) -> DATA_TYPE:
        """Get the type of the signal."""
        return self.__type
    @property
    def value(self) -> Any:
        """Get the current value of the signal."""
        return self.__value
    @property
    def default_value(self) -> Any:
        """Get the default value of the signal."""
        return self.__default_value

    @property
    def name(self) -> str:
        """Get the name of the signal."""
        return self.__name

    @property
    def topic_src(self) -> str:
        """Get the topic source of the signal."""
        return self.__topic_src

    def set_value(self, new_value: Any) -> None:
        """Set the current value of the signal."""
        self.__value = new_value

    # This method is meant to be only called by the message init
    def set_name(self, new_name: str) -> None:
        """Set the name of the signal."""
        self.__name = new_name

    # This method is meant to be only called by the message init
    def set_topic_src(self, new_topic_src: str) -> None:
        """Set the topic source of the signal."""
        self.__topic_src = new_topic_src

    def reset(self) -> None:
        """Reset the signal to its default value."""
        self.__value = self.__default_value

    def toString(self) -> str:
        """Get the string representation of the signal."""
        return f"Signal(type={self.__type}, value={self.__value}, default_value={self.__default_value})"

# Class representing a full can message with an ID, name, and signals.
class Message:
    """Class representing a message with an ID, name, and signals."""
    def __init__(self, id: CAN_MESSAGE_IDS, name: str, subsystem: str, signals: dict[str, Signal], isforJetson: bool):
        self.__id = id
        self.__name = name
        self.__subsystem = subsystem
        self.__isforJetson = isforJetson
        self.__signals = signals

        # Calculate the topic name based on message direction
        if(isforJetson):
            self.__topic_name = RX_TOPIC_NAME + "/" + self.__subsystem + "/" + self.__name
        else:
            self.__topic_name = TX_TOPIC_NAME + "/" + self.__subsystem + "/" + self.__name

        # Update the signals the message source and name of the signals
        for (signal_name, signal) in self.__signals.items():
            signal.set_topic_src(self.__topic_name + "/" + signal_name)
            signal.set_name(signal_name)

    @property
    def id(self) -> CAN_MESSAGE_IDS:
        """Get the ID of the message."""
        return self.__id

    @property
    def name(self) -> str:
        """Get the name of the message."""
        return self.__name

    @property
    def signals(self) -> dict[str, Signal]:
        """Get the signals of the message."""
        return self.__signals
    
    @property
    def topic_name(self) -> str:
        """Get the topic name of the message."""
        return self.__topic_name

    @property
    def isforJetson(self) -> bool:
        """Get the sent status of the message."""
        return self.__isforJetson

    @property
    def subsystem(self) -> str:
        """Get the subsystem of the message."""
        return self.__subsystem

    def reset(self) -> None:
        """Reset the message to its default state."""
        for signal in self.__signals.values():
            signal.set_value(signal.default_value)

    def toString(self) -> str:
        """Get the string representation of the message."""
        return f"Message(id={self.__id}, name={self.__name}, subsystem={self.__subsystem}, signals={self.__signals}, isforJetson={self.__isforJetson})"
