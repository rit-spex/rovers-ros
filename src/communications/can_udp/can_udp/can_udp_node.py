import threading
import select
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import rclpy.subscription
import rclpy.publisher

from custom_interfaces.msg import Can

from constants.CAN_constants import (
    TEENSY_CAN_MESSAGES,
    CAN_MESSAGE_IDS,
    SAFE_STOP_MESSAGE_IDS,
    TEENSY_ESTOP_TOPIC,
)
from constants.CAN_structs import TX_TOPIC_NAME
from constants.can_encoding import TeensyCommunication

from std_msgs.msg import Bool

import socket as skt
from socket import socket
import errno

# this is used to package the message in certain formats
from struct import *


class CAN_UDP(Node):
    __address: str
    __recv_port: int
    __send_port: int
    __socket: socket
    __buffer_size: int

    # Stored as Publishers[message_id]
    __publishers: dict[int, rclpy.publisher.Publisher]

    # Outgoing CAN topic subscriptions. They stay registered during E-STOP;
    # send_msg() ignores commands while the latch is active.
    __can_tx_subscriptions: dict[CAN_MESSAGE_IDS, rclpy.subscription.Subscription]

    def __init__(self) -> None:
        super().__init__("CAN_udp_node")

        self.__address = "127.0.0.1"
        self.__recv_port = 8001
        self.__send_port = 8000
        self.__socket = socket(skt.AF_INET, skt.SOCK_DGRAM)
        self.__buffer_size = 1024
        self.__estop = False

        self.__publishers = {}
        self.__can_tx_subscriptions = {}

        # E_STOP is bidirectional on the same CAN ID. Incoming frames are
        # republished here so master_node can latch a Teensy-side E-STOP.
        self.__estop_publisher = self.create_publisher(
            Can, TEENSY_ESTOP_TOPIC, 10
        )

        for (id, message) in TEENSY_CAN_MESSAGES.items():
            if(message.isforJetson):
                # This is for messages coming in
                self.__publishers[id] = self.create_publisher(
                msg_type=Can,
                topic=message.topic_name,
                qos_profile=10
                )
            else:
                # This is for messages going out
                self.__can_tx_subscriptions[id] = self.create_subscription(
                        msg_type=Can,
                        topic=message.topic_name,
                        callback=self.send_msg,
                        qos_profile=10,
                    )
        
        self.create_subscription(
            msg_type=Bool,
            topic="/ESTOP",
            callback=self.__on_estop_received,
            qos_profile=10,
        )

        self.__stop = threading.Event()
        self.__receiver_thread = None

    def __on_estop_received(self, msg: Bool):
        # /ESTOP is level-triggered. Latch outbound commands while active,
        # send one round of safe defaults, and keep the node alive so it can
        # resume when E-STOP clears.
        if msg.data:
            if self.__estop:
                return
            self.__estop = True
            self.get_logger().info(
                "E-STOP received, sending safe CAN defaults")
            self.__send_safe_stop()
        else:
            if not self.__estop:
                return
            self.__estop = False
            self.__send_estop_clear()
            self.get_logger().info(
                "E-STOP cleared, CAN output re-enabled")

    def __send_safe_stop(self, include_estop: bool = True):
        """Send only outgoing messages whose defaults are safe stop values."""
        for message_id in SAFE_STOP_MESSAGE_IDS:
            if not include_estop and message_id == CAN_MESSAGE_IDS.E_STOP:
                continue
            message = TEENSY_CAN_MESSAGES[message_id]
            message.reset()
            can_packet = TeensyCommunication.encode_can_message(message)
            if can_packet is not None:
                self.__send_can(can_packet)

    def __send_estop_clear(self):
        """Tell the Teensy the E-STOP condition has cleared."""
        message = TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS.E_STOP]
        message.signals["E_STOP"].set_value(0)
        can_packet = TeensyCommunication.encode_can_message(message)
        if can_packet is not None:
            self.__send_can(can_packet)

    def on_message_received(self, msg: Can):
        try:
            if msg.id == CAN_MESSAGE_IDS.E_STOP:
                self.__estop_publisher.publish(msg)
                self.get_logger().info(
                    "Hardware E-STOP frame received from UDP")
                return
            self.get_logger().debug(f"Received ID {msg.id} ({TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS(msg.id)].name}): {msg.buf}")
            self.__publishers[msg.id].publish(msg)
        except Exception as e:
            self.get_logger().error(f"failed to publish can message: {e}")

    def send_msg(self, msg: Can):
        """Callback for outgoing CAN topics; ignored while E-STOP is latched."""
        if self.__estop:
            return
        self.__send_can(msg)

    def __send_can(self, msg: Can):
        """
        pack and send the message to the teensy
        """
        self.get_logger().debug(f"ID {msg.id} ({TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS(msg.id)].name}): {msg.buf}")

        # make sure the message is at most 8 bytes long, otherwise throw an error
        if(len(msg.buf) > 8):
            raise Exception

        # details how to pack the data, i = 4 bytes, B = unsigned byte
        pack_format = ">iB" + "B" * len(msg.buf)

        # package the message with the format and the id and data
        udp_packet = pack(pack_format, msg.id, len(msg.buf), *msg.buf)

        try:
            self.__socket.sendto(udp_packet, (self.__address, self.__send_port))
        except Exception as e:
            err = e.args
            # only error out if it wasn't from non-blocking
            if(err and err[0] == errno.EWOULDBLOCK):
                pass
            else:
                self.get_logger().error(f"failed to send can udp packet: {e}")

    def run(self):
        # start the socket so we can send and recv messages
        self.get_logger().info("starting can udp socket connection...")
        self.__socket.bind((self.__address, self.__recv_port))

        # make it so it will not stop the code when requesting a read
        self.__socket.setblocking(False)

        self.__receiver_thread = threading.Thread(
            target=udp_receive_thread,
            args=(self, self.__socket, self.__stop),
            daemon=True,
        )
        self.__receiver_thread.start()

        rclpy.spin(self)

    def destroy_node(self) -> bool:
        # Best-effort actuator safe state while the socket is still open.
        try:
            self.__send_safe_stop(include_estop=False)
        except Exception:
            pass
        self.__stop.set()
        try:
            self.__socket.close()
        except OSError:
            pass
        if self.__receiver_thread is not None:
            self.__receiver_thread.join(timeout=1.0)
            self.__receiver_thread = None
        return super().destroy_node()


def udp_receive_thread(node: CAN_UDP, socket: socket, stop: threading.Event):
    while rclpy.ok() and not stop.is_set():
        try:
            # Wait for a datagram instead of busy-spinning on a
            # non-blocking socket, which otherwise pegs a CPU core and
            # starves the basestation/heartbeat path.
            ready, _, _ = select.select([socket], [], [], 0.1)
            if not ready:
                continue
            data, addr = socket.recvfrom(1024)
            if data:
                # unpack the message with the same format as packing
                unpacked_data = unpack(">iB" + "B" * (len(data) - 5), data)
                message_id = unpacked_data[0]
                data_length = unpacked_data[1]
                buf = [0] * 8  # Initialize buffer with 8 zeros
                for i in range(unpacked_data[1]):
                    buf[i] = unpacked_data[2+i]
                msg = Can()
                msg.id = message_id
                msg.buf = buf
                node.on_message_received(msg)
        except BlockingIOError:
            continue
        except OSError:
            break
        except Exception as e:
            err = e.args
            # only error out if it wasn't from non-blocking
            if(err and err[0] == errno.EWOULDBLOCK):
                pass
            elif rclpy.ok():
                node.get_logger().info(f"failed to receive can udp packet: {e}")

def main():
    rclpy.init()
    can_udp = CAN_UDP()
    try:
        can_udp.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        can_udp.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
