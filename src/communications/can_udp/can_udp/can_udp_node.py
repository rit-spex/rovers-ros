import threading
import rclpy
from rclpy.node import Node
import rclpy.subscription
import rclpy.publisher

from custom_interfaces.msg import Can

from constants.CAN_constants import TEENSY_CAN_MESSAGES, CAN_MESSAGE_IDS
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

    # This is needed as a var to shut it down early during E-stop
    __can_tx_subscriptions: dict[CAN_MESSAGE_IDS, rclpy.subscription.Subscription]

    def __init__(self) -> None:
        super().__init__("CAN_udp_node")

        self.__address = "127.0.0.1"
        self.__recv_port = 8001
        self.__send_port = 8000
        self.__socket = socket(skt.AF_INET, skt.SOCK_DGRAM)
        self.__buffer_size = 1024

        self.__publishers = {}
        self.__can_tx_subscriptions = {}

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

        # 1. Create and start the receiver thread for udp messages
        receiver_thread = threading.Thread(target=udp_receive_thread, args=(self, self.__socket))
        receiver_thread.start()

        self.run()

    def __on_estop_received(self, msg: Bool):
        self.get_logger().info("E-STOP received, shutting down can_comms_node")

        # First turn off the subscription so no new messages can be sent
        for subscription in self.__can_tx_subscriptions.values():
            self.destroy_subscription(subscription)

        self.get_logger().info("CAN_node: sending default messages before shutting down")
        # clear out all of the messages with default values
        for (message_id, message) in TEENSY_CAN_MESSAGES.items():
            # Set all of the value to default
            message.reset()
            can_packet = TeensyCommunication.encode_can_message(message)
            if can_packet is not None:
                self.send_msg(can_packet)

        self.__socket.close()
        rclpy.shutdown()

    def on_message_received(self, msg: Can):
        try:
            self.get_logger().debug(f"Received ID {msg.id} ({TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS(msg.id)].name}): {msg.buf}")
            self.__publishers[msg.id].publish(msg)
        except Exception as e:
            self.get_logger().error(f"failed to publish can message: {e}")


    def send_msg(self, msg: Can):
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
            if(err[0] == errno.EWOULDBLOCK):
                pass
            else:
                self.get_logger().error(f"failed to send can udp packet: {e}")

    def run(self):
        # start the socket so we can send and recv messages
        self.get_logger().info("starting can udp socket connection...")
        self.__socket.bind((self.__address, self.__recv_port))

        # make it so it will not stop the code when requesting a read
        self.__socket.setblocking(False)

        rclpy.spin(self)

def udp_receive_thread(node: CAN_UDP, socket: socket):
    while rclpy.ok():
        try:
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
        except Exception as e:
            err = e.args
            # only error out if it wasn't from non-blocking
            if(err[0] == errno.EWOULDBLOCK):
                pass
            else:
                node.get_logger().info(f"failed to receive can udp packet: {e}")


def main():
    rclpy.init()
    can_udp = CAN_UDP()
    can_udp.run()

if __name__ == "__main__":
    main()
