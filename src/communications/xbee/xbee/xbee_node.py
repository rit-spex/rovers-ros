import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt8MultiArray
from digi.xbee.devices import (
    RemoteXBeeDevice,
    XBee64BitAddress,
    XBeeDevice,
    TimeoutException,
)
from constants.CommandCodes import CONSTANTS

# from custom_interfaces.msg import Can
# from constants.constants.CAN_Constants import CHANNEL, TOPICS


class Xbee(Node):
    __port: str
    __baud_rate: int
    __xbee_device: XBeeDevice
    __remote_xbee: RemoteXBeeDevice | None
    __publisher: Publisher

    def __init__(self) -> None:
        super().__init__("xbee_node")

        self.__port = "/dev/ttyUSB0"
        self.__baud_rate = 230400
        self.__xbee_device = XBeeDevice(self.__port, self.__baud_rate)
        self.__publisher = self.create_publisher(
            UInt8MultiArray, "/XBEE/MESSAGES",
            QoSProfile(depth=10,
                       reliability=ReliabilityPolicy.BEST_EFFORT))
        self.__remote_xbee = None
        self.__shutting_down = False
        self.__publisher = self.create_publisher(
            UInt8MultiArray, "/XBEE/MESSAGES/RX", 10
        )
        self.create_subscription(
            UInt8MultiArray, "/XBEE/MESSAGES/TX", self.send_msg, 10
        )

    def read_data(self, message) -> None:
        """Publish one radio frame unless the node is shutting down."""
        if self.__shutting_down or not rclpy.ok():
            return
        try:
            # message = self.__xbee_device.read_data(0.0004)
            if message is None:
                return None

            # update the remote xbee's connection
            self.__remote_xbee = message.remote_device
            self.publish_data(list(message.data))
        except Exception as e:
            # The device callback runs on a driver thread and can race with
            # interpreter/context teardown; do not log into a dead context.
            if not self.__shutting_down and rclpy.ok():
                self.get_logger().info(f"failed to read data {e}")
            return

    def send_msg(self, msg: UInt8MultiArray) -> None:
        """
        pack and send the message to the basestation
        """

        # only send if we have a remote xbee to send to
        if self.__remote_xbee is None:
            self.get_logger().error("no remote xbee to send to, message not sent")
            return

        # only send if we have a remote xbee to send to
        if self.__xbee_device is None:
            self.get_logger().error("no remote xbee to send to, message not sent")
            return

        try:
            message_bytes = bytes(msg.data)
            self.__xbee_device.send_data(self.__remote_xbee, message_bytes)
            self.get_logger().debug(f"sent data, {msg.data}")

        except ValueError as e:
            self.get_logger().error(f"value error {e}")
            raise
        except Exception as e:
            self.get_logger().error(f"failed to send data: {e}")
            return

    def publish_data(self, data: list[int]) -> None:
        msg = UInt8MultiArray()
        msg.data = data
        self.__publisher.publish(msg)

    def run(self) -> None:
        self.get_logger().info("starting xbee...")
        try:
            self.__xbee_device.open()
        except Exception as exc:
            self.get_logger().warning(
                f"xbee unavailable on {self.__port}; running without radio ({exc})"
            )
            return

        self.__xbee_device.add_data_received_callback(self.read_data)

    def shutdown(self) -> None:
        """Stop the radio callback and close the device during teardown."""
        self.__shutting_down = True
        try:
            if self.__xbee_device.is_open():
                try:
                    self.__xbee_device.del_data_received_callback(
                        self.read_data)
                except Exception:
                    # Older digi-xbee versions may not have the callback
                    # registered under the same bound method.
                    pass
                self.__xbee_device.close()
        except Exception as exc:
            if rclpy.ok():
                self.get_logger().warning(
                    f"failed to close xbee device: {exc}")


def main() -> None:
    rclpy.init(signal_handler_options=SignalHandlerOptions.ALL)
    node = None
    try:
        node = Xbee()
        node.run()
        # Spin inside the try: Ctrl-C and SIGINT arrive here, and the other
        # arm nodes handle them without a traceback.
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.shutdown()
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
