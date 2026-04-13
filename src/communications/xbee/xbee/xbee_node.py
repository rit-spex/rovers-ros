import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.publisher import Publisher
from rover_protocol.constants import CONSTANTS
from std_msgs.msg import UInt8MultiArray
from digi.xbee.devices import RemoteXBeeDevice, XBee64BitAddress, XBeeDevice, TimeoutException

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
        self.__remote_xbee = None

        self.__publisher = self.create_publisher(UInt8MultiArray, "/XBEE/MESSAGES/RX", 10)
        self.create_subscription(UInt8MultiArray, "/XBEE/MESSAGES/TX", self.send_msg, 10)

    def read_data(self) -> list[int] | None:
        message = None
        try:
            message = self.__xbee_device.read_data(0.0004)
            if message is None:
                return None

            # update the remote xbee's connection
            self.__remote_xbee = message.remote_device
            return list(message.data)
        except TimeoutException:
            return []
            # self.get_logger().info("timed out")
        except Exception:
            self.get_logger().info("failed to read data")
            return None


    def send_msg(self, msg: UInt8MultiArray) -> None:
        """
        pack and send the message to the basestation
        """

        # only send if we have a remote xbee to send to
        if self.__remote_xbee is None:
            self.get_logger().debug("no remote xbee to send to, message not sent")
            return

        # only send if we have a remote xbee to send to
        if self.__xbee_device is None:
            self.get_logger().debug("no remote xbee to send to, message not sent")
            return

        try:
            message_bytes = bytes(msg.data)
            self.__xbee_device.send_data(self.__remote_xbee, message_bytes)

        except ValueError:
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
            rclpy.spin(self)
            return

        try:
            while rclpy.ok():
                data = self.read_data()
                if data is None:
                    self.get_logger().info("message was none")
                    break
                if len(data) == 0:
                    continue
                self.publish_data(data)
        finally:
            try:
                self.__xbee_device.close()
            except Exception:
                pass


def main() -> None:
    rclpy.init()
    xbee = Xbee()
    try:
        xbee.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        xbee.get_logger().error(f"xbee node failed: {exc}")
    finally:
        xbee.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
