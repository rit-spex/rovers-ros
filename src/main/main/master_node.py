#!/usr/bin/env python3

# ros imports
import rclpy
import rclpy.logging
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import rclpy.publisher
import rclpy.subscription
from std_msgs.msg import Bool, UInt8, UInt16
from constants.CommandCodes import CONSTANTS

timeout_duration = 1  # seconds


class Master(Node):

    __estop_publisher: rclpy.publisher.Publisher
    __auto_state_publisher: rclpy.publisher.Publisher

    # heartbeat tracking
    __last_heartbeat_time: int
    __current_heartbeat_time: int
    __received_heartbeat: bool

    def __init__(self):
        super().__init__("Master")

        self.__last_heartbeat_time = 0
        self.__current_heartbeat_time = 0
        self.__received_heartbeat = False

        # create publishers and subscriptions for e-stop and heartbeat
        self.__estop_publisher = self.create_publisher(
            msg_type=Bool,
            topic="/ESTOP",
            qos_profile=10,
        )
        self.__auto_state_publisher = self.create_publisher(
            msg_type=UInt8,
            topic="/ROVER/AUTO_STATE",
            qos_profile=10,
        )
        self.create_subscription(
            msg_type=Bool,
            topic="/BASESTATION/" + CONSTANTS.QUIT.NAME + "/" + CONSTANTS.QUIT.QUIT_MESSAGE,
            callback=self.__on_quit_received,
            qos_profile=10,
        )
        self.create_subscription(
            msg_type=UInt16,
            topic="/BASESTATION/"
            + CONSTANTS.HEARTBEAT.NAME
            + "/"
            + CONSTANTS.HEARTBEAT.TIMESTAMP_MESSAGE,
            callback=self.__on_heartbeat_received,
            qos_profile=10,
        )
        self.create_subscription(
            msg_type=UInt8,
            topic="/BASESTATION/auto_state/auto_state",
            callback=self.__on_auto_state_received,
            qos_profile=10,
        )
        self.create_subscription(
            msg_type=Bool,
            topic="/ESTOP",
            callback=self.__on_estop_received,
            qos_profile=10,
        )
        self.create_timer(
            timer_period_sec=timeout_duration,
            callback=self.__check_timeout,
        )

    def __check_timeout(self) -> None:
        # self.get_logger().info("Checking for heartbeat timeout...")
        if self.__received_heartbeat:
            if self.__current_heartbeat_time == self.__last_heartbeat_time:
                self.get_logger().info("Heartbeat timeout detected")
                estop_msg = Bool()
                estop_msg.data = True
                self.__estop_publisher.publish(estop_msg)
            else:
                self.__last_heartbeat_time = self.__current_heartbeat_time

    def __on_quit_received(self, msg: Bool):
        self.get_logger().info("Quit message received, forwarding e-stop ...")
        self.__estop_publisher.publish(msg)

    def __on_heartbeat_received(self, msg: UInt16):
        # self.get_logger().info("Heartbeat message received")
        self.__received_heartbeat = True
        self.__current_heartbeat_time = int(msg.data)

    def __on_estop_received(self, msg: Bool):
        self.get_logger().info("E-Stop message received, shutting down ...")

        rclpy.shutdown()

    def __on_auto_state_received(self, msg: UInt8):
        self.__auto_state_publisher.publish(UInt8(data=msg.data))

    def run(self):
        self.get_logger().info("starting master node ...")
        rclpy.spin(self)


def main():
    rclpy.init()
    master = Master()
    try:
        master.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        master.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
