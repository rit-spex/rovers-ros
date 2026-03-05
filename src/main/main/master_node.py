#!/usr/bin/env python3

from typing import Any

# ros imports
import rclpy
import rclpy.logging
from rclpy.node import Node, Timer
import rclpy.publisher
import rclpy.subscription
from std_msgs.msg import Bool, Float32, UInt8MultiArray, Int8, UInt16
from constants.CommandCodes import CONSTANTS

# from constants.CommandCodes import TOPICS_JOYSTICK, TOPICS_BUTTON, CONSTANTS
# from constants.CAN_Constants import CHANNEL, TOPICS

timeout_duration = 1  # seconds


class Master(Node):

    __estop_publisher: rclpy.publisher.Publisher

    __quit_subscription: rclpy.subscription.Subscription
    __estop_subscription: rclpy.subscription.Subscription

    __heartbeat_subscription: rclpy.subscription.Subscription
    __heartbeat_timer: Timer

    # heartbeat tracking
    __last_heartbeat_time: UInt16
    __current_heartbeat_time: UInt16
    __recieved_heartbeat: bool

    def __init__(self):
        super().__init__("Master")

        self.__led_status = False

        self.__last_heartbeat_time = UInt16()
        self.__current_heartbeat_time = UInt16()
        self.__recieved_heartbeat = False

        # create publishers and subscriptions for e-stop and heartbeat
        self.__estop_publisher = self.create_publisher(
            msg_type=Bool,
            topic="/ESTOP",
            qos_profile=10,
        )
        self.__quit_subscription = self.create_subscription(
            msg_type=Bool,
            topic="/BASESTATION/" + CONSTANTS.QUIT.NAME + "/" + CONSTANTS.QUIT.NAME,
            callback=self.__on_quit_received,
            qos_profile=10,
        )
        self.__heartbeat_subscription = self.create_subscription(
            msg_type=UInt16,
            topic="/BASESTATION/"
            + CONSTANTS.HEARTBEAT.NAME
            + "/"
            + CONSTANTS.HEARTBEAT.TIMESTAMP_MESSAGE,
            callback=self.__on_heartbeat_received,
            qos_profile=10,
        )
        self.__estop_subscription = self.create_subscription(
            msg_type=Bool,
            topic="/ESTOP",
            callback=self.__on_estop_received,
            qos_profile=10,
        )
        self.__heartbeat_timer = self.create_timer(
            timer_period_sec=timeout_duration,
            callback=self.__check_timeout,
        )

    def __check_timeout(self) -> None:
        # self.get_logger().info("Checking for heartbeat timeout...")
        if self.__recieved_heartbeat:
            if self.__current_heartbeat_time == self.__last_heartbeat_time:
                self.get_logger().info("Heartbeat timeout detected")
                estop_msg = Bool()
                estop_msg.data = True
                self.__estop_publisher.publish(estop_msg)
                return
            else:
                self.__last_heartbeat_time = self.__current_heartbeat_time
        return

    def __on_quit_received(self, msg: Bool):
        self.get_logger().info("Quit message received, forwarding e-stop ...")
        self.__estop_publisher.publish(msg)

    def __on_heartbeat_received(self, msg: UInt16):
        # self.get_logger().info("Heartbeat message received")
        self.__recieved_heartbeat = True
        self.__current_heartbeat_time = msg

    def __on_estop_received(self, msg: Bool):
        self.get_logger().info("E-Stop message received, shutting down ...")

        rclpy.shutdown()

    def run(self):
        self.get_logger().info("starting master node ...")
        rclpy.spin(self)


def main():
    rclpy.init()
    master = Master()
    master.run()


if __name__ == "__main__":
    main()
