from typing import Any
from constants.CAN_Constants import TOPICS
from constants.CommandCodes import CONSTANTS

import rclpy
from rclpy.node import Node, Timer
from rclpy.publisher import Publisher
from std_msgs.msg import Float32
from std_msgs.msg import Bool, Float32, UInt8MultiArray, Int8, Int16
import Jetson.GPIO as GPIO

LED_flicker_sec = 1.0
LED_PIN = 7


class StatusLED(Node):

    LED_flicker_timer: Timer

    # gpio status
    __led_status: bool

    def __init__(self):
        super().__init__("status_led_node")

        # GPIO setup to have led indicate heartbeat status
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(LED_PIN, GPIO.OUT, initial=GPIO.LOW)
        # GPIO.output(LED_PIN, GPIO.LOW)

        # requires some default value
        self.__led_status = False

        self.create_subscription(
            msg_type=Bool,
            topic="/ESTOP",
            callback=self.__on_estop_received,
            qos_profile=10,
        )

        self.LED_flicker_timer = self.create_timer(
            timer_period_sec=LED_flicker_sec,
            callback=self.__flicker_led,
        )

    def __flicker_led(self):
        # self.get_logger().info("flicker_led")
        if self.__led_status == True:
            self.__led_status = False
            GPIO.output(LED_PIN, GPIO.LOW)  # Turn off LED to indicate live jetson
        else:
            self.__led_status = True
            GPIO.output(LED_PIN, GPIO.HIGH)  # Turn on LED to indicate live jetson

    def __on_estop_received(self, msg: Bool):
        self.get_logger().info("E-STOP received, stopping status LED...")
        self.__led_status = True
        GPIO.output(LED_PIN, GPIO.HIGH)  # Turn the LED solid to indicate e-stop
        rclpy.shutdown()

    def run(self):
        self.get_logger().info("starting status LED...")
        rclpy.spin(self)
        self.get_logger().info("stopping status LED")


def main():
    rclpy.init()
    status_led = StatusLED()
    status_led.run()


if __name__ == "__main__":
    main()
