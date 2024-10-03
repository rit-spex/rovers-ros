#!/usr/bin/env python3

import rclpy
import serial
from rclpy import Node
from std_msgs.msg import String
from constants.RoverConstants import PORT, BAUD_RATE


class XBee(Node):
    __xbee: serial.Serial

    def __init__(self):
        # rospy.init_node("XBee")
        self.__xbee = serial.Serial(PORT, BAUD_RATE)

        self.create_subscription(String, "/sensors/GPS/TX", self.send_gps, 10)

        self.run()

    def send_gps(self, data: String):
        print(data)

    def run(self):
        rclpy.spin(self)


def main():
    xbee = XBee()


if __name__ == "__main__":
    main()
