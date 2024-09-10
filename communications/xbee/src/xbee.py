#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import String


class XBee:
    PORT = "COM11"
    BAUD_RATE = 9600

    xbee: serial.Serial

    def __init__(self):
        rospy.init_node("XBee")
        # self.xbee = serial.Serial(self.PORT, self.BAUD_RATE)

        rospy.Subscriber("/sensors/GPS/TX", String, self.send_gps)

        self.run()

    def send_gps(self, data: String):
        print(data)

    def run(self):
        rospy.spin()


def main():
    xbee = XBee()


if __name__ == "__main__":
    main()
