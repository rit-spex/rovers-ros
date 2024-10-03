#!/usr/bin/env python3

import rospy
from std_msgs.msg import String


class GPS:
    pub: rospy.Publisher

    def __init__(self) -> None:
        rospy.init_node("gps")

        self.pub = rospy.Publisher("/sensors/GPS/TX", String, queue_size=10)

        self.run()

    def run(self):
        while not rospy.is_shutdown():
            user_input = input("> ")
            self.pub.publish(user_input)


def main():
    gps = GPS()


if __name__ == "__main__":
    main()
