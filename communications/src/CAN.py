#!/usr/bin/env python3
# from pydoc_data import topics
import rospkg
import sys
rospack = rospkg.RosPack()
package_path = rospack.get_path('constants')
sys.path.append(package_path + '/src')

import can
import rospy
from communications.msg import CAN_msg, CAN_FD_msg
from CAN_Constants import CHANNEL, TOPICS
import spidev



class CAN:
    def __init__(self) -> None:
        rospy.init_node("CAN", anonymous=True)

        self.jetson = can.Bus(channel=CHANNEL.JETSON)
        can.Notifier(self.jetson, [JETSON_LISTENER()])

        subscribers = []
        for message_id in range(0, len(TOPICS)):
            subscribers.append(
                rospy.Subscriber(
                    f"/CAN/TX/{TOPICS[message_id]['name']}", CAN_msg, self.send_msg
                )
            )

        self.run()

    def send_msg(self, msg):
        with can.Bus(msg.channel) as bus:
            bus_msg = can.Message(arbitration_id=msg.id, data=msg.buf)
            try:
                bus.send(bus_msg)
            except:
                rospy.loginfo("Failed to send message")

    def run(self):
        rospy.spin()


class JETSON_LISTENER(can.Listener):
    def __init__(self) -> None:
        super().__init__()

    def on_message_received(self, msg: can.Message) -> None:
        ros_msg = CAN_FD_msg()
        ros_msg.channel = TOPICS[msg.arbitration_id]["channel"]
        ros_msg.id = msg.arbitration_id
        ros_msg.buf = msg.data

        rospy.Publisher(f"/CAN/RX/{TOPICS[msg.arbitration_id]['name']}", CAN_msg).publish(
            ros_msg
        )

def main():
    can = CAN()

if __name__ == "__main__":
    main()
