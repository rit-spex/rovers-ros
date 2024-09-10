from pydoc_data import topics
import can
import rospy
from enum import IntEnum
from rovers.can_msgs import CAN_msg, CAN_FD_msg


class CHANNEL(IntEnum):
    JETSON = 0
    MAIN_BODY = 1
    SCIENCE_BOARD = 2
    ARM_BOARD = 3


TOPICS = {
    0: {
        "id": 0,
        "name": "E_STOP",
        "buf": bytearray(8),
        "channel": CHANNEL.MAIN_BODY,
    },
    1: {
        "id": 1,
        "name": "TARGET_VELOCITY",
        "buf": bytearray(8),
        "channel": CHANNEL.MAIN_BODY,
    },
    2: {
        "id": 2,
        "name": "CURRENT_VELOCITY",
        "buf": bytearray(8),
        "channel": CHANNEL.MAIN_BODY,
    },
}


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
        rospy.init_node("jetson_listener")

    def on_message_received(self, msg: can.Message) -> None:
        ros_msg = CAN_FD_msg()
        ros_msg.channel = TOPICS[msg.arbitration_id]["channel"]
        ros_msg.id = msg.arbitration_id
        ros_msg.buf = msg.data

        rospy.Publisher(f"/CAN{TOPICS[msg.arbitration_id]['name']}", CAN_msg).publish(
            ros_msg
        )
