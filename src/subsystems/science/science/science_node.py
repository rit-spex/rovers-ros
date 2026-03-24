from typing import Any, Callable, Dict

from pygame import key

from pygame import key
from constants.CommandCodes import CONSTANTS
from constants.CAN_structs import Message
from constants.CAN_constants import CAN_MESSAGE_IDS, TEENSY_CAN_MESSAGES, Subsystems_Names
from constants.can_encoding import TeensyCommunication

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from custom_interfaces.msg import Can
from std_msgs.msg import Float32
from std_msgs.msg import Bool, Float32, UInt8MultiArray, Int8, Int16


class Science(Node):
    __publishers: dict[CAN_MESSAGE_IDS, Publisher]

    def __init__(self):
        super().__init__("science_node")

        self._life_detection = {
            "move_auger_up": 0,
            "move_auger_down": 0,
            "limit_switch_2": False,
            "auger_depth": 0,
            "pump_output_level": 0,
            "slide_position": 0,
            "selected_tube": 0,
            "spec_slide_position": 0,
            "spec_color_sensor": 0,
        }

        self.__publishers = {}
        for (message_id, message) in TEENSY_CAN_MESSAGES.items():
            if(message.subsystem == Subsystems_Names.SCIENCE):
                if not message.isforJetson:
                    self.__publishers[message_id] = self.create_publisher(
                        msg_type=Can,
                        topic=message.topic_name,
                        qos_profile=10
                    )
                # else:
                #     self.create_subscription(
                #         msg_type=Can,
                #         topic=message.topic_name,
                #         callback=self.__on_can_message_received,
                #         qos_profile=10,
                #     )

        self.create_subscription(
            msg_type=Bool,
            topic="/ESTOP",
            callback=self.__on_estop_received,
            qos_profile=10,
        )

        def _sub_u8(self, topic: str, target: Dict, key: str) -> None:
            self.create_subscription(Bool, topic, self._setter(target, key), 10)

        def _setter(self, target: Dict, key: str) -> Callable:
            def _callback(msg):
                target[key] = msg.data

            return _callback

    # def __on_can_message_received(self, msg: Can):
        
    #     self.get_logger().info(f"Received CAN message on topic {msg.topic}")

    def __on_estop_received(self, msg: Bool):
        self.get_logger().info("E-STOP received, stopping science..")
        rclpy.shutdown()

    def run(self):
        self.get_logger().info("starting science...")
        rclpy.spin(self)
        self.get_logger().info("stopping science...")


def main():
    rclpy.init()
    science = Science()
    science.run()


if __name__ == "__main__":
    main()
