from typing import Any
from constants.CommandCodes import CONSTANTS
from constants.CAN_structs import Message
from constants.CAN_constants import CAN_MESSAGE_IDS, ODRIVE_MESSAGE_IDS, Subsystems_Names, CAN_MESSAGES
from constants.can_encoding import TeensyCommunication, OdriveCommunication

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from custom_interfaces.msg import Can
from std_msgs.msg import Float32
from std_msgs.msg import Bool, Float32, UInt8MultiArray, Int8, Int16


class Chassis(Node):
    __topic: dict[str, Any]
    
    __LY_value: float
    __RY_value: float

    __publishers: dict[CAN_MESSAGE_IDS, Publisher]

    __front_R_vel_publisher: Publisher
    __front_L_vel_publisher: Publisher
    __back_R_vel_publisher: Publisher
    __back_L_vel_publisher: Publisher

    __front_R_canmsg: Message
    __front_L_canmsg: Message
    __back_R_canmsg: Message
    __back_L_canmsg: Message

    def __init__(self):
        super().__init__("chassis_node")

        self.__publishers = {}
        for (message_id, message) in CAN_MESSAGES.items():
            if message.subsystem != Subsystems_Names.CHASSIS:
                continue

            if message.isforJetson:
                self.create_subscription(
                    msg_type=Can,
                    topic=message.topic_name,
                    callback=self.__on_new_CAN_message_received,
                    qos_profile=10,
                )
            else:
                self.__publishers[message_id] = self.create_publisher(
                    msg_type=Can,
                    topic=message.topic_name,
                    qos_profile=10
                )

        self.__front_R_vel_publisher = self.create_publisher(
            Float32, "CHASSIS/FR/VEL", qos_profile=10
        )
        self.__front_L_vel_publisher = self.create_publisher(
            Float32, "CHASSIS/FL/VEL", qos_profile=10
        )
        self.__back_R_vel_publisher = self.create_publisher(
            Float32, "CHASSIS/BR/VEL", qos_profile=10
        )
        self.__back_L_vel_publisher = self.create_publisher(
            Float32, "CHASSIS/BL/VEL", qos_profile=10
        )

        self.create_subscription(
            Float32, "/BASESTATION/" + CONSTANTS.XBOX.NAME + "/" + CONSTANTS.XBOX.JOYSTICK.AXIS_LY_STR, self.__LY_callback, 10
        )
        self.create_subscription(
            Float32, "/BASESTATION/" + CONSTANTS.XBOX.NAME + "/" + CONSTANTS.XBOX.JOYSTICK.AXIS_RY_STR, self.__RY_callback, 10
        )
        self.create_subscription(
            msg_type=Bool,
            topic="/ESTOP",
            callback=self.__on_estop_received,
            qos_profile=10,
        )

        self.__front_L_value = 0.0
        self.__front_R_value = 0.0
        self.__back_R_value = 0.0
        self.__back_L_value = 0.0

        self.__front_R_canmsg = CAN_MESSAGES[ODRIVE_MESSAGE_IDS.FRONT_R_SET_VEL]
        self.__front_L_canmsg = CAN_MESSAGES[ODRIVE_MESSAGE_IDS.FRONT_L_SET_VEL]
        self.__back_R_canmsg = CAN_MESSAGES[ODRIVE_MESSAGE_IDS.BACK_R_SET_VEL]
        self.__back_L_canmsg = CAN_MESSAGES[ODRIVE_MESSAGE_IDS.BACK_L_SET_VEL]



    def __LY_callback(self, msg: Float32):
        self.__front_L_value = msg.data
        self.__back_L_value = msg.data

        self.__front_L_canmsg.signals["vel_cmd"].set_value(self.__front_L_value)
        self.__back_L_canmsg.signals["vel_cmd"].set_value(self.__back_L_value)

        front_L_can_packet = OdriveCommunication.encode_can_message(self.__front_L_canmsg)
        back_L_can_packet = OdriveCommunication.encode_can_message(self.__back_L_canmsg)

        self.__publishers[ODRIVE_MESSAGE_IDS.FRONT_L_SET_VEL].publish(front_L_can_packet)
        self.__publishers[ODRIVE_MESSAGE_IDS.BACK_L_SET_VEL].publish(back_L_can_packet)

        self.get_logger().info(f"LY_callback message: {msg}")
        self.get_logger().info(f"Topic is {self.__back_L_canmsg.topic_name}")

    def __RY_callback(self, msg: Float32):
        self.__front_R_value = msg.data
        self.__back_R_value = msg.data

        self.__front_R_canmsg.signals["vel_cmd"].set_value(self.__front_R_value)
        self.__back_R_canmsg.signals["vel_cmd"].set_value(self.__back_R_value)

        front_R_can_packet = OdriveCommunication.encode_can_message(self.__front_R_canmsg)
        back_R_can_packet = OdriveCommunication.encode_can_message(self.__back_R_canmsg)

        self.__publishers[ODRIVE_MESSAGE_IDS.FRONT_R_SET_VEL].publish(front_R_can_packet)
        self.__publishers[ODRIVE_MESSAGE_IDS.BACK_R_SET_VEL].publish(back_R_can_packet)

        self.get_logger().info(f"RY_callback message: {msg}")
        self.get_logger().info(f"Topic is {self.__front_R_canmsg.topic_name}")

    def __on_estop_received(self, msg: Bool):
        self.get_logger().info("E-STOP received, stopping chassis...")
        # self.__LY_value = 0
        # self.__RY_value = 0
        # self.__LY_Publisher.publish(self.__LY_value)
        # self.__RY_Publisher.publish(self.__RY_value)

        rclpy.shutdown()

    def __on_new_CAN_message_received(self, msg: Can):
        can_message: Message | None
        try:
            can_message = OdriveCommunication.decode_can_message(msg)
            if can_message is None:
                return
        except Exception as e:
            self.get_logger().error(f"Failed to decode CAN message: {e}")
            return

        if "HEARTBEAT" in can_messsage.name:
            pass
        elif "GET_ENCODER_ESTIMATES" in can_message.name:
            match can_message.name.split('_')[0]:
                case "FRONT_R":
                    self.__front_R_vel_publisher.publish(
                        can_message.signals["vel"].value
                    )
                case "FRONT_L":
                    self.__front_L_vel_publisher.publish(
                        can_message.signals["vel"].value
                    )
                case "BACK_R":
                    self.__back_R_vel_publisher.publish(
                        can_message.signals["vel"].value
                    )
                case "BACK_L":
                    self.__back_L_vel_publisher.publish(
                        can_message.signals["vel"].value
                    )

    def run(self):
        self.get_logger().info("starting chassis...")
        rclpy.spin(self)
        self.get_logger().info("stopping chassis")


def main():
    rclpy.init()
    chassis = Chassis()
    chassis.run()


if __name__ == "__main__":
    main()
