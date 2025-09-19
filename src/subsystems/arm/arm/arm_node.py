from constants.CAN_Constants import TOPICS
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from custom_interfaces.msg import Can
from std_msgs.msg import Bool


class Arm(Node):
    # __base_publisher: Publisher
    # __shoulder_publisher: Publisher
    # __elbow_publsiher: Publisher
    # __wrist_publsiher: Publisher
    # __claw_publsiher: Publisher
    # __solinoid_publsiher: Publisher
    __publishers: list[Publisher]

    __base_forward: int
    __base_backward: int

    __shoulder_forward: int
    __shoulder_backward: int

    __elbow_forward: int
    __elbow_backward: int

    def __init__(self):
        super().__init__("arm_node")

        # self.__base_publisher = self.create_publisher(Can, TOPICS[11]["name"], 10)
        # self.__shoulder_publisher = self.create_publisher(Can, TOPICS[12]["name"], 10)
        # self.__elbow_publsiher = self.create_publisher(Can, TOPICS[13]["name"], 10)
        # self.__wrist_publsiher = self.create_publisher(Can, TOPICS[14]["name"], 10)
        # self.__claw_publsiher = self.create_publisher(Can, TOPICS[15]["name"], 10)
        # self.__solinoid_publsiher = self.create_publisher(Can, TOPICS[16]["name"], 10)

        self.__publishers = []
        for i in range(11, 17): # change based on amount of IDs filled out
            self.__publishers.append(self.create_publisher(Can, f"/CAN/TX/{TOPICS[i]['name']}", 10))

        self.create_subscription(Bool, "/Xbee/RX/Controller/Buttons/LT", self.__base_forward_callback, 10)
        self.create_subscription(Bool, "/Xbee/RX/Controller/Buttons/RT", self.__base_backward_callback, 10)
        self.create_subscription(Bool, "/Xbee/RX/Controller/Buttons/LT", self.__shoulder_forward_callback, 10)
        self.create_subscription(Bool, "/Xbee/RX/Controller/Buttons/RT", self.__shoulder_backward_callback, 10)
        self.create_subscription(Bool, "/Xbee/RX/Controller/Buttons/LEFT_BUMPER", self.__elbow_forward_callback, 10)
        self.create_subscription(Bool, "/Xbee/RX/Controller/Buttons/RIGHT_BUMPER", self.__elbow_backward_callback, 10)
        self.create_subscription(Bool, "/Xbee/RX/Controller/Buttons/X", self.__elbow_forward_callback, 10)
        self.create_subscription(Bool, "/Xbee/RX/Controller/Buttons/Y", self.__elbow_backward_callback, 10)

        self.__base_forward = 0
        self.__base_backward = 0

        self.__shoulder_forward = 0
        self.__shoulder_backward = 0

        self.__elbow_forward = 0
        self.__elbow_backward = 0

    def __base_forward_callback(self, msg: Bool):
        self.__base_forward = int(msg.data)
        self.__send_controller_data(11)

    def __base_backward_callback(self, msg: Bool):
        self.__base_backward = int(msg.data)
        self.__send_controller_data(11)

    def __shoulder_forward_callback(self, msg: Bool):
        self.__shoulder_forward = int(msg.data)
        self.__send_controller_data(12)

    def __shoulder_backward_callback(self, msg: Bool):
        self.__shoulder_backward = int(msg.data)
        self.__send_controller_data(12)

    def __elbow_forward_callback(self, msg: Bool):
        self.__elbow_forward = int(msg.data)
        self.__send_controller_data(13)

    def __elbow_backward_callback(self, msg: Bool):
        self.__elbow_backward = int(msg.data)
        self.__send_controller_data(13)

    def __elbow_forward_callback(self, msg: Bool):
        self.__elbow_forward = int(msg.data)
        self.__send_controller_data(13)

    def __elbow_backward_callback(self, msg: Bool):
        self.__elbow_backward = int(msg.data)
        self.__send_controller_data(13)

    def __send_controller_data(self, ID: int):
        # self.get_logger().info(f"base_forward: {self.__base_forward}")
        # self.get_logger().info(f"base_backward: {self.__base_backward}")
        self.get_logger().info(f"GOTTEN ID: {ID}")

        ros_msg = Can()
        ros_msg.channel = TOPICS[ID]["channel"]
        ros_msg.id = TOPICS[ID]["id"]
        ros_msg.buf = [0, 0, 0, 0, 0, 0, 0, 0]
        match ID:
            case 11:
                self.get_logger().info("BASE")
                engaged = self.__base_forward | self.__base_backward
                direction = self.__base_forward - self.__base_backward
                direction = 0 if direction == -1 else direction
                ros_msg.buf[0] = engaged
                ros_msg.buf[1] = direction
            case 12:
                self.get_logger().info("SHOULDER")
                engaged = self.__shoulder_forward | self.__shoulder_backward
                direction = self.__shoulder_forward - self.__shoulder_backward
                self.get_logger().info(f"engaged for elbow: {engaged}")
                direction = 0 if direction == -1 else direction
                ros_msg.buf[0] = engaged
                ros_msg.buf[1] = direction
            case 13:
                self.get_logger().info("ELBOW")
                engaged = self.__elbow_forward | self.__elbow_backward
                direction = self.__elbow_forward - self.__elbow_backward
                self.get_logger().info(f"engaged for elbow: {engaged}")
                direction = 0 if direction == -1 else direction
                ros_msg.buf[0] = engaged
                ros_msg.buf[1] = direction
            case _:
                self.get_logger().info(f"{ID} not accounted for...")
        self.get_logger().info(f"PUBLISHING TO {self.__publishers[ID - 11].topic_name}")
        self.__publishers[ID - 11].publish(ros_msg)

        # base_ros_msg = Can()
        # base_ros_msg.channel = TOPICS[11]["channel"]
        # base_ros_msg.id = TOPICS[11]["id"]
        # base_ros_msg.buf = [
        #     self.__engage_base,
        #     direction,
        #     0,
        #     0,
        #     0,
        #     0,
        #     0,
        #     0,
        # ]
        # self.__base_publisher.publish(ros_msg)

    def run(self):
        self.get_logger().info("starting arm...")
        rclpy.spin(self)
        self.get_logger().info("stopping arm...")

def main():
    rclpy.init()
    arm = Arm()
    arm.run()
