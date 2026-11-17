from constants.CAN_Constants import CHANNEL, TOPICS, TOPIC_RANGES
from constants.CommandCodes import CONSTANTS

from numpy import uint8
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from custom_interfaces.msg import Can
from std_msgs.msg import Bool


class Arm(Node):
    __publishers: list[Publisher]

    __base_forward: int
    __base_backward: int

    __shoulder_forward: int
    __shoulder_backward: int

    __elbow_forward: int
    __elbow_backward: int

    #heres a change

    def __init__(self):
        super().__init__("arm_node")

        self.__publishers = []
        for i in range(TOPIC_RANGES[CHANNEL.ARM_BOARD][0], TOPIC_RANGES[CHANNEL.ARM_BOARD][1]): # change based on amount of IDs filled out
            self.__publishers.append(self.create_publisher(Can, f"/CAN/TX/{TOPICS[i]['name']}", 10))

        self.create_subscription(Bool, "/BASESTATION/" + CONSTANTS.N64.NAME + "/" + CONSTANTS.N64.BUTTON.L_STR       , self.__base_forward_callback, 10)
        self.create_subscription(Bool, "/BASESTATION/" + CONSTANTS.N64.NAME + "/" + CONSTANTS.N64.BUTTON.R_STR       , self.__base_backward_callback, 10)
        self.create_subscription(Bool, "/BASESTATION/" + CONSTANTS.N64.NAME + "/" + CONSTANTS.N64.BUTTON.C_UP_STR    , self.__shoulder_forward_callback, 10)
        self.create_subscription(Bool, "/BASESTATION/" + CONSTANTS.N64.NAME + "/" + CONSTANTS.N64.BUTTON.C_DOWN_STR  , self.__shoulder_backward_callback, 10)
        self.create_subscription(Bool, "/BASESTATION/" + CONSTANTS.N64.NAME + "/" + CONSTANTS.N64.BUTTON.C_RIGHT_STR , self.__elbow_forward_callback, 10)
        self.create_subscription(Bool, "/BASESTATION/" + CONSTANTS.N64.NAME + "/" + CONSTANTS.N64.BUTTON.C_LEFT_STR  , self.__elbow_backward_callback, 10)
        self.create_subscription(Bool, "/BASESTATION/" + CONSTANTS.N64.NAME + "/" + CONSTANTS.N64.BUTTON.DP_DOWN_STR , self.__bend_wrist_forward_callback, 10)
        self.create_subscription(Bool, "/BASESTATION/" + CONSTANTS.N64.NAME + "/" + CONSTANTS.N64.BUTTON.DP_UP_STR   , self.__bend_wrist_backward_callback, 10)
        self.create_subscription(Bool, "/BASESTATION/" + CONSTANTS.N64.NAME + "/" + CONSTANTS.N64.BUTTON.DP_LEFT_STR , self.__twist_wrist_forward_callback, 10)
        self.create_subscription(Bool, "/BASESTATION/" + CONSTANTS.N64.NAME + "/" + CONSTANTS.N64.BUTTON.DP_RIGHT_STR, self.__twist_wrist_backward_callback, 10)
        self.create_subscription(Bool, "/BASESTATION/" + CONSTANTS.N64.NAME + "/" + CONSTANTS.N64.BUTTON.B_STR       , self.__gripper_forward_callback, 10)
        self.create_subscription(Bool, "/BASESTATION/" + CONSTANTS.N64.NAME + "/" + CONSTANTS.N64.BUTTON.A_STR       , self.__gripper_backward_callback, 10)
        self.create_subscription(Bool, "/BASESTATION/" + CONSTANTS.N64.NAME + "/" + CONSTANTS.N64.BUTTON.Z_STR       , self.__solenoid_callback, 10)

        self.create_subscription(
            msg_type=Bool,
            topic="/ESTOP",
            callback=self.__on_estop_received,
            qos_profile=10,
        )

        self.__base_forward = 0
        self.__base_backward = 0

        self.__shoulder_forward = 0
        self.__shoulder_backward = 0

        self.__elbow_forward = 0
        self.__elbow_backward = 0

        self.__bend_wrist_forward = 0
        self.__bend_wrist_backward = 0

        self.__twist_wrist_forward = 0
        self.__twist_wrist_backward = 0

        self.__gripper_forward = 0
        self.__gripper_backward = 0

        self.__solenoid = 0

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

    def __bend_wrist_forward_callback(self, msg: Bool):
        self.__bend_wrist_forward = int(msg.data)
        self.__send_controller_data(14)

    def __bend_wrist_backward_callback(self, msg: Bool):
        self.__bend_wrist_backward = int(msg.data)
        self.__send_controller_data(14)

    def __twist_wrist_forward_callback(self, msg: Bool):
        self.__twist_wrist_forward = int(msg.data)
        self.__send_controller_data(15)

    def __twist_wrist_backward_callback(self, msg: Bool):
        self.__twist_wrist_backward = int(msg.data)
        self.__send_controller_data(15)

    def __gripper_forward_callback(self, msg: Bool):
        self.__gripper_forward = int(msg.data)
        self.__send_controller_data(16)

    def __gripper_backward_callback(self, msg: Bool):
        self.__gripper_backward = int(msg.data)
        self.__send_controller_data(16)

    def __solenoid_callback(self, msg: Bool):
        self.__solenoid = int(msg.data)
        self.__send_controller_data(17)

    def __base_number(self, num: int, base: int) -> int:
        """
        Floors a number to a certain base\n
        Ex:
        `__base_number(-1, 2) = 2`
        `__base_number(3, 2) = 3`
        """

        return num if num > base else base

    def __send_controller_data(self, ID: int):
        ros_msg = Can()
        ros_msg.channel = TOPICS[ID]["channel"]
        ros_msg.id = TOPICS[ID]["id"]
        ros_msg.buf = [0, 0, 0, 0, 0, 0, 0, 0]

        match ID:
            case 11: # BASE
                ros_msg.buf[0] = self.__base_forward | self.__base_backward
                ros_msg.buf[1] = self.__base_number(self.__base_forward - self.__base_backward, 0)
            case 12: # SHOULDER
                ros_msg.buf[0] = self.__shoulder_forward | self.__shoulder_backward
                ros_msg.buf[1] = self.__base_number(self.__shoulder_forward - self.__shoulder_backward, 0)
            case 13: # ELBOW
                ros_msg.buf[0] = self.__elbow_forward | self.__elbow_backward
                ros_msg.buf[1] = self.__base_number(self.__elbow_forward - self.__elbow_backward, 0)
            case 14: # BEND WRIST
                ros_msg.buf[0] = self.__bend_wrist_forward | self.__bend_wrist_backward
                ros_msg.buf[1] = self.__base_number(self.__bend_wrist_forward - self.__bend_wrist_backward, 0)
            case 15: # TWIST WRIST
                ros_msg.buf[0] = self.__twist_wrist_forward | self.__twist_wrist_backward
                ros_msg.buf[1] = self.__base_number(self.__twist_wrist_forward - self.__twist_wrist_backward, 0)
            case 16: # GRIPPER
                ros_msg.buf[0] = self.__gripper_forward | self.__gripper_backward
                ros_msg.buf[1] = self.__base_number(self.__gripper_forward - self.__gripper_backward, 0)
            case 17: # SOLENOID
                engaged = self.__solenoid
                ros_msg.buf[0] = engaged
            case _:
                self.get_logger().info(f"{ID} not accounted for...")
        self.__publishers[ID - 10].publish(ros_msg)

    def __on_estop_received(self, msg: Bool):
        self.get_logger().info("E-STOP received, stopping arm...")
        self.__base_forward = 0
        self.__base_backward = 0
        self.__shoulder_forward = 0
        self.__shoulder_backward = 0
        self.__elbow_forward = 0
        self.__elbow_backward = 0
        self.__bend_wrist_forward = 0
        self.__bend_wrist_backward = 0
        self.__twist_wrist_forward = 0
        self.__twist_wrist_backward = 0
        self.__gripper_forward = 0
        self.__gripper_backward = 0
        self.__solenoid = 0

        self.__send_controller_data(11)
        self.__send_controller_data(12)
        self.__send_controller_data(13)
        self.__send_controller_data(14)
        self.__send_controller_data(15)
        self.__send_controller_data(16)
        self.__send_controller_data(17)

        rclpy.shutdown()


    def run(self):
        self.get_logger().info("starting arm...")
        rclpy.spin(self)
        self.get_logger().info("stopping arm...")

def main():
    rclpy.init()
    arm = Arm()
    arm.run()
