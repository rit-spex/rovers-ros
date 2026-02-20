from constants.CommandCodes import CONSTANTS
from constants.can_encoding import TeensyCommunication
from constants.CAN_constants import CAN_MESSAGE_IDS, TEENSY_CAN_MESSAGES, ArmState, ArmDirection, Subsystems_Names

from numpy import uint8
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from custom_interfaces.msg import Can
from std_msgs.msg import Bool


class Arm(Node):

    __publishers: dict[CAN_MESSAGE_IDS, Publisher]
    __base_target_angle: float
    __shoulder_target_angle: float
    __elbow_target_angle: float
    __bend_wrist_target_angle: float
    __twist_wrist_target_angle: float
    __gripper_closed: bool
    __solenoid_engaged: bool

    def __init__(self):
        super().__init__("arm_node")

        self.__publishers = {}
        for (message_id, message) in TEENSY_CAN_MESSAGES.items():
            if(message.subsystem == Subsystems_Names.ARM):
                self.__publishers[message_id] = self.create_publisher(
                    msg_type=Can,
                    topic=message.topic_name,
                    qos_profile=10
                )

        # self.create_subscription(Bool, "/BASESTATION/" + CONSTANTS.N64.NAME + "/" + CONSTANTS.N64.BUTTON.L_STR       , self.__base_forward_callback, 10)
        # self.create_subscription(Bool, "/BASESTATION/" + CONSTANTS.N64.NAME + "/" + CONSTANTS.N64.BUTTON.R_STR       , self.__base_backward_callback, 10)
        # self.create_subscription(Bool, "/BASESTATION/" + CONSTANTS.N64.NAME + "/" + CONSTANTS.N64.BUTTON.C_UP_STR    , self.__shoulder_forward_callback, 10)
        # self.create_subscription(Bool, "/BASESTATION/" + CONSTANTS.N64.NAME + "/" + CONSTANTS.N64.BUTTON.C_DOWN_STR  , self.__shoulder_backward_callback, 10)
        # self.create_subscription(Bool, "/BASESTATION/" + CONSTANTS.N64.NAME + "/" + CONSTANTS.N64.BUTTON.C_RIGHT_STR , self.__elbow_forward_callback, 10)
        # self.create_subscription(Bool, "/BASESTATION/" + CONSTANTS.N64.NAME + "/" + CONSTANTS.N64.BUTTON.C_LEFT_STR  , self.__elbow_backward_callback, 10)
        # self.create_subscription(Bool, "/BASESTATION/" + CONSTANTS.N64.NAME + "/" + CONSTANTS.N64.BUTTON.DP_DOWN_STR , self.__bend_wrist_forward_callback, 10)
        # self.create_subscription(Bool, "/BASESTATION/" + CONSTANTS.N64.NAME + "/" + CONSTANTS.N64.BUTTON.DP_UP_STR   , self.__bend_wrist_backward_callback, 10)
        # self.create_subscription(Bool, "/BASESTATION/" + CONSTANTS.N64.NAME + "/" + CONSTANTS.N64.BUTTON.DP_LEFT_STR , self.__twist_wrist_forward_callback, 10)
        # self.create_subscription(Bool, "/BASESTATION/" + CONSTANTS.N64.NAME + "/" + CONSTANTS.N64.BUTTON.DP_RIGHT_STR, self.__twist_wrist_backward_callback, 10)
        # self.create_subscription(Bool, "/BASESTATION/" + CONSTANTS.N64.NAME + "/" + CONSTANTS.N64.BUTTON.B_STR       , self.__gripper_forward_callback, 10)
        # self.create_subscription(Bool, "/BASESTATION/" + CONSTANTS.N64.NAME + "/" + CONSTANTS.N64.BUTTON.A_STR       , self.__gripper_backward_callback, 10)
        # self.create_subscription(Bool, "/BASESTATION/" + CONSTANTS.N64.NAME + "/" + CONSTANTS.N64.BUTTON.Z_STR       , self.__solenoid_callback, 10)

        self.create_subscription(
            msg_type=Bool,
            topic="/ESTOP",
            callback=self.__on_estop_received,
            qos_profile=10,
        )

        self.__bend_wrist_forward = 0
        self.__bend_wrist_backward = 0

        self.__twist_wrist_forward = 0
        self.__twist_wrist_backward = 0

        self.__gripper_forward = 0
        self.__gripper_backward = 0

        self.__solenoid = 0

    def __base_forward_callback(self, msg: Bool):
        self.__base_forward = int(msg.data)
        self.__send_controller_data(CAN_MESSAGE_IDS.MOVE_BASE)

    def __base_backward_callback(self, msg: Bool):
        self.__base_backward = int(msg.data)
        self.__send_controller_data(CAN_MESSAGE_IDS.MOVE_BASE)

    def __shoulder_forward_callback(self, msg: Bool):
        self.__shoulder_forward = int(msg.data)
        self.__send_controller_data(CAN_MESSAGE_IDS.MOVE_SHOULDER)

    def __shoulder_backward_callback(self, msg: Bool):
        self.__shoulder_backward = int(msg.data)
        self.__send_controller_data(CAN_MESSAGE_IDS.MOVE_SHOULDER)

    def __elbow_forward_callback(self, msg: Bool):
        self.__elbow_forward = int(msg.data)
        self.__send_controller_data(CAN_MESSAGE_IDS.MOVE_ELBOW)

    def __elbow_backward_callback(self, msg: Bool):
        self.__elbow_backward = int(msg.data)
        self.__send_controller_data(CAN_MESSAGE_IDS.MOVE_ELBOW)

    def __bend_wrist_forward_callback(self, msg: Bool):
        self.__bend_wrist_forward = int(msg.data)
        self.__send_controller_data(CAN_MESSAGE_IDS.BEND_WRIST)

    def __bend_wrist_backward_callback(self, msg: Bool):
        self.__bend_wrist_backward = int(msg.data)
        self.__send_controller_data(CAN_MESSAGE_IDS.BEND_WRIST)

    def __twist_wrist_forward_callback(self, msg: Bool):
        self.__twist_wrist_forward = int(msg.data)
        self.__send_controller_data(CAN_MESSAGE_IDS.TWIST_WRIST)

    def __twist_wrist_backward_callback(self, msg: Bool):
        self.__twist_wrist_backward = int(msg.data)
        self.__send_controller_data(CAN_MESSAGE_IDS.TWIST_WRIST)

    def __gripper_forward_callback(self, msg: Bool):
        self.__gripper_forward = int(msg.data)
        self.__send_controller_data(CAN_MESSAGE_IDS.MOVE_CLAW)

    def __gripper_backward_callback(self, msg: Bool):
        self.__gripper_backward = int(msg.data)
        self.__send_controller_data(CAN_MESSAGE_IDS.MOVE_CLAW)

    def __solenoid_callback(self, msg: Bool):
        self.__solenoid = int(msg.data)
        self.__send_controller_data(CAN_MESSAGE_IDS.MOVE_SOLENOID)

    def __base_number(self, num: int, base: int) -> int:
        """
        Floors a number to a certain base\n
        Ex:
        `__base_number(-1, 2) = 2`
        `__base_number(3, 2) = 3`
        """

        return num if num > base else base

    def __send_controller_data(self, ID: CAN_MESSAGE_IDS):

        # find the can_message that we are sending
        can_message = TEENSY_CAN_MESSAGES[ID]

        # if can_message not found then exit
        if can_message is None:
            return

        match ID:
            case CAN_MESSAGE_IDS.SEND_BASE: # BASE
                # Update the signals
                state = ArmState.Active if self.__base_forward | self.__base_backward else ArmState.Stop
                position = ArmDirection.Forward if self.__base_forward else ArmDirection.Backward
                can_message.signals["state"].set_value(state)
                can_message.signals["direction"].set_value(position)

            case CAN_MESSAGE_IDS.SEND_SHOULDER: # SHOULDER
                # Update the signals
                state = ArmState.Active if self.__shoulder_forward | self.__shoulder_backward else ArmState.Stop
                position = ArmDirection.Forward if self.__shoulder_forward else ArmDirection.Backward
                can_message.signals["state"].set_value(state)
                can_message.signals["direction"].set_value(position)
            case CAN_MESSAGE_IDS.SEND_ELBOW: # ELBOW
                # Update the signals
                state = ArmState.Active if self.__elbow_forward | self.__elbow_backward else ArmState.Stop
                position = ArmDirection.Forward if self.__elbow_forward else ArmDirection.Backward
                can_message.signals["state"].set_value(state)
                can_message.signals["direction"].set_value(position)
            case CAN_MESSAGE_IDS.BEND_WRIST: # BEND WRIST
                # Update the signals
            case CAN_MESSAGE_IDS.TWIST_WRIST: # TWIST WRIST
                # Update the signals
            case CAN_MESSAGE_IDS.MOVE_CLAW: # CLAW
                # Update the signals
                state = ArmState.Active if self.__gripper_forward | self.__gripper_backward else ArmState.Stop
                position = ArmDirection.Forward if self.__gripper_forward else ArmDirection.Backward
                can_message.signals["state"].set_value(state)
                can_message.signals["direction"].set_value(position)
            case CAN_MESSAGE_IDS.MOVE_SOLENOID: # SOLENOID
                # Update the signals
                can_message.signals["enabled"].set_value(self.__solenoid)
            case _:
                self.get_logger().info(f"{ID} not accounted for...")

        Can_packet = TeensyCommunication.encode_can_message(can_message)

        if Can_packet is None:
            self.get_logger().error("Failed to encode CAN message.")
            return
        else:
            # self.get_logger().info(f"Sending CAN message: {can_message.name} with state {can_message.signals['state'].value} and direction {can_message.signals['direction'].value}")
            self.__publishers[ID].publish(Can_packet)

    def __on_estop_received(self, msg: Bool):
        self.get_logger().info("E-STOP received, stopping arm...")

        rclpy.shutdown()


    def run(self):
        self.get_logger().info("starting arm...")
        rclpy.spin(self)
        self.get_logger().info("stopping arm...")

def main():
    rclpy.init()
    arm = Arm()
    arm.run()
