from constants.CommandCodes import CONSTANTS
from constants.can_encoding import TeensyCommunication
from constants.CAN_constants import (
    CAN_MESSAGE_IDS,
    TEENSY_CAN_MESSAGES,
    ArmState,
    ArmDirection,
    Subsystems_Names,
)
from constants.CAN_structs import Signal, Message
from custom_interfaces.msg import Can, SpaceMouse
from custom_interfaces.msg import ArmWrist

from numpy import uint8, int32
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from std_msgs.msg import Bool, Float32
from enum import IntEnum
import threading


class OPEN_CAN:

    # these are the values for the ID signal of the CAN messages, they are used to specify the type of the message being sent to the arm motors
    class ID(IntEnum):
        ENABLE = 0
        SET_SPEED = 1
        SEND = 2
        READ = 3

    # these are the values for the MESSAGE_TYPE signal of the CAN messages, the return data from the motors
    class MESSAGE_TYPE:
        ENABLE = int(0x2B)
        SET_SPEED = int(0x23)
        SEND = int(0x23)
        READ_SEND = int(0x40)
        READ_GET = int(0x43)

    # these are the values for the OPCODE_LSB signal of the CAN messages, they are used to specify the type of the message being sent to the arm motors
    class OPCODE_LSB:
        ENABLE = int(0x40)
        SET_SPEED = int(0x81)
        SEND = int(0x7A)
        READ = int(0x64)


class ARM_MOTOR_IDX(IntEnum):
    BASE = 0
    SHOULDER = 1
    ELBOW = 2
    BEND_WRIST = 3
    TWIST_WRIST = 4
    GRIPPER = 5


class arm_motor_params:
    rad_2_ticks: float
    ticks_2_rad: float
    upper_limits_ticks: int32 | float
    lower_limits_ticks: int32
    ticks_offset: int32
    max_speed: int32

    def __init__(
        self,
        rad_2_ticks: float,
        upper_limits_ticks: int32 | float,
        lower_limits_ticks: int32,
        ticks_offset: int32,
        max_speed: int32,
    ):
        self.rad_2_ticks = rad_2_ticks
        self.ticks_2_rad = 1 / rad_2_ticks
        self.upper_limits_ticks = upper_limits_ticks
        self.lower_limits_ticks = lower_limits_ticks
        self.ticks_offset = ticks_offset
        self.max_speed = max_speed


# REAL PARAMS BELOW
# ARM_MOTOR_PARAMS = [
#     arm_motor_params(rad_2_ticks=1, upper_limits_ticks=int32(1), lower_limits_ticks=int32(-1), ticks_offset=int32(0), max_speed=int32(100)), # base
#     arm_motor_params(rad_2_ticks=1, upper_limits_ticks=int32(1), lower_limits_ticks=int32(-1), ticks_offset=int32(0), max_speed=int32(100)), # shoulder
#     arm_motor_params(rad_2_ticks=1, upper_limits_ticks=int32(1), lower_limits_ticks=int32(-1), ticks_offset=int32(0), max_speed=int32(100)), # elbow
#     arm_motor_params(rad_2_ticks=1, upper_limits_ticks=int32(1), lower_limits_ticks=int32(-1), ticks_offset=int32(0), max_speed=int32(100)), # bend wrist
#     arm_motor_params(rad_2_ticks=1, upper_limits_ticks=int32(1), lower_limits_ticks=int32(-1), ticks_offset=int32(0), max_speed=int32(100)),  # twist wrist
#     arm_motor_params(rad_2_ticks=1, upper_limits_ticks=int32(1), lower_limits_ticks=int32(-1), ticks_offset=int32(0), max_speed=int32(100))  # gripper
# ]

# https://docs.google.com/spreadsheets/d/1k4CHPq7_ftuCsQyz7srGTpdhJiTCs1ho/edit?gid=788058726#gid=788058726
ARM_MOTOR_PARAMS = [
    arm_motor_params(
        rad_2_ticks=-415000,
        upper_limits_ticks=int32(10225000),
        lower_limits_ticks=int32(8596000),
        ticks_offset=int32(9900000),
        max_speed=int32(100),
    ),  # base
    arm_motor_params(
        rad_2_ticks=-262000,
        upper_limits_ticks=int32(670700),
        lower_limits_ticks=int32(-244000),
        ticks_offset=int32(625000),
        max_speed=int32(30),
    ),  # shoulder
    arm_motor_params(
        rad_2_ticks=415000,
        upper_limits_ticks=int32(1300000 + 268000),
        lower_limits_ticks=int32(-652821 + 268000),
        ticks_offset=int32(325000 + 268000),
        max_speed=int32(100),
    ),  # elbow
    # NOTE: Following limits exist on teensy side, currently redundant, also the teensy expected floats in radians soooo yeah
    arm_motor_params(
        rad_2_ticks=1,
        upper_limits_ticks=int32(10),
        lower_limits_ticks=int32(-10),
        ticks_offset=int32(0),
        max_speed=int32(100),
    ),  # bend wrist
    arm_motor_params(
        rad_2_ticks=1,
        upper_limits_ticks=int32(20),
        lower_limits_ticks=int32(-20),
        ticks_offset=int32(0),
        max_speed=int32(100),
    ),  # twist wrist
    arm_motor_params(
        rad_2_ticks=1,
        upper_limits_ticks=15.7,
        lower_limits_ticks=int32(0),
        ticks_offset=int32(0),
        max_speed=int32(100),
    ),  # gripper
]

REQUESTED_ARM_UPDATE_RATE = 0.5  # in seconds, this is the rate at which the arm node will request updates from the arm motors, it should be at least as fast as the rate at which the arm motors update their position to ensure smooth movement of the arm


class Arm(Node):

    __publishers: dict[CAN_MESSAGE_IDS, Publisher]

    __base_angle_publisher: Publisher
    __shoulder_angle_publisher: Publisher
    __elbow_angle_publisher: Publisher
    __wrist_bend_angle_publisher: Publisher
    __wrist_twist_angle_publisher: Publisher
    __gripper_angle_publisher: Publisher
    __solenoid_state_publisher: Publisher

    # arm MUST be enabled to move, this is a safety feature to prevent the arm from moving unexpectedly
    arm_enabled: bool
    __solenoid_engaged: bool
    __motors_target_ticks: list[int32 | float]
    __motors_actual_ticks: list[int32 | float]

    # this thread will continuously request the position of the can open motors
    request_position_thread: threading.Thread

    # this is needed to prevent message object from being modified at the same time by multiple threads when sending messages to the arm
    __open_can_send_lock: threading.Lock

    def __init__(self):
        super().__init__("arm_node")

        self.__publishers = {}
        for message_id, message in TEENSY_CAN_MESSAGES.items():
            if message.subsystem == Subsystems_Names.ARM and not message.isforJetson:
                self.__publishers[message_id] = self.create_publisher(
                    msg_type=Can, topic=message.topic_name, qos_profile=10
                )

        for message_id, message in TEENSY_CAN_MESSAGES.items():
            if message.subsystem == Subsystems_Names.ARM and message.isforJetson:
                self.create_subscription(
                    msg_type=Can,
                    topic=message.topic_name,
                    callback=self.__on_new_CAN_message_received,
                    qos_profile=10,
                )

        self.create_subscription(
            msg_type=SpaceMouse,
            topic="/BASESTATION/spacemouse",
            callback=self.__on_spacemouse_received,
            qos_profile=10,
        )

        self.create_subscription(Bool, "/ARM/ENABLED", self.__on_arm_enable_received, 10)
        # create subscribers for the arm motor angle commands
        self.create_subscription(
            Float32, "/ARM/BASE/TARGET_ANGLE", self.__base_callback, 10
        )
        self.create_subscription(
            Float32, "/ARM/SHOULDER/TARGET_ANGLE", self.__shoulder_callback, 10
        )
        self.create_subscription(
            Float32, "/ARM/ELBOW/TARGET_ANGLE", self.__elbow_callback, 10
        )

        self.create_subscription(
            ArmWrist, "/ARM/WRIST/TARGET_ANGLE", self.__wrist_callback, 10
        )

        self.create_subscription(
            Float32, "/ARM/GRIPPER/TARGET_ANGLE", self.__gripper_callback, 10
        )
        self.create_subscription(
            Bool, "/ARM/SOLENOID/ENABLED", self.__solenoid_callback, 10
        )

        # create publishers for the arm motor angles
        self.__base_angle_publisher = self.create_publisher(
            Float32, "/ARM/BASE/CURR_ANGLE", 10
        )
        self.__shoulder_angle_publisher = self.create_publisher(
            Float32, "/ARM/SHOULDER/CURR_ANGLE", 10
        )
        self.__elbow_angle_publisher = self.create_publisher(
            Float32, "/ARM/ELBOW/CURR_ANGLE", 10
        )
        self.__wrist_bend_angle_publisher = self.create_publisher(
            Float32, "/ARM/WRIST_BEND/CURR_ANGLE", 10
        )
        self.__wrist_twist_angle_publisher = self.create_publisher(
            Float32, "/ARM/WRIST_TWIST/CURR_ANGLE", 10
        )
        self.__gripper_angle_publisher = self.create_publisher(
            Float32, "/ARM/GRIPPER/CURR_ANGLE", 10
        )

        self.create_subscription(
            Bool, "/ARM/ENABLED", self.__on_arm_enable_received, 10
        )

        self.create_subscription(
            msg_type=Bool,
            topic="/ESTOP",
            callback=self.__on_estop_received,
            qos_profile=10,
        )

        self.arm_enabled = False
        self.__solenoid_engaged = False
        self.__motors_target_ticks = [float("nan")] * len(ARM_MOTOR_IDX)
        self.__motors_actual_ticks = [float("nan")] * len(ARM_MOTOR_IDX)
        self.__open_can_send_lock = threading.Lock()

        # 1. Create and start a timer to continuously request the position of the arm motors at the requested update rate
        self.create_timer(
            timer_period_sec=REQUESTED_ARM_UPDATE_RATE,
            callback=self.request_position,
        )

    def __on_spacemouse_received(self, msg: SpaceMouse):
        """Handle incoming SpaceMouse 6DOF input for arm control.

        msg.x/y/z  – translation axes (-1.0 to 1.0)
        msg.rx/ry/rz – rotation axes  (-1.0 to 1.0)
        msg.buttons  – bitmask of SpaceMouse button states

        TODO: map axes to arm motors here.
        """
        if not self.arm_enabled:
            return

        self.get_logger().debug(
            "SpaceMouse: x=%.3f y=%.3f z=%.3f rx=%.3f ry=%.3f rz=%.3f buttons=%d",
            msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz, msg.buttons,
        )

    def __base_callback(self, msg: Float32):        
        self.__send_motor_command(ARM_MOTOR_IDX.BASE, int32(msg.data * ARM_MOTOR_PARAMS[ARM_MOTOR_IDX.BASE].rad_2_ticks))
    def __base_callback(self, msg: Float32):
        ticks = (
            int32(msg.data * ARM_MOTOR_PARAMS[ARM_MOTOR_IDX.BASE].rad_2_ticks)
            + ARM_MOTOR_PARAMS[ARM_MOTOR_IDX.BASE].ticks_offset
        )
        self.__send_motor_command(ARM_MOTOR_IDX.BASE, ticks)

    def __shoulder_callback(self, msg: Float32):
        ticks = (
            int32(msg.data * ARM_MOTOR_PARAMS[ARM_MOTOR_IDX.SHOULDER].rad_2_ticks)
            + ARM_MOTOR_PARAMS[ARM_MOTOR_IDX.SHOULDER].ticks_offset
        )
        self.__send_motor_command(ARM_MOTOR_IDX.SHOULDER, ticks)

    def __elbow_callback(self, msg: Float32):
        ticks = (
            int32(msg.data * ARM_MOTOR_PARAMS[ARM_MOTOR_IDX.ELBOW].rad_2_ticks)
            + ARM_MOTOR_PARAMS[ARM_MOTOR_IDX.ELBOW].ticks_offset
        )
        self.__send_motor_command(ARM_MOTOR_IDX.ELBOW, ticks)

    def __wrist_callback(self, msg: ArmWrist):
        try:
            can_message = TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS.MOVE_WRIST]
            can_message.signals["Position_Bend"].set_value(msg.wrist_bend)
            can_message.signals["Position_Twist"].set_value(msg.wrist_twist)
            self.__send_CAN_data(can_message)
        except KeyError as e:
            self.get_logger().error(
                f"CAN message for moving the wrist does not exist: {e}"
            )
            return
        
    def __gripper_callback(self, msg: Float32):
        self.__send_motor_command(ARM_MOTOR_IDX.GRIPPER, msg.data)

    def __solenoid_callback(self, msg: Bool):
        # only send message if arm is enabled
        if not self.arm_enabled:
            return

        # reject message that are the same as current
        if msg.data == self.__solenoid_engaged:
            return

        self.__solenoid_engaged = msg.data

        # try to send the message to the arm motor
        try:
            can_message = TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS.MOVE_CLAW]
            can_message.signals["enabled"].set_value(self.__solenoid_engaged)
            self.__send_CAN_data(can_message)
        except KeyError as e:
            self.get_logger().error(
                f"CAN message for moving the claw does not exist: {e}"
            )
            return

    def __send_motor_command(
        self, motor_idx: ARM_MOTOR_IDX, target_tick: int32 | float
    ):
        # only send message if arm is enabled
        if not self.arm_enabled:
            return

        self.get_logger().info(
            f"Target angle {target_tick} for motor {motor_idx.name}]"
        )

        # check if the target angle is within the limits of the motor
        if (
            target_tick < ARM_MOTOR_PARAMS[motor_idx].lower_limits_ticks
            or target_tick > ARM_MOTOR_PARAMS[motor_idx].upper_limits_ticks
        ):
            self.get_logger().error(
                f"Target angle {target_tick} for motor {motor_idx.name} is out of limits. Limits: [{ARM_MOTOR_PARAMS[motor_idx].lower_limits_ticks}, {ARM_MOTOR_PARAMS[motor_idx].upper_limits_ticks}]"
            )
            return

        # reject message that are the same as current
        if self.__motors_target_ticks[motor_idx] == target_tick:
            return

        # update the target angle of the motor
        self.__motors_target_ticks[motor_idx] = target_tick

        # send the message to the arm motor based on the motor index
        match motor_idx:
            case ARM_MOTOR_IDX.BASE:
                self.__send_open_can_message(
                    CAN_MESSAGE_IDS.SEND_BASE, int32(target_tick), OPEN_CAN.ID.SEND
                )
            case ARM_MOTOR_IDX.SHOULDER:
                self.__send_open_can_message(
                    CAN_MESSAGE_IDS.SEND_SHOULDER, int32(target_tick), OPEN_CAN.ID.SEND
                )
            case ARM_MOTOR_IDX.ELBOW:
                self.__send_open_can_message(
                    CAN_MESSAGE_IDS.SEND_ELBOW, int32(target_tick), OPEN_CAN.ID.SEND
                )
            case ARM_MOTOR_IDX.GRIPPER:
                try:
                    can_message = TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS.MOVE_CLAW]
                    can_message.signals["Position"].set_value(target_tick)
                    self.__send_CAN_data(can_message)
                except KeyError as e:
                    self.get_logger().error(
                        f"CAN message for moving the claw does not exist: {e}"
                    )
                    return

    def __send_open_can_message(
        self, message_id: CAN_MESSAGE_IDS, value: int32, message_type: OPEN_CAN.ID
    ):
        # first check if parameters are valid
        if message_id not in self.__publishers:
            self.get_logger().error(f"Invalid message ID: {message_id}")
            return

        can_message = TEENSY_CAN_MESSAGES[message_id]

        # lock before modifying the message to prevent multiple threads from modifying the message at the same time and
        with self.__open_can_send_lock:

            # set the message_type code based on the message id
            try:
                if message_type == OPEN_CAN.ID.ENABLE:
                    can_message.signals["MESSAGE_TYPE"].set_value(
                        OPEN_CAN.MESSAGE_TYPE.ENABLE
                    )
                elif message_type == OPEN_CAN.ID.SEND:
                    can_message.signals["MESSAGE_TYPE"].set_value(
                        OPEN_CAN.MESSAGE_TYPE.SEND
                    )
                elif message_type == OPEN_CAN.ID.READ:
                    can_message.signals["MESSAGE_TYPE"].set_value(
                        OPEN_CAN.MESSAGE_TYPE.READ_SEND
                    )
                elif message_type == OPEN_CAN.ID.SET_SPEED:
                    can_message.signals["MESSAGE_TYPE"].set_value(
                        OPEN_CAN.MESSAGE_TYPE.SET_SPEED
                    )
            except KeyError as e:
                self.get_logger().error(
                    f"CAN message {can_message.name} does not have a MESSAGE_TYPE signal."
                )
                return

            # set the OPCODE_LSB of the signal based on the message id
            try:
                if message_type == OPEN_CAN.ID.ENABLE:
                    can_message.signals["OPCODE_LSB"].set_value(
                        OPEN_CAN.OPCODE_LSB.ENABLE
                    )
                elif message_type == OPEN_CAN.ID.SEND:
                    can_message.signals["OPCODE_LSB"].set_value(
                        OPEN_CAN.OPCODE_LSB.SEND
                    )
                elif message_type == OPEN_CAN.ID.READ:
                    can_message.signals["OPCODE_LSB"].set_value(
                        OPEN_CAN.OPCODE_LSB.READ
                    )
                elif message_type == OPEN_CAN.ID.SET_SPEED:
                    can_message.signals["OPCODE_LSB"].set_value(
                        OPEN_CAN.OPCODE_LSB.SET_SPEED
                    )
            except KeyError as e:
                self.get_logger().error(
                    f"CAN message {can_message.name} does not have a OPCODE_LSB signal."
                )
                return

            # set the OPCODE_MSB to always be the default value
            try:
                can_message.signals["OPCODE_MSB"].reset()
            except KeyError as e:
                self.get_logger().error(
                    f"CAN message {can_message.name} does not have a OPCODE_MSB signal."
                )
                return

            # set the EMPTY to always be the default value
            try:
                can_message.signals["EMPTY"].reset()
            except KeyError as e:
                self.get_logger().error(
                    f"CAN message {can_message.name} does not have a EMPTY signal."
                )
                return

            # set the value of the signal to the provided value
            try:
                if message_type == OPEN_CAN.ID.ENABLE:
                    if (
                        value == 0
                    ):  # if value is 0 then disable the arm and set all target angles to 0
                        can_message.signals["DATA1"].set_value(0)
                    else:
                        # enable position mode and turn on
                        can_message.signals["DATA1"].set_value(int(0x2F))

                    # make sure to reset the other data signals to default values
                    can_message.signals["DATA2"].set_value(0)
                    can_message.signals["DATA3"].set_value(0)
                    can_message.signals["DATA4"].set_value(0)

                elif (
                    message_type == OPEN_CAN.ID.SEND
                    or message_type == OPEN_CAN.ID.SET_SPEED
                ):
                    can_message.signals["DATA1"].set_value(
                        (value >> 0) & 0xFF
                    )  # set DATA1 to the least significant byte of the value
                    can_message.signals["DATA2"].set_value(
                        (value >> 8) & 0xFF
                    )  # set DATA2 to the second least significant byte of the value
                    can_message.signals["DATA3"].set_value(
                        (value >> 16) & 0xFF
                    )  # set DATA3 to the third least significant byte of the value
                    can_message.signals["DATA4"].set_value(
                        (value >> 24) & 0xFF
                    )  # set DATA4 to the most significant byte of the value

                elif message_type == OPEN_CAN.ID.READ:
                    can_message.signals["DATA1"].set_value(0)
                    can_message.signals["DATA2"].set_value(0)
                    can_message.signals["DATA3"].set_value(0)
                    can_message.signals["DATA4"].set_value(0)
            except KeyError as e:
                self.get_logger().error(
                    f"CAN message {can_message.name} failed to set value for signal: {e}"
                )
                return

            # after setting up the message, send it to the arm motors
            self.__send_CAN_data(can_message)

    def __send_CAN_data(self, can_message: Message):

        # if can_message empty then exit
        if can_message is None:
            return

        Can_packet = TeensyCommunication.encode_can_message(can_message)

        if Can_packet is None:
            self.get_logger().error("Failed to encode CAN message.")
            return
        else:
            # self.get_logger().info(f"Sending CAN message: {can_message.name} with state {can_message.signals['state'].value} and direction {can_message.signals['direction'].value}")
            self.__publishers[can_message.id].publish(Can_packet)

    def __on_estop_received(self, msg: Bool):
        self.get_logger().info("E-STOP received, stopping arm...")

        # Turn off the arm immediately by sending disable messages to all the open can motors
        self.__send_open_can_message(
            CAN_MESSAGE_IDS.SEND_BASE, int32(0), OPEN_CAN.ID.ENABLE
        )
        self.__send_open_can_message(
            CAN_MESSAGE_IDS.SEND_SHOULDER, int32(0), OPEN_CAN.ID.ENABLE
        )
        self.__send_open_can_message(
            CAN_MESSAGE_IDS.SEND_ELBOW, int32(0), OPEN_CAN.ID.ENABLE
        )

        rclpy.shutdown()

    def __on_arm_enable_received(self, msg: Bool):

        # reject message that are the same as current
        if msg.data == self.arm_enabled:
            return

        self.arm_enabled = msg.data

        # if arm is being enabled
        if msg.data:
            self.__send_open_can_message(
                CAN_MESSAGE_IDS.SEND_BASE, int32(1), OPEN_CAN.ID.ENABLE
            )
            self.__send_open_can_message(
                CAN_MESSAGE_IDS.SEND_SHOULDER, int32(1), OPEN_CAN.ID.ENABLE
            )
            self.__send_open_can_message(
                CAN_MESSAGE_IDS.SEND_ELBOW, int32(1), OPEN_CAN.ID.ENABLE
            )
            self.__send_open_can_message(
                CAN_MESSAGE_IDS.SEND_BASE,
                ARM_MOTOR_PARAMS[ARM_MOTOR_IDX.BASE].max_speed,
                OPEN_CAN.ID.SET_SPEED,
            )
            self.__send_open_can_message(
                CAN_MESSAGE_IDS.SEND_SHOULDER,
                ARM_MOTOR_PARAMS[ARM_MOTOR_IDX.SHOULDER].max_speed,
                OPEN_CAN.ID.SET_SPEED,
            )
            self.__send_open_can_message(
                CAN_MESSAGE_IDS.SEND_ELBOW,
                ARM_MOTOR_PARAMS[ARM_MOTOR_IDX.ELBOW].max_speed,
                OPEN_CAN.ID.SET_SPEED,
            )
            self.get_logger().info("Enabling arm...")
        else:
            self.__send_open_can_message(
                CAN_MESSAGE_IDS.SEND_BASE, int32(0), OPEN_CAN.ID.ENABLE
            )
            self.__send_open_can_message(
                CAN_MESSAGE_IDS.SEND_SHOULDER, int32(0), OPEN_CAN.ID.ENABLE
            )
            self.__send_open_can_message(
                CAN_MESSAGE_IDS.SEND_ELBOW, int32(0), OPEN_CAN.ID.ENABLE
            )
            self.get_logger().info("Disabling arm...")

    def __on_new_CAN_message_received(self, msg: Can):
        # first try to decode the message into a CAN message object
        if self.arm_enabled == False:
            return

        can_message: Message | None
        try:
            can_message = TeensyCommunication.decode_can_packet(msg)
            if can_message is None:
                return
        except Exception as e:
            self.get_logger().error(f"Failed to decode CAN message: {e}")
            return

        match (msg.id):
            case CAN_MESSAGE_IDS.READ_BASE:
                if (
                    can_message.signals["MESSAGE_TYPE"].value
                    == OPEN_CAN.MESSAGE_TYPE.READ_GET
                    and can_message.signals["OPCODE_LSB"].value
                    == OPEN_CAN.OPCODE_LSB.READ
                ):
                    # update the actual ticks of the base motor based on the data in the message
                    data1 = can_message.signals["DATA1"].value
                    data2 = can_message.signals["DATA2"].value
                    data3 = can_message.signals["DATA3"].value
                    data4 = can_message.signals["DATA4"].value

                    actual_tick = (data4 << 24) | (data3 << 16) | (data2 << 8) | data1

                    if (
                        actual_tick >= 2**31
                    ):  # if the value is negative then convert it to a signed integer
                        actual_tick -= 2**32

                    # do not update if the same value
                    if actual_tick == self.__motors_actual_ticks[ARM_MOTOR_IDX.BASE]:
                        return

                    self.__motors_actual_ticks[ARM_MOTOR_IDX.BASE] = actual_tick
                    self.__base_angle_publisher.publish(
                        Float32(
                            data=(
                                actual_tick
                                - ARM_MOTOR_PARAMS[ARM_MOTOR_IDX.BASE].ticks_offset
                            )
                            * ARM_MOTOR_PARAMS[ARM_MOTOR_IDX.BASE].ticks_2_rad
                        )
                    )
                    self.get_logger().info(
                        f"Updated actual tick for base motor: {actual_tick}"
                    )

            case CAN_MESSAGE_IDS.READ_SHOULDER:
                if (
                    can_message.signals["MESSAGE_TYPE"].value
                    == OPEN_CAN.MESSAGE_TYPE.READ_GET
                    and can_message.signals["OPCODE_LSB"].value
                    == OPEN_CAN.OPCODE_LSB.READ
                ):
                    # update the actual ticks of the base motor based on the data in the message
                    data1 = can_message.signals["DATA1"].value
                    data2 = can_message.signals["DATA2"].value
                    data3 = can_message.signals["DATA3"].value
                    data4 = can_message.signals["DATA4"].value

                    actual_tick = (data4 << 24) | (data3 << 16) | (data2 << 8) | data1

                    if (
                        actual_tick >= 2**31
                    ):  # if the value is negative then convert it to a signed integer
                        actual_tick -= 2**32

                    # do not update if the same value
                    if (
                        actual_tick
                        == self.__motors_actual_ticks[ARM_MOTOR_IDX.SHOULDER]
                    ):
                        return

                    self.__motors_actual_ticks[ARM_MOTOR_IDX.SHOULDER] = actual_tick
                    self.__shoulder_angle_publisher.publish(
                        Float32(
                            data=(
                                actual_tick
                                - ARM_MOTOR_PARAMS[ARM_MOTOR_IDX.SHOULDER].ticks_offset
                            )
                            * ARM_MOTOR_PARAMS[ARM_MOTOR_IDX.SHOULDER].ticks_2_rad
                        )
                    )
                    self.get_logger().info(
                        f"Updated actual tick for shoulder motor: {actual_tick}"
                    )

            case CAN_MESSAGE_IDS.READ_ELBOW:
                if (
                    can_message.signals["MESSAGE_TYPE"].value
                    == OPEN_CAN.MESSAGE_TYPE.READ_GET
                    and can_message.signals["OPCODE_LSB"].value
                    == OPEN_CAN.OPCODE_LSB.READ
                ):
                    # update the actual ticks of the base motor based on the data in the message
                    data1 = can_message.signals["DATA1"].value
                    data2 = can_message.signals["DATA2"].value
                    data3 = can_message.signals["DATA3"].value
                    data4 = can_message.signals["DATA4"].value

                    actual_tick = (data4 << 24) | (data3 << 16) | (data2 << 8) | data1

                    if (
                        actual_tick >= 2**31
                    ):  # if the value is negative then convert it to a signed integer
                        actual_tick -= 2**32

                    self.get_logger().info(f"ticks {actual_tick}:)")

                    # do not update if the same value
                    if actual_tick == self.__motors_actual_ticks[ARM_MOTOR_IDX.ELBOW]:
                        return

                    self.__motors_actual_ticks[ARM_MOTOR_IDX.ELBOW] = actual_tick
                    self.__elbow_angle_publisher.publish(
                        Float32(
                            data=(
                                actual_tick
                                - ARM_MOTOR_PARAMS[ARM_MOTOR_IDX.ELBOW].ticks_offset
                            )
                            * ARM_MOTOR_PARAMS[ARM_MOTOR_IDX.ELBOW].ticks_2_rad
                        )
                    )
                    self.get_logger().info(
                        f"Updated actual tick for elbow motor: {actual_tick}"
                    )

            case CAN_MESSAGE_IDS.READ_WRIST_BEND:
                # do not update if the same value to prevent spamming the topic with the same angle which can cause issues with the arm movement
                if (
                    can_message.signals["Position"].value
                    == self.__motors_actual_ticks[ARM_MOTOR_IDX.BEND_WRIST]
                ):
                    return

                self.__motors_actual_ticks[ARM_MOTOR_IDX.BEND_WRIST] = (
                    can_message.signals["Position"].value
                )
                self.__wrist_bend_angle_publisher.publish(
                    Float32(
                        data=self.__motors_actual_ticks[ARM_MOTOR_IDX.BEND_WRIST]
                        * ARM_MOTOR_PARAMS[ARM_MOTOR_IDX.BEND_WRIST].ticks_2_rad
                    )
                )
                self.get_logger().info(
                    f"Updated actual tick for bend wrist motor: {can_message.signals['Position'].value}"
                )
            case CAN_MESSAGE_IDS.READ_WRIST_TWIST:
                if (
                    can_message.signals["Position"].value
                    == self.__motors_actual_ticks[ARM_MOTOR_IDX.TWIST_WRIST]
                ):
                    return
                self.__motors_actual_ticks[ARM_MOTOR_IDX.TWIST_WRIST] = (
                    can_message.signals["Position"].value
                )
                self.__wrist_twist_angle_publisher.publish(
                    Float32(
                        data=self.__motors_actual_ticks[ARM_MOTOR_IDX.TWIST_WRIST]
                        * ARM_MOTOR_PARAMS[ARM_MOTOR_IDX.TWIST_WRIST].ticks_2_rad
                    )
                )
                self.get_logger().info(
                    f"Updated actual tick for twist wrist motor: {can_message.signals['Position'].value}"
                )
            case CAN_MESSAGE_IDS.READ_CLAW:
                if (
                    can_message.signals["Position"].value
                    == self.__motors_actual_ticks[ARM_MOTOR_IDX.GRIPPER]
                ):
                    return
                self.__motors_actual_ticks[ARM_MOTOR_IDX.GRIPPER] = can_message.signals[
                    "Position"
                ].value
                self.__gripper_angle_publisher.publish(
                    Float32(
                        data=self.__motors_actual_ticks[ARM_MOTOR_IDX.GRIPPER]
                        * ARM_MOTOR_PARAMS[ARM_MOTOR_IDX.GRIPPER].ticks_2_rad
                    )
                )
                self.get_logger().info(
                    f"Updated actual tick for gripper: {can_message.signals['Position'].value}"
                )
            case _:
                self.get_logger().warning(
                    f"Received CAN message with unrecognized ID: {msg.id}"
                )

    def request_position(self):
        if self.arm_enabled:
            self.__send_open_can_message(
                CAN_MESSAGE_IDS.SEND_BASE, int32(0), OPEN_CAN.ID.READ
            )
            self.__send_open_can_message(
                CAN_MESSAGE_IDS.SEND_SHOULDER, int32(0), OPEN_CAN.ID.READ
            )
            self.__send_open_can_message(
                CAN_MESSAGE_IDS.SEND_ELBOW, int32(0), OPEN_CAN.ID.READ
            )

    def run(self):
        self.get_logger().info("starting arm...")
        rclpy.spin(self)
        self.get_logger().info("stopping arm...")


def main():
    rclpy.init()
    arm = Arm()
    arm.run()
