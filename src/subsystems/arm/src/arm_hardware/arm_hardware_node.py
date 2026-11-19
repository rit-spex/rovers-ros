"""ROS node that owns the arm's CAN hardware link."""

import math
import threading

from arm_constants import (
    ARM_ENABLED_TOPIC, ARM_MOTOR_IDX, CURRENT_ANGLE_TOPICS, DT,
    ESTOP_TOPIC, SOLENOID_TARGET_TOPIC, STANDARD_CAN_ANGLE_SIGNALS,
    STANDARD_CAN_READ_MOTORS, TARGET_ANGLE_TOPICS, angle_limits,
)
from constants.CAN_constants import (
    CAN_MESSAGE_IDS,
    TEENSY_CAN_MESSAGES,
    Subsystems_Names,
)
from constants.CAN_structs import Message
from constants.can_encoding import TeensyCommunication
from custom_interfaces.msg import Can

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import Bool, Float32

from .open_can_handler import OpenCanHandler


class ArmHardwareNode(Node):
    """Bridge the arm ROS topics to the motor backends over CAN."""

    __open_can: OpenCanHandler
    __angle_publishers: dict[ARM_MOTOR_IDX, Publisher]
    __can_publishers: dict[CAN_MESSAGE_IDS, Publisher]
    __can_lock: threading.RLock

    # The arm MUST be enabled to move. This is a safety feature that
    # prevents unexpected arm motion.
    arm_enabled: bool
    __motors_target_angles: list[float]
    __motors_actual_angles: list[float]

    def __init__(self):
        """Set up arm state, topic wiring, and the position request timer."""
        super().__init__("arm_hardware_node")

        self.arm_enabled = False
        self.__estop = False
        self.__enable_ignored_logged = False
        self.__motors_target_angles = [float("nan")] * len(ARM_MOTOR_IDX)
        self.__motors_actual_angles = [float("nan")] * len(ARM_MOTOR_IDX)

        # shared outgoing frames are mutated and published under one lock
        self.__can_publishers = {}
        self.__can_lock = threading.RLock()

        self.__open_can = OpenCanHandler(self)

        self.set_arm_motor_publishers()
        self.set_arm_motor_subscribers()

        # arm state and e-stop stay with the node lifecycle
        self.create_subscription(
            msg_type=Bool,
            topic=ARM_ENABLED_TOPIC,
            callback=self.__on_arm_enable_received,
            qos_profile=10,
        )
        self.create_subscription(
            msg_type=Bool,
            topic=ESTOP_TOPIC,
            callback=self.__on_estop_received,
            qos_profile=10,
        )

        # Request OpenCAN positions at the controller rate.  The feedback
        # path used to run at 2 Hz, which left the 20 Hz controller working
        # from stale angles for several cycles.
        self.create_timer(
            timer_period_sec=DT,
            callback=self.request_position,
        )

    def set_arm_motor_publishers(self):
        """Create the CAN and joint-state publishers for the arm motors."""
        # one publisher for every outgoing arm CAN message
        for message_id, message in TEENSY_CAN_MESSAGES.items():
            if message.subsystem == Subsystems_Names.ARM and not message.isforJetson:
                self.__can_publishers[message_id] = self.create_publisher(
                    msg_type=Can,
                    topic=message.topic_name,
                    qos_profile=10,
                )

        # create publishers for the arm motor angles
        self.__angle_publishers = {}
        for motor_idx, topic in CURRENT_ANGLE_TOPICS.items():
            self.__angle_publishers[motor_idx] = self.create_publisher(
                msg_type=Float32,
                topic=topic,
                qos_profile=10,
            )

    def set_arm_motor_subscribers(self):
        """Create the CAN and joint-target subscriptions for the arm motors."""
        # subscribers for the CAN messages coming back from the arm motors
        for message_id, message in TEENSY_CAN_MESSAGES.items():
            if message.subsystem == Subsystems_Names.ARM and message.isforJetson:
                self.create_subscription(
                    msg_type=Can,
                    topic=message.topic_name,
                    callback=self.__on_new_CAN_message_received,
                    qos_profile=10,
                )

        # create subscribers for the arm motor angle commands
        self.create_subscription(
            msg_type=Float32,
            topic=TARGET_ANGLE_TOPICS[ARM_MOTOR_IDX.BASE],
            callback=self.__base_callback,
            qos_profile=10,
        )
        self.create_subscription(
            msg_type=Float32,
            topic=TARGET_ANGLE_TOPICS[ARM_MOTOR_IDX.SHOULDER],
            callback=self.__shoulder_callback,
            qos_profile=10,
        )
        self.create_subscription(
            msg_type=Float32,
            topic=TARGET_ANGLE_TOPICS[ARM_MOTOR_IDX.ELBOW],
            callback=self.__elbow_callback,
            qos_profile=10,
        )
        self.create_subscription(
            msg_type=Float32,
            topic=TARGET_ANGLE_TOPICS[ARM_MOTOR_IDX.BEND_WRIST],
            callback=self.__wrist_bend_callback,
            qos_profile=10,
        )
        self.create_subscription(
            msg_type=Float32,
            topic=TARGET_ANGLE_TOPICS[ARM_MOTOR_IDX.TWIST_WRIST],
            callback=self.__wrist_twist_callback,
            qos_profile=10,
        )
        self.create_subscription(
            msg_type=Float32,
            topic=TARGET_ANGLE_TOPICS[ARM_MOTOR_IDX.GRIPPER],
            callback=self.__gripper_callback,
            qos_profile=10,
        )
        self.create_subscription(
            msg_type=Bool,
            topic=SOLENOID_TARGET_TOPIC,
            callback=self.__solenoid_strike_callback,
            qos_profile=10,
        )

    @property
    def can_lock(self) -> threading.RLock:
        """Lock guarding mutation and publishing of shared CAN frames."""
        return self.__can_lock

    def publish_can(self, can_message: Message) -> bool:
        """Encode and publish one arm CAN frame."""
        if can_message is None:
            return False

        with self.__can_lock:
            packet = TeensyCommunication.encode_can_message(can_message)
            if packet is None:
                self.get_logger().error("Failed to encode CAN message")
                return False

            publisher = self.__can_publishers.get(can_message.id)
            if publisher is None:
                self.get_logger().error(
                    f"No publisher for {can_message.name}")
                return False

            publisher.publish(packet)
            return True

    def decode_can_packet(self, msg: Can) -> Message | None:
        """Decode an incoming CAN packet into a Message."""
        try:
            return TeensyCommunication.decode_can_packet(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to decode CAN message: {e}")
            return None

    def __send_standard_angle(self, motor_idx: ARM_MOTOR_IDX, angle: float):
        """Send one wrist or gripper joint to *angle* [rad]."""
        message_id, signal_name = STANDARD_CAN_ANGLE_SIGNALS[motor_idx]
        can_message = TEENSY_CAN_MESSAGES.get(message_id)
        if can_message is None:
            self.get_logger().error(f"Unknown CAN message {message_id}")
            return

        with self.__can_lock:
            try:
                can_message.signals[signal_name].set_value(angle)
            except KeyError:
                self.get_logger().error(
                    f"{can_message.name} has no {signal_name} signal")
                return
            self.publish_can(can_message)

    def __decode_standard_angle(self, can_message: Message) -> float | None:
        """Return the measured angle [rad] from a position frame, else None."""
        try:
            return float(can_message.signals["Position"].value)
        except KeyError:
            return None

    def __base_callback(self, msg: Float32):
        """Send the requested base angle to the motor."""
        self.__send_motor_command(ARM_MOTOR_IDX.BASE, msg.data)

    def __shoulder_callback(self, msg: Float32):
        """Send the requested shoulder angle to the motor."""
        self.__send_motor_command(ARM_MOTOR_IDX.SHOULDER, msg.data)

    def __elbow_callback(self, msg: Float32):
        """Send the requested elbow angle to the motor."""
        self.__send_motor_command(ARM_MOTOR_IDX.ELBOW, msg.data)

    def __wrist_bend_callback(self, msg: Float32):
        """Send the requested wrist bend angle to the motor."""
        self.__send_motor_command(ARM_MOTOR_IDX.BEND_WRIST, msg.data)

    def __wrist_twist_callback(self, msg: Float32):
        """Send the requested wrist twist angle to the motor."""
        self.__send_motor_command(ARM_MOTOR_IDX.TWIST_WRIST, msg.data)

    def __gripper_callback(self, msg: Float32):
        """Send the requested gripper position to the gripper motor."""
        self.__send_motor_command(ARM_MOTOR_IDX.GRIPPER, msg.data)

    def __solenoid_strike_callback(self, msg: Bool):
        """Fire one solenoid strike.

        This is a command, not a state: the Teensy energizes and retracts
        the solenoid on its own, so only the trigger pulse is sent and the
        rover never commands it back off.
        """
        if not msg.data or not self.arm_enabled:
            return

        can_message = TEENSY_CAN_MESSAGES.get(CAN_MESSAGE_IDS.MOVE_SOLENOID)
        if can_message is None:
            self.get_logger().error("MOVE_SOLENOID is not defined")
            return

        with self.__can_lock:
            try:
                can_message.signals["Enabled"].set_value(1)
            except KeyError:
                self.get_logger().error(
                    f"{can_message.name} has no Enabled signal")
                return
            self.publish_can(can_message)
        self.get_logger().debug("Solenoid strike frame sent")

    def __forget_targets(self):
        """Drop cached targets so a re-enable cannot resume stale motion.

        The cache exists only to suppress duplicate frames.  After an
        E-stop or a disable there is no current target any more, so the next
        target the planner publishes must be sent even when it matches the
        value that was in flight before the stop.
        """
        self.__motors_target_angles = [float("nan")] * len(ARM_MOTOR_IDX)

    def __send_motor_command(
        self, motor_idx: ARM_MOTOR_IDX, target_angle: float
    ):
        """Validate and send an angle target for one motor."""
        # only send message if arm is enabled
        if not self.arm_enabled:
            return

        # check if the target angle is within the limits of the motor
        lower, upper = angle_limits(motor_idx)
        if target_angle < lower or target_angle > upper:
            self.get_logger().error(
                f"Target angle {target_angle:.4f} rad for motor "
                f"{motor_idx.name} is out of limits "
                f"[{lower:.4f}, {upper:.4f}] rad"
            )
            return

        # reject message that are the same as current
        if self.__motors_target_angles[motor_idx] == target_angle:
            return

        # update the target angle of the motor
        self.__motors_target_angles[motor_idx] = target_angle

        # send the angle through the backend that owns this motor
        if self.__open_can.handles(motor_idx):
            self.__open_can.send_angle(motor_idx, target_angle)
        elif motor_idx in STANDARD_CAN_ANGLE_SIGNALS:
            self.__send_standard_angle(motor_idx, target_angle)
        else:
            self.get_logger().error(
                f"No driver for {motor_idx.name}")

    def __on_estop_received(self, msg: Bool):
        """Latch or clear the E-STOP and disable the arm on stop."""
        # /ESTOP is level-triggered: True stops the arm, False clears the
        # latch. The node stays alive so the arm can resume after a
        # transient heartbeat loss.
        if not msg.data:
            if not self.__estop:
                return
            self.__estop = False
            self.__enable_ignored_logged = False
            self.get_logger().info("E-STOP cleared: arm can be re-enabled")
            return

        if self.__estop:
            return
        self.__estop = True
        self.get_logger().info("E-STOP latched: disabling arm motors")
        self.arm_enabled = False
        # Forget the in-flight targets: a re-enable must not inherit motion
        # that was interrupted by the stop.
        self.__forget_targets()

        # Turn off the arm immediately by sending disable messages to all the
        # OpenCAN motors.
        self.__open_can.set_enabled(False)

    def __on_arm_enable_received(self, msg: Bool):
        """Enable or disable the motors, honouring the E-STOP latch."""
        # An E-STOP latches the arm off; ignore enable requests until the
        # level clears so a stray enable cannot move the arm.
        if self.__estop and msg.data:
            if not self.__enable_ignored_logged:
                self.__enable_ignored_logged = True
                self.get_logger().warning(
                    "Ignoring arm enable while E-STOP is latched")
            return

        if not msg.data:
            if not self.arm_enabled:
                return
            self.arm_enabled = False
            self.__forget_targets()
            self.get_logger().info("Arm disabled")
            self.__open_can.set_enabled(False)
            return

        # The master republishes this level while the arm is enabled.  Re-send
        # the enable/speed frames each time so a frame dropped during an
        # E-STOP cannot leave the motors latched off.
        if not self.arm_enabled:
            self.arm_enabled = True
            self.get_logger().info("Arm enabled")
        self.__open_can.set_enabled(True)

    def __on_new_CAN_message_received(self, msg: Can):
        """Decode a motor frame and publish the reported angle."""
        # Feedback is published even while the arm is disabled so the
        # controller can initialize and track the real pose.
        # find the motor that owns this feedback message
        resolved = self.__open_can.resolve_message(msg.id)
        if resolved is not None:
            motor_idx = resolved[0]
        else:
            motor_idx = STANDARD_CAN_READ_MOTORS.get(msg.id)

        if motor_idx is None:
            self.get_logger().warning(
                f"Ignoring CAN feedback with unknown ID {msg.id}")
            return

        can_message = self.decode_can_packet(msg)
        if can_message is None:
            return

        # decode with the backend that owns the motor
        if self.__open_can.handles(motor_idx):
            angle = self.__open_can.decode_angle(motor_idx, can_message)
        else:
            angle = self.__decode_standard_angle(can_message)
        if angle is None:
            return

        # do not update if the same value, to avoid spamming the topic
        if angle == self.__motors_actual_angles[motor_idx]:
            return

        self.__motors_actual_angles[motor_idx] = angle
        self.__angle_publishers[motor_idx].publish(Float32(data=angle))
        self.get_logger().debug(
            f"{motor_idx.name} angle {math.degrees(angle):.1f} deg")

    def request_position(self):
        """Ask the OpenCAN motors for their current positions.

        This runs even while the arm is disabled so measured angles stay
        available to the controller.
        """
        self.__open_can.request_positions()

    def destroy_node(self):
        """Disable the motors before shutting the node down."""
        # Best effort: disable the OpenCAN joints if this node goes down
        # alone. A full stack shutdown is also covered by can_comms.
        try:
            self.arm_enabled = False
            self.__open_can.set_enabled(False)
        except Exception:
            pass
        return super().destroy_node()

    def run(self):
        """Spin the node until it is shut down."""
        self.get_logger().info("Arm hardware started")
        rclpy.spin(self)
        self.get_logger().info("Arm hardware stopped")


def main():
    """Create and run the arm hardware node."""
    rclpy.init(signal_handler_options=SignalHandlerOptions.ALL)
    motors = None
    try:
        motors = ArmHardwareNode()
        motors.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if motors is not None:
            try:
                motors.destroy_node()
            except Exception:
                pass
        rclpy.try_shutdown()
