"""ROS node that owns the arm's CAN hardware link."""

import math
import threading
import time

from arm_constants import (
    ARM_ENABLED_TOPIC, ARM_MOTOR_IDX,
    # RESTING DISABLED: the /ARM/HARDWARE/RESTING latch subscriber below is
    # commented out, so the topic constant is unused here.
    # ARM_RESTING_TOPIC,
    ARM_TORQUE_ENABLED_TOPIC, ARM_TORQUE_STATE_TOPIC,
    CURRENT_ANGLE_TOPICS, CURRENT_VELOCITY_TOPICS, DT,
    ESTOP_TOPIC, RESTING_LATCH_TIMEOUT_SEC, RESTING_LIMITS,
    SOLENOID_TARGET_TOPIC, STANDARD_CAN_ANGLE_SIGNALS,
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

# Measured angles are only published when they change, but a late subscriber
# (the route planner or telemetry uplink starting after the arm) would then
# never see an unchanged joint.  Refresh every finite measured angle at 1 Hz so
# constant feedback, notably the simulated Teensy's pinned wrists, still
# reaches every consumer and lets the planner initialize.
FEEDBACK_REFRESH_CYCLES = 20


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
        self.__torque_ignored_logged = False
        # Resting latches: widened target limits and removed drive torque.
        # The resting latch times out if the planner goes silent; the torque
        # latch is sticky and clears only on an explicit re-enable.
        self.__resting = False
        self.__resting_at = 0.0
        self.__torque_off = False
        self.__velocity_toggle = False
        self.__feedback_refresh_counter = 0
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
        # RESTING DISABLED: do not subscribe to the resting hardware latch.
        # With no subscriber the latch level can never become active, so
        # the widened resting limits and torque-off path stay unused.
        # Re-enable by un-commenting this subscription.
        # self.create_subscription(
        #     msg_type=Bool,
        #     topic=ARM_RESTING_TOPIC,
        #     callback=self.__on_resting_received,
        #     qos_profile=10,
        # )
        self.create_subscription(
            msg_type=Bool,
            topic=ARM_TORQUE_ENABLED_TOPIC,
            callback=self.__on_torque_enabled_received,
            qos_profile=10,
        )

        # Request OpenCAN positions at the controller rate.  The feedback
        # path used to run at 2 Hz, which left the 20 Hz controller working
        # from stale angles for several cycles.
        self.create_timer(
            timer_period_sec=DT,
            callback=self.request_position,
        )
        self.__torque_state_publisher.publish(Bool(data=not self.__torque_off))

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

        # velocity feedback for the OpenCAN joints (resting stall signal)
        self.__velocity_publishers = {}
        for motor_idx, topic in CURRENT_VELOCITY_TOPICS.items():
            self.__velocity_publishers[motor_idx] = self.create_publisher(
                msg_type=Float32,
                topic=topic,
                qos_profile=10,
            )

        self.__torque_state_publisher = self.create_publisher(
            msg_type=Bool,
            topic=ARM_TORQUE_STATE_TOPIC,
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

        # The resting torque latch suppresses every OpenCAN target until the
        # restore explicitly re-enables torque.
        if self.__torque_off:
            return

        # check if the target angle is within the limits of the motor
        if (self.__resting
                and motor_idx in (ARM_MOTOR_IDX.SHOULDER,
                                  ARM_MOTOR_IDX.ELBOW)):
            lower, upper = RESTING_LIMITS[motor_idx]
        else:
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
        """Latch or clear the E-STOP and hold the arm's measured pose."""
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
        self.get_logger().info("E-STOP latched: holding measured arm pose")

        # Lock the current pose instead of removing torque.  Send each
        # measured position as the new target while the arm is still
        # enabled, so the drives hold where they are and a later re-enable
        # cannot resume a stale in-flight target.
        self.__forget_targets()
        self.__target_measured_positions()
        self.arm_enabled = False

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
            # A disable (for example the arm Teensy heartbeat timing out)
            # must not remove OpenCAN torque: the arm would drop under
            # gravity.  Hold the measured pose and only stop accepting new
            # targets until the master re-enables the arm.
            self.__target_measured_positions()
            self.arm_enabled = False
            self.__forget_targets()
            self.get_logger().info(
                "Arm disabled; holding measured pose "
                "(OpenCAN torque retained)")
            return

        # The master republishes this level while the arm is enabled.  Re-send
        # the enable/speed frames each time so a frame dropped during an
        # E-STOP cannot leave the motors latched off.
        if self.__torque_off:
            if not self.__torque_ignored_logged:
                self.__torque_ignored_logged = True
                self.get_logger().warning(
                    "Ignoring arm enable while resting torque is latched")
            return

        if not self.arm_enabled:
            self.arm_enabled = True
            # Pull a fresh position sample before the first post-enable
            # command: while disabled the measured angles can be stale, and
            # the controller must not start from old feedback.
            self.request_position()
            self.get_logger().info("Arm enabled")
        self.__open_can.set_enabled(True)

    def __on_resting_received(self, msg: Bool):
        """Widen the shoulder/elbow target limits while resting is active.

        The planner refreshes the level at 1 Hz; the latch times out if the
        planner goes silent so a dead node cannot keep the widened limits.
        """
        self.__resting = bool(msg.data)
        self.__resting_at = time.monotonic()

    def __on_torque_enabled_received(self, msg: Bool):
        """Remove or restore OpenCAN torque (resting latch, sticky off)."""
        if not msg.data:
            if self.__torque_off:
                return
            self.__torque_off = True
            self.get_logger().info("Resting torque latch: motors off")
            self.__forget_targets()
            self.__open_can.set_enabled(False)
            self.__publish_torque_state()
            return

        if self.__estop:
            self.get_logger().warning(
                "Ignoring torque enable while E-STOP is latched")
            return
        if not self.__torque_off:
            return
        self.__torque_off = False
        self.get_logger().info("Resting torque latch: restoring motors")
        # Re-enable at the measured positions, and only when the master has
        # the arm enabled: the torque level never enables a disabled arm.
        if self.arm_enabled:
            self.__target_measured_positions()
            self.__open_can.set_enabled(True)
        self.__publish_torque_state()

    def __target_measured_positions(self):
        """Send each measured position as the target.

        Used before re-enabling after a resting torque-off, and by the
        E-STOP handler to lock the current pose.
        """
        for motor_idx in ARM_MOTOR_IDX:
            measured = self.__motors_actual_angles[motor_idx]
            if math.isfinite(measured):
                self.__motors_target_angles[motor_idx] = float("nan")
                self.__send_motor_command(motor_idx, float(measured))

    def __publish_torque_state(self):
        """Report the actual torque latch state for diagnostics."""
        self.__torque_state_publisher.publish(
            Bool(data=not self.__torque_off))

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
            if angle is None:
                # Not a position response: try the velocity object (0x606C)
                # the resting push polls.  decode_velocity rejects frames
                # whose object index does not match.
                velocity = self.__open_can.decode_velocity(can_message)
                if velocity is not None:
                    self.__velocity_publishers[motor_idx].publish(
                        Float32(data=velocity))
                return
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
        available to the controller.  While resting, the shoulder and elbow
        velocity objects (0x606C) are polled too, staggered on alternate
        cycles so they never collide with the position reads in one cycle.
        """
        self.__open_can.request_positions()
        self.__feedback_refresh_counter = (
            (self.__feedback_refresh_counter + 1) % FEEDBACK_REFRESH_CYCLES)
        if self.__feedback_refresh_counter == 0:
            self.__publish_measured_angles()
        if self.__resting:
            if (time.monotonic() - self.__resting_at
                    > RESTING_LATCH_TIMEOUT_SEC):
                self.__resting = False
                self.get_logger().warning(
                    "Resting latch timed out; normal target limits restored")
            else:
                self.__velocity_toggle = not self.__velocity_toggle
                self.__open_can.request_velocity(
                    ARM_MOTOR_IDX.SHOULDER if self.__velocity_toggle
                    else ARM_MOTOR_IDX.ELBOW)

    def __publish_measured_angles(self):
        """Republish the latest measured angles for late subscribers.

        ``__on_new_CAN_message_received`` suppresses unchanged values, so a
        consumer that starts after the arm would otherwise never initialize
        from a stationary joint.
        """
        for motor_idx in ARM_MOTOR_IDX:
            angle = self.__motors_actual_angles[motor_idx]
            if math.isfinite(angle):
                self.__angle_publishers[motor_idx].publish(
                    Float32(data=angle))

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
