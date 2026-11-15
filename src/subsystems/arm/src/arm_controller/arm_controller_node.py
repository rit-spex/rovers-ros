"""Arm controller node.

This node replaces the old, standalone ``linear_algebra``/``math_helpers``
implementation with the full controller from ``utils/simulated_arm.py``
(the core of ``armSimulator4``), while keeping the rover's ROS topics:

    inputs  : /ESTOP
              /ARM/CMD_VELOCITY       (custom_interfaces/ArmVelocity)
              /ARM/CMD_GRIPPER_AXIS   (std_msgs/Float32)
              /ARM/JOINT/CURRENT_{BASE,SHOULDER,ELBOW,WRIST_BEND,WRIST_TWIST,GRIPPER}
              /ARM/COMMAND            (std_msgs/String, see below)
    outputs : /ARM/JOINT/TARGET_{BASE,SHOULDER,ELBOW,WRIST_BEND,WRIST_TWIST,GRIPPER}
              /ARM/JOINT/TARGET_SOLENOID  (Bool, one pulse per strike)
              /ARM/STATE/{HOMING,FLIPPING,FLIPPED,PLANAR,JOINT_DRIVE,
                          POSITION_LOCK,ORIENTATION_LOCK,GRIPPER_DRIVE}
              /ARM/STATE/{LOCKED_JOINTS,HOMING_JOINTS,OPERATION_MODE,ERROR}

SpaceMouse controls (matching armSimulator4):
    Position drive (default):
        x/y/z   translate the target frame in world space.
        rx/ry/rz rotate the target frame; the IK solver follows it.
    Joint drive (``joint_drive`` command):
        rx -> elbow, ry -> shoulder, rz -> base.

Every other armSimulator4 action is available on ``/ARM/COMMAND`` as a
``std_msgs/String``.  One command per message::

    flip                    # side flip (normal <-> flipped)
    planar                  # toggle the base-locked planar modifier
                            # (also accepted as toggle_planar)
    pnp_normal              # return to Pick & Place normal side
    toggle_flipped          # direct side modifier (no animation)
    homing                  # toggle homing
    joint_drive             # toggle joint drive
    position_drive          # select position drive
    toggle_position_lock    # lock target translation
    toggle_orientation_lock # lock target rotation
    guide_orientation       # snap target to nearest axis-aligned rotation
    toggle_joint_lock:<n>   # n = 0..4 (base, shoulder, elbow, bend, twist)
    home_joint:<n>          # home a single joint, n = 0..4
    gripper_drive           # toggle node-level gripper control (ry axis)
    fire_solenoid           # fire one solenoid strike (toggle_solenoid is
                            # accepted as a legacy alias)

The 5-DOF controller starts uninitialized: joint angles are NaN until all
five measured motor angles arrive on ``/ARM/JOINT/CURRENT_*`` (from real
hardware, or from the Gazebo bridge in simulation).  It then runs at
``DT`` (20 Hz).  Measured joint angles are fed back with
``RoverArmController.sync_angles`` every cycle, and the rate-limited
``RoverArmController.angles`` are published as hardware targets so the arm
never receives a target jump larger than ``MAX_ANG_VEL * DT``.
The gripper joint is not part of the 5-DOF model; it keeps its old behavior
(track the measured angle and optionally drive it with the ``ry`` axis).
"""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from custom_interfaces.msg import ArmVelocity
from std_msgs.msg import Bool, Float32, String, UInt8, UInt32

from constants import (
    ARM_COMMAND_TOPIC, ARM_JOINT_NAMES, ARM_OPERATION_MODES,
    ARM_STATE_TOPICS, DT, ESTOP_TOPIC, GRIPPER_AXIS_TOPIC,
    GRIPPER_MAX_ANGLE, GRIPPER_MIN_ANGLE, JOINT_CURRENT_TOPICS,
    JOINT_TARGET_TOPICS, METERS_PER_INCH,
    SOLENOID_TARGET_TOPIC, VELOCITY_TOPIC,
)
from rover_protocol.arm_errors import (
    ARM_ERROR_ESTOP,
    ARM_ERROR_NONE,
    ARM_ERROR_NOT_READY,
    ARM_ERROR_UNKNOWN_COMMAND,
    classify_failure,
    error_text,
)

from .utils.generic_frames import wrap2pi
from .utils.simulated_arm import RoverArmController

# Old node used a finer scale for the gripper than the ported joint scale.
GRIPPER_SENS = 0.0007

# Stop applying a velocity once the teleop node has gone quiet.
VELOCITY_TIMEOUT_SEC = 0.5


def wrap_to_minus_90(angle: float) -> float:
    """Wrap an angle to [-pi/2, 3pi/2)."""
    return (angle + math.pi / 2) % (2 * math.pi) - math.pi / 2


class ArmControllerNode(Node):
    """ROS node driving the physical arm through ``RoverArmController``."""

    def __init__(self):
        super().__init__("arm_controller_node")

        # The controller is built from measured motor angles once all five
        # arm joints have reported in.  Angles stay NaN (uninitialized)
        # until a real motor, or the Gazebo bridge, publishes them.
        self.__arm = None

        # Measured joint angles (nan until the first hardware update).
        # These parameters are isolated due to the very different control mechanisms
        self.__measured = [float("nan")] * 5
        self.__measured_gripper = float("nan")

        # Raw SpaceMouse axis values, latched until the next message.
        # Latest normalized command from arm_teleop_node.
        self.__velocity = None
        self.__velocity_received_at = 0.0
        self.__gripper_axis = 0.0

        # Node-level state (not owned by the 5-DOF controller).
        self.__estop = False
        self.__gripper_drive = False
        self.__gripper_target = float("nan")
        self.__log_divider = 0
        self.__last_status_key = None
        self.__last_targets = None
        self.__publish_counter = 0
        # Last published target for each joint.  Locked joints hold this
        # value instead of echoing noisy measured angles straight back.
        self.__locked_targets = [None] * 5
        # Last command rejection: packed as (seq << 16) | (code << 8) | arg.
        self.__error_seq = 0
        self.__last_error = None

        # ROS interfaces and update timer.
        self.arm_controller_joint_publishers()
        self.arm_controller_joint_subscribers()
        self.arm_controller_state_publishers()
        self.arm_controller_state_subscribers()
        self.create_timer(DT, self.__update)

    def arm_controller_joint_publishers(self):
        """Create the joint command and feedback publishers."""
        self.__base_angle_publisher = self.create_publisher(
            Float32, JOINT_TARGET_TOPICS["base"], 10)
        self.__shoulder_angle_publisher = self.create_publisher(
            Float32, JOINT_TARGET_TOPICS["shoulder"], 10)
        self.__elbow_angle_publisher = self.create_publisher(
            Float32, JOINT_TARGET_TOPICS["elbow"], 10)
        self.__wrist_bend_angle_publisher = self.create_publisher(
            Float32, JOINT_TARGET_TOPICS["wrist_bend"], 10)
        self.__wrist_twist_angle_publisher = self.create_publisher(
            Float32, JOINT_TARGET_TOPICS["wrist_twist"], 10)
        self.__gripper_angle_publisher = self.create_publisher(
            Float32, JOINT_TARGET_TOPICS["gripper"], 10)
        self.__solenoid_publisher = self.create_publisher(
            Bool, SOLENOID_TARGET_TOPIC, 10)

    def arm_controller_joint_subscribers(self):
        """Create the measured joint angle subscriptions."""
        for joint in ARM_JOINT_NAMES:
            self.create_subscription(
                Float32,
                JOINT_CURRENT_TOPICS[joint],
                self.__on_joint_angle_received(joint),
                10,
            )
        self.create_subscription(
            Float32,
            JOINT_CURRENT_TOPICS["gripper"],
            self.__on_gripper_angle_received,
            10,
        )

    def arm_controller_state_publishers(self):
        """Create the arm state and error publishers."""
        self.__locked_publisher = self.create_publisher(
            UInt8, ARM_STATE_TOPICS["locked_joints"], 10)
        self.__homing_publisher = self.create_publisher(
            UInt8, ARM_STATE_TOPICS["homing_joints"], 10)
        self.__mode_publisher = self.create_publisher(
            UInt8, ARM_STATE_TOPICS["operation_mode"], 10)
        self.__flag_publishers = {
            name: self.create_publisher(Bool, topic, 10)
            for name, topic in ARM_STATE_TOPICS["flags"].items()
        }
        # One topic keeps code/arg/seq consistent for the telemetry uplink.
        self.__error_publisher = self.create_publisher(
            UInt32, ARM_STATE_TOPICS["error"], 10)

    def arm_controller_state_subscribers(self):
        """Create the E-stop, command, velocity, and gripper subscriptions."""
        self.create_subscription(
            Bool, ESTOP_TOPIC, self.__on_estop_received, 10)
        self.create_subscription(
            msg_type=String,
            topic=ARM_COMMAND_TOPIC,
            callback=self.__on_command_received,
            qos_profile=10,
        )
        self.create_subscription(
            msg_type=ArmVelocity,
            topic=VELOCITY_TOPIC,
            callback=self.__on_velocity_received,
            qos_profile=10,
        )
        self.create_subscription(
            msg_type=Float32,
            topic=GRIPPER_AXIS_TOPIC,
            callback=self.__on_gripper_axis_received,
            qos_profile=10,
        )

    # ------------------------------------------------------------------
    # Input callbacks
    # ------------------------------------------------------------------

    def __on_estop_received(self, msg: Bool):
        estop = bool(msg.data)
        if estop == self.__estop:
            return
        self.__estop = estop
        if self.__estop:
            self.get_logger().info("E-STOP latched: arm controller paused")
            self.__publish_error(code=ARM_ERROR_ESTOP, log=False)
        else:
            self.get_logger().info("E-STOP cleared: arm controller resumed")
            self.__publish_error(code=ARM_ERROR_NONE)

    def __on_velocity_received(self, msg: ArmVelocity):
        """Latch the latest normalized velocity from the teleop node."""
        self.__velocity = msg
        self.__velocity_received_at = time.monotonic()

    def __on_gripper_axis_received(self, msg: Float32):
        """Latch the latest normalized gripper axis."""
        self.__gripper_axis = float(msg.data)

    def __publish_error(self, message: str = "", code=None, arg: int = 0,
                        force: bool = False, log: bool = True):
        """Publish the last arm rejection for the basestation banner.

        Runtime rejections (a jog into a restricted octant) repeat every
        control cycle, so identical codes are suppressed unless ``force``
        is set.  Command rejections force a new sequence number: each key
        press is a new user action and should re-show the banner.
        ``code=ARM_ERROR_NONE`` clears a latched error after the next
        successful command.  ``log=False`` updates telemetry without
        duplicating an E-STOP transition line in the terminal.
        """
        if code is None:
            code, arg = classify_failure(message)
        code = int(code) & 0xFF
        arg = int(arg) & 0xFF

        if code == ARM_ERROR_NONE:
            if self.__last_error is None:
                return
            self.__last_error = None
        elif self.__last_error == (code, arg) and not force:
            return
        else:
            self.__last_error = (code, arg)
            if log:
                self.get_logger().warning(
                    f"Arm command rejected: "
                    f"{message or error_text(code, arg)}")

        self.__error_seq = (self.__error_seq + 1) & 0xFF
        self.__error_publisher.publish(
            UInt32(data=(self.__error_seq << 16) | (code << 8) | arg))

    def __on_joint_angle_received(self, joint: str):
        index = {
            "base": 0,
            "shoulder": 1,
            "elbow": 2,
            "wrist_bend": 3,
            "wrist_twist": 4,
        }[joint]

        def callback(msg: Float32):
            self.__measured[index] = wrap2pi(msg.data)
        return callback

    def __on_gripper_angle_received(self, msg: Float32):
        self.__measured_gripper = float(msg.data)
        # Track the hardware unless the node is actively driving the gripper.
        if not self.__gripper_drive or math.isnan(self.__gripper_target):
            self.__gripper_target = self.__clamp_gripper(
                self.__measured_gripper)

    @staticmethod
    def __clamp_gripper(angle):
        return max(GRIPPER_MIN_ANGLE, min(GRIPPER_MAX_ANGLE, angle))

    def __on_command_received(self, msg: String):
        command = msg.data.strip()
        if command:
            self.__handle_command(command)

    def __handle_command(self, command: str):
        if self.__estop:
            self.__publish_error(
                f"Arm command '{command}' ignored: e-stop active",
                code=ARM_ERROR_ESTOP, force=True)
            return

        if command in ("fire_solenoid", "toggle_solenoid"):
            self.__fire_solenoid()
            return

        if self.__arm is None:
            self.__publish_error(
                f"Arm command '{command}' ignored: arm not initialized yet",
                code=ARM_ERROR_NOT_READY, force=True)
            return

        handled, _ = self.__dispatch_command(command)
        failure = self.__arm.take_failure()
        if failure:
            self.__publish_error(failure, force=True)
        elif not handled:
            self.__publish_error(
                f"Unknown arm command '{command}'",
                code=ARM_ERROR_UNKNOWN_COMMAND, force=True)
        else:
            # A successful command clears an error latched by an earlier one.
            self.__publish_error(code=ARM_ERROR_NONE)

    def __fire_solenoid(self):
        """Fire one solenoid strike.

        The Teensy drives the solenoid and retracts it on its own, so this
        is a single command pulse rather than a state the rover tracks.
        """
        self.__solenoid_publisher.publish(Bool(data=True))
        self.get_logger().info("Solenoid strike")

    def __dispatch_command(self, command: str):
        """Execute one semantic command; returns (handled, result)."""
        if command in ("sync_target", "sync"):
            self.__arm.sync_angles(
                list(self.__measured), reset_target=True)
            self.__locked_targets = [None] * 5
            self.get_logger().info(
                "Arm target synchronized to the measured pose.")
            return True, True
        if command == "flip":
            return True, self.__arm.activate_flip()
        if command in ("homing", "home"):
            return True, self.__arm.toggle_homing()
        if command == "joint_drive":
            return True, self.__arm.toggle_joint_drive()
        if command == "position_drive":
            self.__arm.set_joint_drive(False)
            return True, True
        if command in ("pnp_normal", "normal"):
            return True, self.__arm.set_pnp_normal()
        if command in ("planar", "toggle_planar"):
            return True, self.__arm.toggle_planar()
        if command in ("toggle_flipped", "flipped"):
            return True, self.__arm.toggle_flipped()
        if command == "toggle_position_lock":
            return True, self.__arm.toggle_position_lock()
        if command == "toggle_orientation_lock":
            return True, self.__arm.toggle_orientation_lock()
        if command == "guide_orientation":
            return True, self.__arm.guide_orientation()
        if command == "gripper_drive":
            self.__gripper_drive = not self.__gripper_drive
            if not self.__gripper_drive:
                self.__gripper_target = self.__measured_gripper
            self.get_logger().info(
                "Gripper drive enabled." if self.__gripper_drive
                else "Gripper drive disabled.")
            return True, True

        if ":" in command:
            action, _, argument = command.partition(":")
            try:
                index = int(argument)
            except ValueError:
                return False, False
            if action == "toggle_joint_lock" and 0 <= index < 5:
                return True, self.__arm.toggle_joint_lock(index)
            if action == "home_joint" and 0 <= index < 5:
                return True, self.__arm.home_joint(index)

        return False, False

    # ------------------------------------------------------------------
    # Control cycle
    # ------------------------------------------------------------------

    def __scaled_deltas(self):
        """Convert the latest normalized velocity into per-cycle deltas.

        The teleop node latches the raw axes and republishes continuously.
        If it goes quiet, stop applying its last command instead of
        continuing to move the arm.

        Returns ``(deltas, gripper_axis)`` or ``(None, 0.0)`` when idle or
        stale.  ``gripper_axis`` is used only by the node-level gripper
        drive.
        """
        if self.__velocity is None:
            return None, 0.0
        if (time.monotonic() - self.__velocity_received_at
                > VELOCITY_TIMEOUT_SEC):
            return None, 0.0

        velocity = self.__velocity
        linear = (velocity.linear_x, velocity.linear_y, velocity.linear_z)
        angular = (velocity.angular_x, velocity.angular_y,
                   velocity.angular_z)
        if not any(linear + angular):
            return None, 0.0

        # Linear velocity is m/s; the target frame and controller use inches.
        deltas = {
            "dx": velocity.linear_x / METERS_PER_INCH * DT,
            "dy": velocity.linear_y / METERS_PER_INCH * DT,
            "dz": velocity.linear_z / METERS_PER_INCH * DT,
            "drx": velocity.angular_x * DT,
            "dry": velocity.angular_y * DT,
            "drz": velocity.angular_z * DT,
        }
        return deltas, self.__gripper_axis

    def __update_gripper(self, axis: float):
        if math.isnan(self.__measured_gripper):
            return
        if self.__gripper_drive:
            if math.isnan(self.__gripper_target):
                self.__gripper_target = self.__measured_gripper
            self.__gripper_target += axis * GRIPPER_SENS
            self.__gripper_target = self.__clamp_gripper(
                self.__gripper_target)
        else:
            self.__gripper_target = self.__clamp_gripper(
                self.__measured_gripper)

    def __update(self):
        if self.__estop:
            return

        if self.__arm is None:
            # Wait until every arm joint has a measured angle.
            if any(math.isnan(a) for a in self.__measured):
                return
            self.__arm = RoverArmController(
                initial_angles=list(self.__measured))
            # Adopt the side (normal/flipped) that matches the measured
            # pose so we do not pull the arm across to the wrong limits.
            self.__arm._settle_side()
            angle_text = " ".join(
                f"{math.degrees(a):.1f}" for a in self.__measured)
            self.get_logger().info(
                f"Arm controller ready | measured angles {angle_text} deg")
            # Clear the "waiting for joint angles" banner now that the
            # controller exists.
            self.__publish_error(code=ARM_ERROR_NONE)

            limits = self.__arm.effective_limits
            outside = [
                index
                for index, (angle, (lo, hi))
                in enumerate(zip(self.__measured, limits))
                if angle < lo - 1e-6 or angle > hi + 1e-6
            ]
            if outside:
                self.get_logger().warning(
                    f"Measured pose outside {self.__arm.limit_mode.name} "
                    f"limits for joints {outside}; clamping")
            return

        self.__arm.sync_angles(
            list(self.__measured), reset_target=False)

        deltas, gripper_axis = self.__scaled_deltas()

        if deltas is not None:
            if self.__gripper_drive:
                # Gripper drive is exclusive, like the old node's gripper
                # mode: ry moves the gripper and the arm holds still.
                self.__update_gripper(gripper_axis)
                deltas = {key: 0.0 for key in deltas}

        consumed = self.__arm.update(deltas)
        self.__publish_counter += 1
        self.__publish_targets(force=self.__publish_counter % 20 == 0)
        self.__publish_arm_status()
        self.__report_status(consumed)

    @property
    def __arm_angles(self):
        return [float(a) for a in self.__arm.angles]

    def __publish_arm_status(self):
        """Publish per-joint lock/homing and high-level arm state."""
        locked = 0
        homing = 0
        for index, locked_joint in enumerate(self.__arm.locked_joints):
            if locked_joint:
                locked |= 1 << index
        for index, homing_joint in enumerate(self.__arm.homing_joints):
            if homing_joint:
                homing |= 1 << index

        if self.__arm.is_homing:
            mode = ARM_OPERATION_MODES["HOMING"]
        elif self.__arm.planar:
            mode = ARM_OPERATION_MODES["PLANAR"]
        elif self.__arm.flipped:
            mode = ARM_OPERATION_MODES["PNP_FLIPPED"]
        else:
            mode = ARM_OPERATION_MODES["PNP"]

        flag_values = {
            "HOMING": self.__arm.is_homing,
            "FLIPPING": self.__arm.is_flipping,
            "FLIPPED": self.__arm.flipped,
            "PLANAR": self.__arm.planar,
            "JOINT_DRIVE": self.__arm.joint_drive,
            "POSITION_LOCK": self.__arm.position_locked,
            "ORIENTATION_LOCK": self.__arm.orientation_locked,
            "GRIPPER_DRIVE": self.__gripper_drive,
        }

        self.__locked_publisher.publish(UInt8(data=locked))
        self.__homing_publisher.publish(UInt8(data=homing))
        self.__mode_publisher.publish(UInt8(data=mode))
        for name, value in flag_values.items():
            self.__flag_publishers[name].publish(Bool(data=bool(value)))

    def __publish_targets(self, force=False):
        angles = self.__arm_angles
        gripper = (None if math.isnan(self.__gripper_target)
                   else self.__gripper_target)
        targets = [
            wrap_to_minus_90(angles[0]),
            wrap_to_minus_90(angles[1]),
            wrap2pi(angles[2]),
            wrap2pi(angles[3]),
            wrap2pi(angles[4]),
        ]
        # A locked joint must not chase encoder noise.  Hold its last
        # commanded target instead of republishing the measured angle.
        for index, locked in enumerate(self.__arm.locked_joints):
            if locked and self.__locked_targets[index] is not None:
                targets[index] = self.__locked_targets[index]
            else:
                self.__locked_targets[index] = targets[index]
        payload = (*targets, gripper)
        # Publishing unchanged targets every cycle floods arm_hardware_node's log.
        # Republish periodically so a late-starting consumer still gets them.
        if payload == self.__last_targets and not force:
            return
        self.__last_targets = payload

        self.__base_angle_publisher.publish(Float32(data=payload[0]))
        self.__shoulder_angle_publisher.publish(Float32(data=payload[1]))
        self.__elbow_angle_publisher.publish(Float32(data=payload[2]))
        self.__wrist_bend_angle_publisher.publish(Float32(data=payload[3]))
        self.__wrist_twist_angle_publisher.publish(Float32(data=payload[4]))
        if gripper is not None:
            self.__gripper_angle_publisher.publish(Float32(data=gripper))

    def __report_status(self, consumed: bool):
        failure = self.__arm.take_failure()
        if failure:
            self.__publish_error(failure)

        drive = "joint" if self.__arm.joint_drive else "position"
        if self.__arm.is_homing:
            state = "homing"
        elif self.__arm.is_flipping:
            state = "flipping"
        else:
            state = f"{drive} drive"
        side = "flipped" if self.__arm.flipped else "normal"
        planar = " planar" if self.__arm.planar else ""
        limits = tuple(
            (round(math.degrees(lo), 1), round(math.degrees(hi), 1))
            for lo, hi in self.__arm.effective_limits
        )

        # Log the mode/side/limit state only when it changes so the terminal
        # is not flooded with identical lines at 1 Hz.
        state_key = (state, self.__arm.limit_mode.name, side, planar, limits)
        if state_key != self.__last_status_key:
            self.__last_status_key = state_key
            limit_text = ", ".join(
                f"{lo:.0f}..{hi:.0f}" for lo, hi in limits)
            self.get_logger().info(
                f"Arm state: {state} | {self.__arm.limit_mode.name} "
                f"({side}{planar}) | limits {limit_text} deg")

        # Periodic detail stays available at DEBUG.
        self.__log_divider = (self.__log_divider + 1) % max(
            1, round(1.0 / DT))
        if self.__log_divider:
            return
        angles = " ".join(
            f"{math.degrees(a):.1f}" for a in self.__arm.angles)
        self.get_logger().debug(
            f"Arm angles [{angles}] deg | "
            f"input={'yes' if consumed else 'idle'}")

    def run(self):
        self.get_logger().info("Arm controller started")
        rclpy.spin(self)
        self.get_logger().info("Arm controller stopped")


def main():
    rclpy.init()
    controller = None
    try:
        controller = ArmControllerNode()
        controller.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if controller is not None:
            controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
