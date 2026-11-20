"""Arm route planner node.

Owns every 5-DOF target-generation path that needs inverse kinematics:
inverse kinematics, limit modes, octant gates, joint locks, teleop, and the
measured end-effector pose.  Homing, flips, zenith, and resting motion live
in ``arm_action_driver_node``; while that node reports an action the planner
stops publishing targets and mirrors the action's mode flags and locks.

It is the only writer of the five arm joint targets outside an action, so
the master node can focus on local actuators.

It also consumes teleop velocity directly.  The master node does not convert
or forward motion deltas; it only reports whether gripper drive is active so
this node can hold the arm still while the gripper moves.  Absolute Cartesian
targets (``/ARM/CMD_POSE``) take priority over teleop and are the interface
used by ``arm_typing_node``.

Inputs:
    /ARM/CMD_VELOCITY       custom_interfaces/Velocity
    /ARM/CMD_POSE           custom_interfaces/Pose
    /ARM/PLANNER/COMMAND    custom_interfaces/ArmCommand
    /ESTOP                  std_msgs/Bool
    /ARM/JOINT/CURRENT_*    std_msgs/Float32 (5-DOF joints only)

Outputs:
    /ARM/JOINT/TARGET_*     std_msgs/Float32 (5-DOF joints only)
    /ARM/STATE/{HOMING,FLIPPING,FLIPPED,PLANAR,JOINT_DRIVE,
                POSITION_LOCK,ORIENTATION_LOCK,ZENITH}
    /ARM/STATE/{LOCKED_JOINTS,HOMING_JOINTS,OPERATION_MODE}
    /ARM/STATE/POSE         custom_interfaces/Pose
    /ARM/PLANNER/STATUS     custom_interfaces/ArmPlannerStatus

The master node owns the gripper and solenoid, so this node never touches
``TARGET_GRIPPER`` or ``TARGET_SOLENOID`` and never receives their commands.
It validates the 5-DOF motion commands and classifies their failures; the
master node republishes the resulting error events on ``/ARM/STATE/ERROR``
so that topic has a single writer.
"""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from custom_interfaces.msg import (
    ArmActionStatus,
    ArmCommand,
    ArmPlannerStatus,
    Pose,
    Velocity,
)
from std_msgs.msg import Bool, Float32, Int8, UInt8

from arm_route_planner.kinematics.linkage_models import (
    rotation_from_rpy, rotation_to_rpy, wrap2pi,
)
from arm_route_planner.kinematics.arm_controller import RoverArmController
from arm_route_planner.kinematics.limiters import (
    PICK_AND_PLACE_MODE,
    RESTING_MODE,
    ZENITH_MODE,
)
from arm_route_planner.kinematics.regions import RegionTracker
from arm_constants import (
    ACTION_ACTIVE_FLIPPING,
    ACTION_ACTIVE_HOMING,
    ACTION_ACTIVE_IDLE,
    ACTION_ACTIVE_RESTING,
    ACTION_ACTIVE_TOPIC,
    ACTION_ACTIVE_TIMEOUT_SEC,
    ACTION_ACTIVE_ZENITH,
    ACTION_COMMAND_TOPIC,
    ACTION_STATUS_TOPIC,
    ARM_ENABLED_TOPIC,
    ARM_FLAG_NAMES,
    ARM_JOINT_NAMES,
    ARM_OPERATION_MODES,
    ARM_STATE_TOPICS,
    AUTONOMOUS_POSE_TIMEOUT_SEC,
    DT,
    ESTOP_TOPIC,
    JOINT_CURRENT_TOPICS,
    JOINT_TARGET_TOPICS,
    METERS_PER_INCH,
    PLANNER_COMMAND_TOPIC,
    PLANNER_HOLD_COMMAND,
    PLANNER_STATUS_TOPIC,
    POSE_TOPIC,
    VELOCITY_TIMEOUT_SEC,
    VELOCITY_TOPIC,
)
from rover_protocol.arm_errors import (
    ARM_ERROR_ESTOP,
    ARM_ERROR_NONE,
    ARM_ERROR_NOT_READY,
    ARM_ERROR_UNKNOWN_COMMAND,
    classify_failure,
    error_text,
)

# Every arm flag topic is planner state now that GRIPPER_DRIVE is gone.
PLANNER_FLAG_NAMES = ARM_FLAG_NAMES

JOINT_INDEX = {
    "base": 0,
    "shoulder": 1,
    "elbow": 2,
    "wrist_bend": 3,
    "wrist_twist": 4,
}


class ArmRoutePlannerNode(Node):
    """Compute and publish the 5-DOF arm joint targets and planner state."""

    def __init__(self):
        super().__init__("arm_route_planner_node")

        # The controller is always present; it starts at its nominal pose and
        # adopts the measured angles once all five joints have reported in.
        # A separate action driver owns homing/flip/zenith/resting motion.
        self.__arm: RoverArmController = RoverArmController()
        self.__measured = [float("nan")] * len(ARM_JOINT_NAMES)
        self.__regions = RegionTracker()

        # Latest arbitrated command.  The master only forwards teleop
        # velocity, so this node does not need to know who owns the arm.
        self.__velocity: Velocity | None = None
        self.__velocity_received_at = 0.0

        # Error events.  error_seq increments for every new event, including
        # clears, and the master node republishes them on /ARM/STATE/ERROR.
        self.__error = (ARM_ERROR_NONE, 0)
        self.__error_seq = 0

        self.__estop = False
        self.__arm_enabled = False
        self.__log_divider = 0
        self.__last_status_key: tuple[object, ...] | None = None
        self.__last_targets: tuple[float, ...] | None = None
        self.__publish_counter = 0
        self.__joint_feedback_at = [0.0] * len(ARM_JOINT_NAMES)

        # Action driver coordination.  ACTIVE is the published action enum;
        # the planner mirrors mode flags and stops publishing targets while
        # it is nonzero.
        self.__action_active = ACTION_ACTIVE_IDLE
        self.__action_active_at = 0.0
        self.__action_prev = ACTION_ACTIVE_IDLE
        self.__current_active = ACTION_ACTIVE_IDLE
        self.__action_pending = ACTION_ACTIVE_IDLE
        self.__action_pending_at = 0.0
        self.__action_saved_locks: list[bool] | None = None
        self.__driver_status: ArmActionStatus | None = None

        # Latest absolute Cartesian target.  The master forwards it only
        # while an autonomous source owns the arm, so this node stays
        # source-agnostic.
        self.__pose: Pose | None = None
        self.__pose_received_at = 0.0
        self.__pose_active = False

        self.__joint_publishers()
        self.__joint_subscribers()
        self.__state_publishers()
        self.__input_subscribers()
        self.create_timer(DT, self.__update)

    # ------------------------------------------------------------------
    # ROS interfaces
    # ------------------------------------------------------------------

    def __joint_publishers(self):
        """Create the 5-DOF joint target publishers."""
        self.__target_publishers = {
            joint: self.create_publisher(
                Float32, JOINT_TARGET_TOPICS[joint], 10)
            for joint in ARM_JOINT_NAMES
        }

    def __joint_subscribers(self):
        """Subscribe to the measured 5-DOF joint angles."""
        for joint in ARM_JOINT_NAMES:
            self.create_subscription(
                Float32,
                JOINT_CURRENT_TOPICS[joint],
                self.__on_joint_angle_received(joint),
                10,
            )

    def __state_publishers(self):
        """Create the planner-owned arm state publishers."""
        self.__locked_publisher = self.create_publisher(
            UInt8, ARM_STATE_TOPICS["locked_joints"], 10)
        self.__homing_publisher = self.create_publisher(
            UInt8, ARM_STATE_TOPICS["homing_joints"], 10)
        self.__mode_publisher = self.create_publisher(
            UInt8, ARM_STATE_TOPICS["operation_mode"], 10)
        self.__flag_publishers = {
            name: self.create_publisher(
                Bool, ARM_STATE_TOPICS["flags"][name], 10)
            for name in PLANNER_FLAG_NAMES
        }
        self.__pose_publisher = self.create_publisher(
            Pose, ARM_STATE_TOPICS["pose"], 10)
        self.__status_publisher = self.create_publisher(
            ArmPlannerStatus, PLANNER_STATUS_TOPIC, 10)
        self.__action_command_publisher = self.create_publisher(
            ArmCommand, ACTION_COMMAND_TOPIC, 10)

    def __input_subscribers(self):
        """Create the E-stop, command, velocity, and gripper subscriptions."""
        self.create_subscription(
            Bool, ESTOP_TOPIC, self.__on_estop_received, 10)
        self.create_subscription(
            ArmCommand,
            PLANNER_COMMAND_TOPIC,
            self.__on_command_received,
            10,
        )
        self.create_subscription(
            Velocity, VELOCITY_TOPIC, self.__on_velocity_received, 10)
        self.create_subscription(
            Pose, POSE_TOPIC, self.__on_pose_received, 10)
        self.create_subscription(
            Bool, ARM_ENABLED_TOPIC, self.__on_arm_enable_received, 10)
        self.create_subscription(
            Int8, ACTION_ACTIVE_TOPIC, self.__on_action_active_received, 10)
        self.create_subscription(
            ArmActionStatus, ACTION_STATUS_TOPIC,
            self.__on_action_status_received, 10)

    # ------------------------------------------------------------------
    # Input callbacks
    # ------------------------------------------------------------------

    def __on_estop_received(self, msg: Bool):
        estop = bool(msg.data)
        if estop == self.__estop:
            return
        self.__estop = estop
        # Do not resume a held teleop trajectory after a stop or a clear.
        self.__arm.invalidate_teleop_reference()
        if self.__estop:
            self.get_logger().info("E-STOP latched: route planner paused")
            self.__emit_error(
                ARM_ERROR_ESTOP, force=True, log=False)
        else:
            self.get_logger().info("E-STOP cleared: route planner resumed")
            self.__emit_error(ARM_ERROR_NONE, log=False)

    def __on_velocity_received(self, msg: Velocity):
        """Latch the latest arbitrated teleop velocity."""
        self.__velocity = msg
        self.__velocity_received_at = time.monotonic()

    def __on_pose_received(self, msg: Pose):
        """Latch the latest arbitrated absolute pose target."""
        self.__pose = msg
        self.__pose_received_at = time.monotonic()

    def __on_arm_enable_received(self, msg: Bool):
        """Track the master's arm enable level (resting push precondition)."""
        self.__arm_enabled = bool(msg.data)

    def __on_action_active_received(self, msg: Int8):
        """Track which action the driver currently owns."""
        value = int(msg.data)
        if value not in (ACTION_ACTIVE_IDLE, ACTION_ACTIVE_HOMING,
                         ACTION_ACTIVE_FLIPPING, ACTION_ACTIVE_ZENITH,
                         ACTION_ACTIVE_RESTING):
            return
        self.__action_active = value
        self.__action_active_at = time.monotonic()
        if value != ACTION_ACTIVE_IDLE:
            # The driver has taken ownership; the boot-inference request is
            # no longer pending.
            self.__action_pending = ACTION_ACTIVE_IDLE

    def __on_action_status_received(self, msg: ArmActionStatus):
        """Latch the latest action-driver status (homing mask, diagnostics)."""
        self.__driver_status = msg

    def __on_command_received(self, msg: ArmCommand):
        """Handle one high-level operator command."""
        command = msg.command.strip()
        if not command:
            return
        self.__handle_command(command)

    def __on_joint_angle_received(self, joint: str):
        index = JOINT_INDEX[joint]

        def callback(msg: Float32):
            self.__measured[index] = wrap2pi(msg.data)
            self.__joint_feedback_at[index] = time.monotonic()
        return callback

    # ------------------------------------------------------------------
    # Command validation and errors
    # ------------------------------------------------------------------

    def __emit_error(self, code: int, arg: int = 0, reason: str = "",
                     force: bool = False, log: bool = True):
        """Record one error event for the master node to republish.

        Identical runtime codes are suppressed unless ``force`` is set;
        command rejections force a new event because each key press is a
        new user action.  ``code=ARM_ERROR_NONE`` clears a latched error.
        """
        code = int(code) & 0xFF
        arg = int(arg) & 0xFF

        if code == ARM_ERROR_NONE:
            if self.__error == (ARM_ERROR_NONE, 0):
                return
        elif self.__error == (code, arg) and not force:
            return

        self.__error = (code, arg)
        self.__error_seq = (self.__error_seq + 1) & 0xFF
        if log and code:
            self.get_logger().warning(
                f"Arm command rejected: "
                f"{reason or error_text(code, arg)}")

    def __handle_command(self, command: str):
        # Holding is always legal: the master emits it on a handover or
        # E-stop so a previous owner's target cannot keep moving the arm.
        if command in (PLANNER_HOLD_COMMAND, "stop"):
            self.__hold_position()
            return

        # Gripper and solenoid commands never reach this node; the master
        # routes them to the end effector.  Anything unrecognised here is a
        # motion command validation failure.
        if self.__estop:
            self.__emit_error(
                ARM_ERROR_ESTOP,
                reason=f"Arm command '{command}' ignored: e-stop active",
                force=True,
            )
            return

        if not self.__arm.initialized:
            self.__emit_error(
                ARM_ERROR_NOT_READY,
                reason=(f"Arm command '{command}' ignored: "
                        "arm not initialized yet"),
                force=True,
            )
            return

        handled, _ = self.__dispatch_command(command)
        failure = self.__arm.take_failure()
        if failure:
            code, arg = classify_failure(failure)
            self.__emit_error(code, arg, reason=failure, force=True)
        elif not handled:
            self.__emit_error(
                ARM_ERROR_UNKNOWN_COMMAND,
                reason=f"Unknown arm command '{command}'",
                force=True,
            )
        else:
            # A successful command clears errors latched by earlier ones.
            self.__emit_error(ARM_ERROR_NONE, log=False)

    def __dispatch_command(self, command: str):
        """Execute one semantic planner command; returns (handled, result)."""
        if command in ("sync_target", "sync"):
            self.__arm.sync_angles(
                list(self.__measured), reset_target=True)
            self.get_logger().info(
                "Arm target synchronized to the measured pose.")
            return True, True
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
        if command in ("typing_mode", "typing"):
            return True, self.__arm.set_typing_mode(True)
        if command in ("typing_off", "typing_disable"):
            return True, self.__arm.set_typing_mode(False)

        if ":" in command:
            action, _, argument = command.partition(":")
            try:
                index = int(argument)
            except ValueError:
                return False, False
            if action == "toggle_joint_lock" and 0 <= index < 5:
                return True, self.__arm.toggle_joint_lock(index)

        return False, False

    def __hold_position(self):
        """Drop any pose/velocity target and hold the measured pose."""
        self.__pose = None
        self.__pose_received_at = 0.0
        self.__pose_active = False
        self.__velocity = None
        self.__velocity_received_at = 0.0
        self.__arm.hold()
        self.get_logger().info("Arm target held")

    # ------------------------------------------------------------------
    # Action-driver coordination
    # ------------------------------------------------------------------

    def __action_value(self):
        """Current action enum, with a stale-driver fail-safe.

        A dead action driver must not leave target publishing suppressed,
        so an ACTIVE value that has not refreshed within the timeout is
        treated as IDLE.  The boot-inference request stays valid for a
        short grace period until the driver reports its own state.
        """
        now = time.monotonic()
        if now - self.__action_active_at > ACTION_ACTIVE_TIMEOUT_SEC:
            value = ACTION_ACTIVE_IDLE
        else:
            value = self.__action_active
        if value == ACTION_ACTIVE_IDLE \
                and self.__action_pending != ACTION_ACTIVE_IDLE:
            if now - self.__action_pending_at <= 2.0:
                return self.__action_pending
            self.__action_pending = ACTION_ACTIVE_IDLE
        return value

    def __mirror_action_state(self, active):
        """Reflect the driver's action in the planner's mode flags/locks.

        The planner owns modes and locks; the driver publishes what it is
        doing and this method applies the same flag/lock changes the old
        in-planner actions performed.
        """
        zenith_now = active == ACTION_ACTIVE_ZENITH
        resting_now = active == ACTION_ACTIVE_RESTING

        if self.__arm.zenith and not zenith_now:
            self.__arm.zenith = False
            self.__arm.limit_mode = PICK_AND_PLACE_MODE
            self.__arm.planar = False
            self.__arm.typing = False
            if self.__action_saved_locks is not None:
                self.__arm.locked_joints = list(self.__action_saved_locks)
                self.__action_saved_locks = None
            self.__arm.settle_side()
        if self.__arm.resting and not resting_now:
            self.__arm.resting = False
            self.__arm.limit_mode = PICK_AND_PLACE_MODE
            self.__arm.planar = False
            self.__arm.typing = False
            self.__arm.joint_drive = False
            if self.__action_saved_locks is not None:
                self.__arm.locked_joints = list(self.__action_saved_locks)
                self.__action_saved_locks = None
            self.__arm.settle_side()

        if zenith_now and not self.__arm.zenith:
            self.__action_saved_locks = list(self.__arm.locked_joints)
            self.__arm.zenith = True
            self.__arm.limit_mode = ZENITH_MODE
            self.__arm.planar = False
            self.__arm.flipped = False
            self.__arm.typing = False
            self.__arm.locked_joints[0] = True
            self.__arm.locked_joints[4] = True
        if resting_now and not self.__arm.resting:
            self.__action_saved_locks = list(self.__arm.locked_joints)
            self.__arm.resting = True
            self.__arm.limit_mode = RESTING_MODE
            self.__arm.planar = True
            self.__arm.locked_joints = [
                False] * len(self.__arm.locked_joints)

    def __request_inferred_action(self, mode):
        """Ask the action driver to adopt the startup pose."""
        if mode == "ZENITH":
            self.__action_pending = ACTION_ACTIVE_ZENITH
            command = "adopt:zenith"
        elif mode == "RESTING":
            self.__action_pending = ACTION_ACTIVE_RESTING
            command = "adopt:resting"
        else:
            return
        self.__action_pending_at = time.monotonic()
        self.__action_command_publisher.publish(
            ArmCommand(command=command))

    # ------------------------------------------------------------------
    # Control cycle
    # ------------------------------------------------------------------

    def __pose_is_fresh(self) -> bool:
        """True when an absolute target has refreshed recently."""
        if self.__pose is None:
            return False
        if (time.monotonic() - self.__pose_received_at
                > AUTONOMOUS_POSE_TIMEOUT_SEC):
            if self.__pose_active:
                self.__pose_active = False
                self.get_logger().warning(
                    "Absolute pose target stale; holding measured pose")
            self.__pose = None
            return False
        return True

    def __apply_pose_target(self):
        """Solve and latch the current absolute Cartesian target."""
        if self.__pose is None:
            return
        msg = self.__pose
        position = (
            float(msg.x) / METERS_PER_INCH,
            float(msg.y) / METERS_PER_INCH,
            float(msg.z) / METERS_PER_INCH,
        )
        rotation = rotation_from_rpy(
            float(msg.roll), float(msg.pitch), float(msg.yaw))
        if not self.__pose_active:
            self.__pose_active = True
            self.get_logger().debug("Absolute pose target acquired")
        self.__arm.command_pose(position, rotation, drive=False)

    def __next_deltas(self):
        """Return the current teleop delta, or None when idle.

        The master only publishes this topic while manual owns the arm, so a
        missing or stale message means no manual motion.
        """
        velocity = self.__velocity
        if velocity is None:
            return None
        if (time.monotonic() - self.__velocity_received_at
                > VELOCITY_TIMEOUT_SEC):
            return None

        linear = (velocity.linear_x, velocity.linear_y, velocity.linear_z)
        angular = (velocity.angular_x, velocity.angular_y,
                   velocity.angular_z)
        if not any(linear + angular):
            return None

        # Linear velocity is m/s; the target frame and controller use inches.
        return {
            "dx": velocity.linear_x / METERS_PER_INCH * DT,
            "dy": velocity.linear_y / METERS_PER_INCH * DT,
            "dz": velocity.linear_z / METERS_PER_INCH * DT,
            "drx": velocity.angular_x * DT,
            "dry": velocity.angular_y * DT,
            "drz": velocity.angular_z * DT,
        }

    def __velocity_is_fresh(self) -> bool:
        """Return True when a manual velocity sample is still current."""
        return (
            self.__velocity is not None
            and time.monotonic() - self.__velocity_received_at
            <= VELOCITY_TIMEOUT_SEC
        )

    def __teleop_velocity(self) -> tuple:
        """Return the teleop command as (in/s, in/s, in/s, rad/s, rad/s, rad/s).

        The controller works in inches, so convert the wire units at this
        boundary exactly once.
        """
        if not self.__velocity_is_fresh():
            return (0.0,) * 6
        velocity = self.__velocity
        return (
            float(velocity.linear_x) / METERS_PER_INCH,
            float(velocity.linear_y) / METERS_PER_INCH,
            float(velocity.linear_z) / METERS_PER_INCH,
            float(velocity.angular_x),
            float(velocity.angular_y),
            float(velocity.angular_z),
        )

    def __update(self):
        if self.__estop:
            # Keep the master's readiness view alive while paused.
            self.__publish_status()
            return

        if not self.__arm.initialized:
            self.__publish_status()
            # Wait until every arm joint has a measured angle.
            if any(math.isnan(a) for a in self.__measured):
                return
            # Adopt the measured pose and the mode that best fits it.  The
            # arm may start outside the boot-default normal limits (for
            # example straight up, flipped, or reaching into the top-back
            # octant), so infer the real mode before publishing status.
            self.__arm.sync_angles(
                list(self.__measured), reset_target=True)
            mode = self.__arm.infer_initial_mode()
            self.__regions.update(self.__measured)
            angle_text = " ".join(
                f"{math.degrees(a):.1f}" for a in self.__measured)
            self.get_logger().info(
                f"Route planner ready | mode {mode or 'PNP'} | "
                f"measured angles {angle_text} deg")
            self.__publish_status()
            # Publish the inferred mode/flags immediately instead of waiting
            # for the next control cycle, so the basestation display reflects
            # the starting pose as soon as the arm is ready.
            self.__publish_arm_status()
            self.__publish_pose()
            # Hand an inferred zenith/resting pose to the action driver; the
            # planner never starts those actions itself.
            self.__request_inferred_action(mode)

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
                    f"limits for joints {outside}; arm may need homing")
            return

        # Action-driver ownership.  While the driver reports an action the
        # planner neither steps the controller nor publishes targets; it
        # only mirrors the mode flags and keeps telemetry flowing.
        active = self.__action_value()
        self.__current_active = active
        self.__regions.update(self.__measured)
        self.__mirror_action_state(active)
        if active != ACTION_ACTIVE_IDLE:
            self.__action_prev = active
            self.__publish_arm_status()
            self.__publish_pose()
            self.__publish_status()
            return
        if self.__action_prev != ACTION_ACTIVE_IDLE:
            # Falling edge: adopt the measured pose before resuming so the
            # arm never jumps back to its pre-action targets.
            self.__arm.hold()
            self.__arm.settle_side()
            self.__last_targets = None
        self.__action_prev = ACTION_ACTIVE_IDLE

        pose_active = self.__pose_is_fresh()
        # Manual teleop owns the arm while a velocity sample is arriving or
        # while the teleop stage is still ramping down.  It gets its own
        # output stage so the command is a smooth internal reference rather
        # than "measured + one step".
        teleop_active = (
            not pose_active
            and not self.__arm.joint_drive
            and (self.__velocity_is_fresh()
                 or self.__arm.teleop_motion_active)
        )

        if teleop_active:
            moved = self.__arm.update_teleop(
                self.__teleop_velocity(), self.__measured)
            if moved:
                consumed = True
            else:
                # Stopped (or stalled): drop the target so the arm holds
                # wherever it finished instead of chasing a stale frame.
                self.__arm.hold()
                consumed = False
        else:
            self.__arm.sync_angles(
                list(self.__measured), reset_target=False)

            if pose_active:
                self.__apply_pose_target()
            deltas = None if pose_active else self.__next_deltas()
            # An idle non-teleop path holds the achieved pose instead of
            # chasing the last target frame.  Absolute pose targets still
            # finish normally.
            if not pose_active and deltas is None:
                self.__arm.hold()
            consumed = self.__arm.update(
                deltas,
                feedback_fresh=True,
                resting_enabled=self.__arm_enabled)
        self.__publish_counter += 1
        self.__publish_targets(force=self.__publish_counter % 20 == 0)
        self.__publish_arm_status()
        self.__publish_pose()
        self.__report_status(consumed)
        self.__publish_status()

    def __publish_targets(self, force=False):
        """Publish the rate-limited 5-DOF joint targets."""
        commanded = self.__arm.commanded_angles()
        if commanded is None:
            return
        angles = [float(a) for a in commanded]
        # Wrap every joint around zero.  The base spans -135..135 deg, so a
        # [-90, 270) wrap would turn a valid -100 deg target into +260 deg,
        # which arm_hardware_node rejects against the motor limits.
        targets = [wrap2pi(angle) for angle in angles]
        payload = tuple(targets)
        # Publishing unchanged targets every cycle floods arm_hardware_node's
        # log.  Republish periodically so a late consumer still gets them.
        if payload == self.__last_targets and not force:
            return
        self.__last_targets = payload
        for joint, value in zip(ARM_JOINT_NAMES, targets):
            self.__target_publishers[joint].publish(Float32(data=value))

    def __publish_arm_status(self):
        """Publish per-joint lock/homing and high-level arm state."""
        locked = 0
        homing = 0
        for index, locked_joint in enumerate(self.__arm.locked_joints):
            if locked_joint:
                locked |= 1 << index
        active = self.__current_active
        if active == ACTION_ACTIVE_HOMING and self.__driver_status is not None:
            homing = int(self.__driver_status.homing_mask)

        if active == ACTION_ACTIVE_ZENITH:
            mode = ARM_OPERATION_MODES["ZENITH"]
        elif active == ACTION_ACTIVE_RESTING:
            mode = ARM_OPERATION_MODES["RESTING"]
        elif active == ACTION_ACTIVE_HOMING:
            mode = ARM_OPERATION_MODES["HOMING"]
        elif self.__arm.planar:
            mode = ARM_OPERATION_MODES["PLANAR"]
        elif self.__arm.flipped:
            mode = ARM_OPERATION_MODES["PNP_FLIPPED"]
        else:
            mode = ARM_OPERATION_MODES["PNP"]

        flag_values = {
            "HOMING": active == ACTION_ACTIVE_HOMING,
            "FLIPPING": active == ACTION_ACTIVE_FLIPPING,
            # Flipped and zenith are observed regions of the measured pose;
            # planar and the locks stay planner-owned modes.
            "FLIPPED": self.__regions.flipped,
            "PLANAR": self.__arm.planar,
            "JOINT_DRIVE": self.__arm.joint_drive,
            "POSITION_LOCK": self.__arm.position_locked,
            "ORIENTATION_LOCK": self.__arm.orientation_locked,
            "ZENITH": self.__regions.zenith,
            "RESTING": active == ACTION_ACTIVE_RESTING,
        }

        self.__locked_publisher.publish(UInt8(data=locked))
        self.__homing_publisher.publish(UInt8(data=homing))
        self.__mode_publisher.publish(UInt8(data=mode))
        for name, value in flag_values.items():
            self.__flag_publishers[name].publish(Bool(data=bool(value)))

    def __publish_pose(self):
        """Publish the measured end-effector pose for the basestation.

        Teleop keeps an internal joint reference, so the controller's FK is
        the commanded pose, not the achieved one.  Publish the measured pose
        explicitly: the typing node's arrival checks and the operator display
        both depend on it.
        """
        if any(math.isnan(a) for a in self.__measured):
            return
        frame = self.__arm.model.solve_forward_kinematics(self.__measured)
        x, y, z = (float(value) for value in frame.position())
        roll, pitch, yaw = rotation_to_rpy(frame.rotation_matrix())
        self.__pose_publisher.publish(Pose(
            x=x * METERS_PER_INCH,
            y=y * METERS_PER_INCH,
            z=z * METERS_PER_INCH,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
        ))

    def __publish_status(self):
        """Publish readiness and the latest error event."""
        msg = ArmPlannerStatus()
        msg.ready = bool(self.__arm.initialized)
        msg.error_seq = int(self.__error_seq)
        msg.error_code = self.__error[0]
        msg.error_arg = self.__error[1]
        self.__status_publisher.publish(msg)

    def __report_status(self, consumed: bool):
        failure = self.__arm.take_failure()
        if failure:
            code, arg = classify_failure(failure)
            # Latch the runtime failure until a successful command clears
            # it, matching the old single-node error behaviour.
            self.__emit_error(code, arg, reason=failure)

        drive = "joint" if self.__arm.joint_drive else "position"
        active = self.__current_active
        if active == ACTION_ACTIVE_ZENITH:
            state = "zenith"
        elif active == ACTION_ACTIVE_RESTING:
            state = "resting"
        elif active == ACTION_ACTIVE_HOMING:
            state = "homing"
        elif active == ACTION_ACTIVE_FLIPPING:
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
        self.get_logger().info("Arm route planner started")
        rclpy.spin(self)
        self.get_logger().info("Arm route planner stopped")


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.ALL)
    planner: ArmRoutePlannerNode | None = None
    try:
        planner = ArmRoutePlannerNode()
        planner.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if planner is not None:
            try:
                planner.destroy_node()
            except Exception:
                pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
