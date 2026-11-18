"""Arm master node: command routing, readiness, and error ownership.

This is the arm subsystem master and the command mux.  It is the only node
that reads the control source and the only subscriber of the manual inputs.
Every command reaches an actuator through this node:

    manual teleop   /ARM/MANUAL/CMD_VELOCITY -> /ARM/CMD_VELOCITY
    manual keys     /ARM/COMMAND             -> /ARM/PLANNER/COMMAND
                                            -> /ARM/END_EFFECTOR/COMMAND

The functional nodes are source-agnostic and consume only the arbitrated
output:

    arm_route_planner_node : all 5-DOF motion. It validates the motion
                             commands and owns their readiness, e-stop, and
                             unknown-command errors
    arm_end_effector_node  : gripper jog and solenoid strikes. It receives
                             only end-effector commands; the master validates
                             their preconditions and throttles errors for
                             commands a held arrow repeats

The master keeps the cross-cutting concerns that need a single owner: the
planner health watchdog and ``/ARM/STATE/ERROR``, the one topic the
basestation error banner reads.  The planner reports error events in
``ArmPlannerStatus``; the master republishes them, so the packed sequence
counter has exactly one writer.

The control source is published for diagnostics, not as a gate the actuators
read.  A handover publishes a zero velocity, an explicit gripper-jog stop, and
an internal planner ``hold``, so stale manual or autonomous intent cannot leak
into the new owner's window.

Inputs:
    /ESTOP                    std_msgs/Bool
    /ARM/COMMAND              std_msgs/String
    /ARM/MANUAL/CMD_VELOCITY  custom_interfaces/Velocity
    /ARM/AUTONOMOUS/CMD_POSE  custom_interfaces/Pose
    /ARM/AUTONOMOUS/COMMAND   custom_interfaces/ArmCommand
    /ARM/CONTROL/REQUEST      std_msgs/UInt8   (control ownership requests)
    /ARM/PLANNER/STATUS       custom_interfaces/ArmPlannerStatus

Outputs:
    /ARM/CMD_VELOCITY          custom_interfaces/Velocity
    /ARM/CMD_POSE              custom_interfaces/Pose
    /ARM/PLANNER/COMMAND       custom_interfaces/ArmCommand
    /ARM/END_EFFECTOR/COMMAND  custom_interfaces/ArmCommand
    /ARM/STATE/CONTROL_SOURCE  std_msgs/UInt8   (manual/autonomous/none)
    /ARM/STATE/ERROR           std_msgs/UInt32
"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from custom_interfaces.msg import (
    ArmCommand,
    ArmPlannerStatus,
    Pose,
    Velocity,
)
from std_msgs.msg import Bool, String, UInt8, UInt32

from arm_constants import (
    ARM_COMMAND_TOPIC,
    ARM_CONTROL_REQUEST_TOPIC,
    ARM_CONTROL_SOURCE_TOPIC,
    ARM_STATE_TOPICS,
    AUTONOMOUS_COMMAND_TOPIC,
    AUTONOMOUS_POSE_TOPIC,
    AUTONOMOUS_POSE_TIMEOUT_SEC,
    DEFAULT_ARM_CONTROL_SOURCE,
    DT,
    END_EFFECTOR_COMMANDS,
    END_EFFECTOR_COMMAND_TOPIC,
    END_EFFECTOR_STOP_COMMAND,
    ESTOP_TOPIC,
    GRIPPER_JOG_COMMANDS,
    JOG_REJECT_INTERVAL_SEC,
    MANUAL_VELOCITY_TOPIC,
    PLANNER_COMMAND_TOPIC,
    PLANNER_HOLD_COMMAND,
    PLANNER_STATUS_TOPIC,
    PLANNER_TIMEOUT_SEC,
    POSE_TOPIC,
    PRE_READY_END_EFFECTOR_COMMANDS,
    VELOCITY_NEUTRAL_EPSILON,
    VELOCITY_TOPIC,
    ArmControlSource,
    control_source_label,
    parse_control_source,
)
from rover_protocol.arm_errors import (
    ARM_ERROR_ESTOP,
    ARM_ERROR_NONE,
    ARM_ERROR_NOT_READY,
    ARM_ERROR_REJECTED,
)

# Ownership is state, not a stream.  Transient-local lets a late joiner catch
# the current value; the periodic republish stays as a fallback for bridges
# that do not implement durability.
CONTROL_SOURCE_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# ----------------------------------------------------------------------
# Pure mux policy
#
# Both pieces live here so the master node is the single place that defines
# ownership and manual-input admission.  They have no ROS dependency and are
# exercised directly by the test suite.
# ----------------------------------------------------------------------


def select_control_source(
    current,
    request,
    planner_alive,
    planner_ready,
    estop,
):
    """Decide whether one control request may take effect.

    Policy: ``MANUAL`` always wins so the operator can always reclaim the arm;
    ``NONE`` is always accepted and means no source may move the arm;
    ``AUTONOMOUS`` requires a live, ready planner and no active E-stop.
    Unknown wire values are refused.

    Args:
        current: The currently active ``ArmControlSource``.
        request: Requested wire value (int or ``ArmControlSource``).
        planner_alive: True when planner status is arriving.
        planner_ready: True when the planner has initialized the arm.
        estop: True while the E-stop is latched.

    Returns:
        tuple[ArmControlSource, bool, str]: ``(source, accepted, reason)``.
        ``source`` is the new source on acceptance and ``current`` on
        refusal.
    """
    requested = parse_control_source(request)
    if requested is None:
        return current, False, f"unknown control source {request!r}"
    if requested == current:
        return current, True, ""
    if requested == ArmControlSource.MANUAL:
        return requested, True, "manual override"
    if requested == ArmControlSource.NONE:
        return requested, True, "control released"

    # AUTONOMOUS
    if estop:
        return current, False, "e-stop active"
    if not planner_alive:
        return current, False, "route planner unavailable"
    if not planner_ready:
        return current, False, "arm not ready"
    return requested, True, "autonomous control granted"


def velocity_is_neutral(values, epsilon=VELOCITY_NEUTRAL_EPSILON) -> bool:
    """True when every velocity component is within ``epsilon`` of zero."""
    return all(abs(float(value)) <= epsilon for value in values)


class ManualVelocityGate:
    """Require neutral input before re-arming manual velocity.

    A handover or an E-stop disarms the gate, so a stick that was deflected
    during another owner's window cannot move the arm the instant manual
    returns.
    """

    def __init__(self, epsilon=VELOCITY_NEUTRAL_EPSILON):
        self.epsilon = max(0.0, float(epsilon))
        self.__armed = False

    @property
    def armed(self) -> bool:
        """True once a neutral sample has re-armed the gate."""
        return self.__armed

    def reset(self) -> None:
        """Disarm; the next admitted sample must be neutral."""
        self.__armed = False

    def admit(self, values) -> bool:
        """Return True when this sample may be forwarded.

        While disarmed only a neutral sample arms the gate, and that neutral
        sample itself passes so the planner receives a clean zero.  Once
        armed, every sample passes.
        """
        if not self.__armed:
            if not velocity_is_neutral(values, self.epsilon):
                return False
            self.__armed = True
        return True


class ArmMasterNode(Node):
    """Mux arm commands, arbitrate control ownership, and own errors."""

    def __init__(self):
        super().__init__("arm_master_node")

        self.__estop = False

        # Control ownership: the master is the single reader of the state
        # topic and the only node that gates on it.  Manual ownership is the
        # safe default so the existing operator workflow is unchanged;
        # autonomy must request the arm.
        self.__control_source = DEFAULT_ARM_CONTROL_SOURCE

        # Teleop is only forwarded once the stick has been neutral since the
        # last handover, so a deflected stick cannot move the arm the moment
        # manual ownership returns.
        self.__velocity_gate = ManualVelocityGate()

        # Planner readiness and the latest error event it reported.
        self.__planner_status = None
        self.__planner_status_received_at = 0.0
        self.__planner_ready = False
        self.__last_error_seq = None

        # Autonomous pose stream watchdog.  The producer refreshes the target
        # while it is active; silence means it died or deliberately paused, so
        # the master holds the measured pose instead of chasing stale intent.
        self.__autonomous_pose_seen = False
        self.__autonomous_pose_lost = False
        self.__autonomous_pose_received_at = 0.0

        # Last error republished on /ARM/STATE/ERROR.
        self.__error_seq = 0
        self.__last_error = None

        # Rejection throttle for held gripper jog commands.
        self.__jog_report_command = None
        self.__jog_report_at = 0.0

        self.__node_links()
        self.__input_subscribers()
        self.create_timer(DT, self.__update)

    # ------------------------------------------------------------------
    # ROS interfaces
    # ------------------------------------------------------------------

    def __node_links(self):
        """Create the command publishers, error publisher, and status sub."""
        self.__velocity_publisher = self.create_publisher(
            Velocity, VELOCITY_TOPIC, 10)
        self.__pose_publisher = self.create_publisher(
            Pose, POSE_TOPIC, 10)
        self.__planner_command_publisher = self.create_publisher(
            ArmCommand, PLANNER_COMMAND_TOPIC, 10)
        self.__end_effector_command_publisher = self.create_publisher(
            ArmCommand, END_EFFECTOR_COMMAND_TOPIC, 10)
        # One topic keeps code/arg/seq consistent for the telemetry uplink.
        self.__error_publisher = self.create_publisher(
            UInt32, ARM_STATE_TOPICS["error"], 10)
        # Ownership is state: keep it transient-local so a late joiner sees
        # the current source, and republish every cycle as a bridge fallback.
        self.__control_source_publisher = self.create_publisher(
            UInt8, ARM_CONTROL_SOURCE_TOPIC, CONTROL_SOURCE_QOS)
        self.__publish_control_source()
        self.create_subscription(
            ArmPlannerStatus,
            PLANNER_STATUS_TOPIC,
            self.__on_planner_status_received,
            10,
        )

    def __input_subscribers(self):
        """Create the E-stop, command, teleop, and ownership subscriptions."""
        self.create_subscription(
            Bool, ESTOP_TOPIC, self.__on_estop_received, 10)
        self.create_subscription(
            String, ARM_COMMAND_TOPIC, self.__on_command_received, 10)
        self.create_subscription(
            Velocity,
            MANUAL_VELOCITY_TOPIC,
            self.__on_manual_velocity_received,
            10,
        )
        self.create_subscription(
            Pose,
            AUTONOMOUS_POSE_TOPIC,
            self.__on_autonomous_pose_received,
            10,
        )
        self.create_subscription(
            ArmCommand,
            AUTONOMOUS_COMMAND_TOPIC,
            self.__on_autonomous_command_received,
            10,
        )
        self.create_subscription(
            UInt8,
            ARM_CONTROL_REQUEST_TOPIC,
            self.__on_control_request_received,
            10,
        )

    # ------------------------------------------------------------------
    # Input callbacks
    # ------------------------------------------------------------------

    def __on_estop_received(self, msg: Bool):
        estop = bool(msg.data)
        if estop == self.__estop:
            return
        self.__estop = estop
        # An E-stop invalidates manual intent in both directions: never resume
        # a held stick after the latch clears.
        self.__velocity_gate.reset()
        self.__publish_zero_velocity()
        self.__publish_hold()
        if self.__estop:
            self.__publish_stop_jog()
            self.get_logger().info("E-STOP latched: arm master paused")
        else:
            self.get_logger().info("E-STOP cleared: arm master resumed")

    def __on_command_received(self, msg: String):
        command = msg.data.strip()
        if command:
            self.__handle_command(command)

    def __on_manual_velocity_received(self, msg: Velocity):
        """Forward manual teleop velocity, or drop it if manual is not active.

        The planner consumes only the arbitrated topic, so this node is the
        one place that decides whether teleop may move the arm.  After a
        handover the gate stays closed until the stick returns to neutral.
        """
        if self.__estop:
            return
        if self.__control_source != ArmControlSource.MANUAL:
            return

        values = (
            msg.linear_x, msg.linear_y, msg.linear_z,
            msg.angular_x, msg.angular_y, msg.angular_z,
        )
        if not self.__velocity_gate.admit(values):
            return
        self.__velocity_publisher.publish(msg)

    def __on_autonomous_pose_received(self, msg: Pose):
        """Forward an autonomous target while autonomy owns the arm."""
        if self.__estop:
            return
        if self.__control_source != ArmControlSource.AUTONOMOUS:
            return
        self.__pose_publisher.publish(msg)
        self.__autonomous_pose_received_at = time.monotonic()
        self.__autonomous_pose_seen = True
        if self.__autonomous_pose_lost:
            self.__autonomous_pose_lost = False
            self.get_logger().info("Autonomous pose stream resumed")

    def __on_autonomous_command_received(self, msg: ArmCommand):
        """Route an autonomous command through the same ownership gate."""
        command = msg.command.strip()
        if not command or self.__estop:
            return
        if self.__control_source != ArmControlSource.AUTONOMOUS:
            return
        if command in END_EFFECTOR_COMMANDS:
            self.__forward(self.__end_effector_command_publisher, command)
        else:
            self.__forward(self.__planner_command_publisher, command)

    def __on_control_request_received(self, msg: UInt8):
        """Apply one control-ownership request or explain the refusal."""
        requested = int(msg.data)
        source, accepted, reason = select_control_source(
            self.__control_source,
            requested,
            self.__planner_is_alive(),
            self.__planner_is_ready(),
            self.__estop,
        )
        if not accepted:
            self.get_logger().warning(
                f"Control request for {control_source_label(requested)} "
                f"refused: {reason}")
            self.__publish_error(ARM_ERROR_REJECTED)
            return
        if source == self.__control_source:
            return
        self.__control_source = source
        self.__publish_control_source()
        # A handover invalidates the previous owner's in-flight intent.  The
        # zero velocity, planner hold, and explicit jog stop land after any
        # earlier forwarded sample on the same publisher, so ordering is
        # defined.
        self.__velocity_gate.reset()
        self.__publish_zero_velocity()
        self.__publish_hold()
        self.__publish_stop_jog()
        if source == ArmControlSource.AUTONOMOUS:
            self.__autonomous_pose_seen = False
            self.__autonomous_pose_lost = False
            self.__autonomous_pose_received_at = 0.0
        self.get_logger().info(
            f"Control source: {control_source_label(source)} ({reason})")

    def __on_planner_status_received(self, msg: ArmPlannerStatus):
        """Relay a new planner error event and refresh readiness."""
        self.__planner_status = msg
        self.__planner_status_received_at = time.monotonic()

        seq = int(msg.error_seq) & 0xFF
        if seq == self.__last_error_seq:
            return
        self.__last_error_seq = seq
        self.__publish_error(msg.error_code, msg.error_arg)

    # ------------------------------------------------------------------
    # Command routing
    # ------------------------------------------------------------------

    def __handle_command(self, command: str):
        """Route one command to the node that owns it.

        ``/ARM/COMMAND`` is the manual command surface.  Autonomy gets its
        own input topic when the setpoint interface lands; until then this
        mux keeps manual key presses from moving an arm that an autonomous
        controller has claimed.

        Motion commands go to the planner, which validates them and owns
        their readiness/e-stop/unknown-command errors.  Gripper and solenoid
        commands belong to the end effector and never reach the planner, so
        the master validates their preconditions here.
        """
        if self.__control_source != ArmControlSource.MANUAL:
            if self.__should_report(command):
                self.get_logger().warning(
                    f"Manual command '{command}' ignored: "
                    f"{control_source_label(self.__control_source)} "
                    f"owns the arm")
                self.__publish_error(ARM_ERROR_REJECTED)
            return

        if command in END_EFFECTOR_COMMANDS:
            self.__handle_end_effector_command(command)
            return

        self.__forward(self.__planner_command_publisher, command)

        if self.__estop:
            return

        if not self.__planner_is_alive():
            # No planner to report this, so surface it locally.
            self.__publish_error(ARM_ERROR_REJECTED)

    def __handle_end_effector_command(self, command: str):
        """Validate and route one gripper or solenoid command."""
        if self.__estop:
            self.__reject_end_effector(
                command,
                ARM_ERROR_ESTOP,
                f"End effector command '{command}' ignored: e-stop active",
            )
            return

        if command in PRE_READY_END_EFFECTOR_COMMANDS:
            # One-shot hardware pulse; legal before the arm initializes.
            self.__forward(self.__end_effector_command_publisher, command)
            return

        if not self.__planner_is_alive():
            self.__reject_end_effector(
                command,
                ARM_ERROR_REJECTED,
                f"End effector command '{command}' ignored: "
                "route planner unavailable",
            )
            return

        # The gripper jog needs the 5-DOF arm to be initialized, but the
        # planner is not involved in the command itself.
        if not self.__planner_is_ready():
            self.__reject_end_effector(
                command,
                ARM_ERROR_NOT_READY,
                f"End effector command '{command}' ignored: arm not ready",
            )
            return

        self.__clear_jog_report(command)
        self.__forward(self.__end_effector_command_publisher, command)

    def __reject_end_effector(self, command, code, message):
        """Report a rejected end-effector command, throttling held arrows."""
        if not self.__should_report(command):
            return
        self.get_logger().warning(message)
        self.__publish_error(code)

    def __should_report(self, command: str) -> bool:
        """Rate-limit rejections of a held gripper jog command."""
        if command not in GRIPPER_JOG_COMMANDS:
            return True
        now = time.monotonic()
        if (command == self.__jog_report_command
                and now - self.__jog_report_at < JOG_REJECT_INTERVAL_SEC):
            return False
        self.__jog_report_command = command
        self.__jog_report_at = now
        return True

    def __clear_jog_report(self, command: str):
        """Allow the next rejection of this direction to report immediately."""
        if command == self.__jog_report_command:
            self.__jog_report_command = None

    @staticmethod
    def __forward(publisher, command: str):
        publisher.publish(ArmCommand(command=command))

    # ------------------------------------------------------------------
    # Planner health and errors
    # ------------------------------------------------------------------

    def __planner_is_alive(self):
        """True when planner status is arriving at all."""
        if self.__planner_status is None:
            return False
        return (time.monotonic() - self.__planner_status_received_at
                <= PLANNER_TIMEOUT_SEC)

    def __planner_is_ready(self):
        """True when a fresh planner status says the 5-DOF arm is ready."""
        return self.__planner_is_alive() and bool(self.__planner_status.ready)

    def __check_planner_health(self):
        """Log readiness transitions and surface a lost planner."""
        ready = self.__planner_is_ready()
        if ready == self.__planner_ready:
            return
        self.__planner_ready = ready
        if ready:
            self.get_logger().info(
                "Route planner connected; arm commands enabled")
            # Accept the planner's current error state after a restart.
            self.__last_error_seq = None
            self.__publish_error(ARM_ERROR_NONE)
        else:
            self.get_logger().warning(
                "Route planner unavailable; arm commands rejected")
            if not self.__planner_is_alive():
                self.__publish_error(ARM_ERROR_REJECTED)

    def __publish_error(self, code: int, arg: int = 0):
        """Publish the latest error event for the basestation banner."""
        code = int(code) & 0xFF
        arg = int(arg) & 0xFF
        if code == ARM_ERROR_NONE and self.__last_error is None:
            return
        self.__last_error = (code, arg)
        self.__error_seq = (self.__error_seq + 1) & 0xFF
        self.__error_publisher.publish(
            UInt32(data=(self.__error_seq << 16) | (code << 8) | arg))

    def __publish_control_source(self):
        """Publish the authoritative control source."""
        self.__control_source_publisher.publish(
            UInt8(data=int(self.__control_source)))

    def __publish_zero_velocity(self):
        """Send a clean zero so the planner drops any latched manual motion."""
        self.__velocity_publisher.publish(Velocity())

    def __publish_hold(self):
        """Tell the planner to hold its measured pose and drop any target."""
        self.__planner_command_publisher.publish(
            ArmCommand(command=PLANNER_HOLD_COMMAND))

    def __publish_stop_jog(self):
        """Release a held jog so the actuator need not read ownership."""
        self.__end_effector_command_publisher.publish(
            ArmCommand(command=END_EFFECTOR_STOP_COMMAND))

    # ------------------------------------------------------------------
    # Control cycle
    # ------------------------------------------------------------------

    def __update(self):
        # Republish even while E-stopped so late joiners still see who owns
        # the arm; the E-stop itself gates motion independently.
        self.__publish_control_source()
        if self.__estop:
            return
        self.__check_planner_health()
        self.__check_autonomous_pose()

    def __check_autonomous_pose(self):
        """Hold the arm if an autonomous target stream disappears."""
        if self.__control_source != ArmControlSource.AUTONOMOUS:
            return
        if not self.__autonomous_pose_seen or self.__autonomous_pose_lost:
            return
        if (time.monotonic() - self.__autonomous_pose_received_at
                <= AUTONOMOUS_POSE_TIMEOUT_SEC):
            return
        self.__autonomous_pose_lost = True
        self.get_logger().warning(
            "Autonomous pose stream stale; holding the measured pose")
        self.__publish_hold()

    def run(self):
        self.get_logger().info("Arm master started")
        rclpy.spin(self)
        self.get_logger().info("Arm master stopped")


def main():
    rclpy.init()
    master = None
    try:
        master = ArmMasterNode()
        master.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if master is not None:
            master.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
