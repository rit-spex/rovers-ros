"""Autonomous keyboard typing with the arm and drivebase.

The node accepts a keyboard plane and a literal key string, then types the
string by moving the arm to each key in a reference layout and firing the
solenoid.  It is an autonomous arm source, so it follows the same ownership
rules as every other command producer:

* request ``AUTONOMOUS`` on ``/ARM/CONTROL/REQUEST``;
* publish targets on ``/ARM/AUTONOMOUS/CMD_POSE`` and commands on
  ``/ARM/AUTONOMOUS/COMMAND``;
* let ``arm_master_node`` arbitrate those onto the planner and end effector.

The keyboard is expected to sit on the rover's left or right, roughly
parallel to its forward axis.  That lets the arm hold its base near +/-90 deg
(the wrist has little useful freedom) while the drivebase creeps along the
keyboard edge.  For each key the node first drives the key onto the arm's
lateral reach line, then sends the Cartesian pose and waits for the measured
end-effector pose to arrive before firing the solenoid.

The drive command is capped at ``TYPING_DRIVE_MAX_POWER`` (10%).  It is
published on the normal Xbox joystick axes, as if the basestation were
holding the sticks, so ``chassis_node`` needs no autonomous-drive path.
The node never drives blind: it needs fresh plane and control-source input,
and it stops the wheels on every state transition, E-stop, completion, or
failure.

Inputs:
    /ARM/TYPING/PLANE          custom_interfaces/ArmTypingPlane
    /ARM/TYPING/KEYS           std_msgs/String
    /ARM/STATE/CONTROL_SOURCE  std_msgs/UInt8
    /ARM/PLANNER/STATUS        custom_interfaces/ArmPlannerStatus
    /ARM/STATE/POSE            custom_interfaces/Pose
    /ESTOP                     std_msgs/Bool

Outputs:
    /ARM/CONTROL/REQUEST           std_msgs/UInt8
    /ARM/AUTONOMOUS/CMD_POSE       custom_interfaces/Pose
    /ARM/AUTONOMOUS/COMMAND        custom_interfaces/ArmCommand
    /BASESTATION/xbox/AXIS_LY      std_msgs/Float32
    /BASESTATION/xbox/AXIS_RY      std_msgs/Float32
    /ARM/TYPING/STATUS             custom_interfaces/ArmTypingStatus
"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from custom_interfaces.msg import (
    ArmCommand,
    ArmPlannerStatus,
    Pose,
    ArmTypingPlane,
    ArmTypingStatus,
)
from constants.CommandCodes import CONSTANTS
from std_msgs.msg import Bool, Float32, String, UInt8

from arm_constants import (
    ARM_CONTROL_REQUEST_TOPIC,
    ARM_CONTROL_SOURCE_TOPIC,
    ARM_STATE_TOPICS,
    AUTONOMOUS_COMMAND_TOPIC,
    AUTONOMOUS_POSE_TOPIC,
    ESTOP_TOPIC,
    PLANNER_STATUS_TOPIC,
    TYPING_ALIGN_TOL_M,
    TYPING_CONTROL_RETRY_SEC,
    TYPING_KEYS_TOPIC,
    TYPING_MODE_SETTLE_SEC,
    TYPING_PLANE_TIMEOUT_SEC,
    TYPING_PLANE_TOPIC,
    TYPING_PRESS_SETTLE_SEC,
    TYPING_RATE_HZ,
    TYPING_STATUS_PERIOD_SEC,
    TYPING_STATUS_TOPIC,
    ArmControlSource,
)
from arm_typing.typing_plan import (
    alignment_error,
    drive_power_for_error,
    key_target,
    parse_key_sequence,
    plane_frame,
    plane_is_sideways,
    pose_matches,
)

CONTROL_SOURCE_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
# Match chassis_node's joystick subscriptions.  A reliable teleop publisher
# would also connect, but using the same profile keeps the link symmetrical.
TELEOP_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

IDLE = "IDLE"
WAIT_CONTROL = "WAIT_CONTROL"
WAIT_PLANE = "WAIT_PLANE"
ALIGNING = "ALIGNING"
MOVING = "MOVING"
PRESSING = "PRESSING"
DONE = "DONE"
ERROR = "ERROR"


class ArmTypingNode(Node):
    """Type a key string using the arm, solenoid, and drivebase."""

    def __init__(self):
        """Create the typing interfaces and control timer."""
        super().__init__("arm_typing_node")

        self.__estop = False
        self.__control_source = ArmControlSource.NONE
        self.__owns_control = False
        self.__planner_ready = False

        self.__plane = None
        self.__plane_received_at = 0.0
        self.__plane_side_warned = False
        self.__current_pose = None

        self.__keys = []
        self.__index = 0
        self.__state = IDLE
        self.__detail = "idle"
        self.__resume_state = ALIGNING
        self.__typing_mode_sent = False
        self.__typing_mode_at = 0.0
        self.__fired = False
        self.__press_started_at = 0.0

        self.__last_control_request = 0.0
        self.__last_status_at = 0.0
        self.__last_status_key = None
        self.__last_logged_key = None
        self.__last_drive = (0.0, 0.0)

        self.__set_publishers()
        self.__set_subscribers()
        self.create_timer(1.0 / max(1.0, TYPING_RATE_HZ), self.__update)

    # ------------------------------------------------------------------
    # ROS interfaces
    # ------------------------------------------------------------------

    def __set_publishers(self):
        """Create the ownership, target, drive, and status publishers."""
        self.__control_request_publisher = self.create_publisher(
            UInt8, ARM_CONTROL_REQUEST_TOPIC, 10)
        self.__pose_publisher = self.create_publisher(
            Pose, AUTONOMOUS_POSE_TOPIC, 10)
        self.__command_publisher = self.create_publisher(
            ArmCommand, AUTONOMOUS_COMMAND_TOPIC, 10)
        # Publish the exact topics chassis_node already subscribes to for the
        # Xbox sticks; no new interface is introduced for typing drive.
        self.__drive_publishers = (
            self.create_publisher(
                Float32,
                "/BASESTATION/" + CONSTANTS.XBOX.NAME + "/"
                + CONSTANTS.XBOX.JOYSTICK.AXIS_LY_STR,
                TELEOP_QOS),
            self.create_publisher(
                Float32,
                "/BASESTATION/" + CONSTANTS.XBOX.NAME + "/"
                + CONSTANTS.XBOX.JOYSTICK.AXIS_RY_STR,
                TELEOP_QOS),
        )
        self.__status_publisher = self.create_publisher(
            ArmTypingStatus, TYPING_STATUS_TOPIC, 10)

    def __set_subscribers(self):
        """Create the plane, key, ownership, planner, pose, and E-stop subs."""
        self.create_subscription(
            ArmTypingPlane,
            TYPING_PLANE_TOPIC,
            self.__on_plane_received,
            10,
        )
        self.create_subscription(
            String, TYPING_KEYS_TOPIC, self.__on_keys_received, 10)
        self.create_subscription(
            UInt8,
            ARM_CONTROL_SOURCE_TOPIC,
            self.__on_control_source_received,
            CONTROL_SOURCE_QOS,
        )
        self.create_subscription(
            ArmPlannerStatus,
            PLANNER_STATUS_TOPIC,
            self.__on_planner_status_received,
            10,
        )
        self.create_subscription(
            Pose,
            ARM_STATE_TOPICS["pose"],
            self.__on_pose_received,
            10,
        )
        self.create_subscription(
            Bool, ESTOP_TOPIC, self.__on_estop_received, 10)

    # ------------------------------------------------------------------
    # Input callbacks
    # ------------------------------------------------------------------

    def __on_plane_received(self, msg: ArmTypingPlane):
        """Latch the keyboard plane in the rover base frame."""
        self.__plane = plane_frame(
            msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, msg.side)
        self.__plane_received_at = time.monotonic()
        if (not plane_is_sideways(self.__plane)
                and not self.__plane_side_warned):
            self.__plane_side_warned = True
            self.get_logger().warning(
                "Keyboard plane is nearly ahead of the rover; the arm base "
                "will not stay near +/-90 deg")

    def __on_keys_received(self, msg: String):
        """Start a new typing sequence from the literal key string."""
        text = msg.data
        if not text.strip():
            self.__abort("empty key string")
            return
        try:
            keys = parse_key_sequence(text)
        except ValueError as error:
            self.__abort(str(error))
            self.get_logger().error(f"Typing rejected: {error}")
            return
        if not keys:
            self.__abort("no keys to type")
            return

        self.__keys = keys
        self.__index = 0
        self.__fired = False
        self.__press_started_at = 0.0
        self.__state = WAIT_CONTROL
        self.__resume_state = ALIGNING
        self.__typing_mode_sent = False
        self.__typing_mode_at = 0.0
        self.__detail = "waiting for autonomous control"
        self.__last_status_key = None
        self.__stop_drive()
        self.__request_control()
        self.get_logger().info(
            f"Typing requested: {len(keys)} keys")
        self.__publish_status(force=True)

    def __on_control_source_received(self, msg: UInt8):
        """Track ownership; an autonomous revoke pauses the sequence."""
        try:
            source = ArmControlSource(int(msg.data))
        except ValueError:
            return
        previous = self.__control_source
        self.__control_source = source
        if source == ArmControlSource.AUTONOMOUS:
            self.__owns_control = True
            return

        if previous == ArmControlSource.AUTONOMOUS:
            self.__owns_control = False
            self.__stop_drive()
            if self.__state in (ALIGNING, MOVING, PRESSING, WAIT_CONTROL):
                self.__state = WAIT_CONTROL
                self.__resume_state = ALIGNING
                self.__typing_mode_sent = False
                self.__detail = "control released; waiting to reclaim"
                self.__publish_status(force=True)

    def __on_planner_status_received(self, msg: ArmPlannerStatus):
        """Track planner readiness before sending any target."""
        self.__planner_ready = bool(msg.ready)
        if not self.__planner_ready:
            self.__typing_mode_sent = False

    def __on_pose_received(self, msg: Pose):
        """Latch the measured end-effector pose used for arrival checks."""
        self.__current_pose = msg

    def __on_estop_received(self, msg: Bool):
        """Abort the sequence while the E-stop is asserted."""
        estop = bool(msg.data)
        if estop == self.__estop:
            return
        self.__estop = estop
        if self.__estop:
            self.__abort("e-stop latched")
            self.get_logger().warning("Typing stopped by E-stop")
        else:
            self.get_logger().info("Typing E-stop cleared; send new keys")

    # ------------------------------------------------------------------
    # Output helpers
    # ------------------------------------------------------------------

    def __request_control(self):
        """Ask for autonomous ownership, rate-limited while waiting."""
        now = time.monotonic()
        if now - self.__last_control_request < TYPING_CONTROL_RETRY_SEC:
            return
        self.__last_control_request = now
        self.__control_request_publisher.publish(
            UInt8(data=int(ArmControlSource.AUTONOMOUS)))

    def __release_control(self):
        """Hand the arm back to the operator once the sequence ends."""
        if self.__control_source == ArmControlSource.AUTONOMOUS:
            self.__control_request_publisher.publish(
                UInt8(data=int(ArmControlSource.MANUAL)))
        self.__owns_control = False

    def __publish_pose(self, target):
        """Publish the current Cartesian key target."""
        self.__pose_publisher.publish(Pose(
            x=target.x,
            y=target.y,
            z=target.z,
            roll=target.roll,
            pitch=target.pitch,
            yaw=target.yaw,
        ))

    def __hold_arm(self):
        """Republish the measured pose so the arm holds during alignment."""
        if self.__current_pose is not None:
            self.__publish_pose(self.__current_pose)

    def __publish_command(self, command: str):
        """Publish one autonomous end-effector command."""
        self.__command_publisher.publish(ArmCommand(command=command))

    def __publish_drive(self, left: float, right: float):
        """Publish wheel power on the basestation joystick axes.

        ``chassis_node`` treats these values exactly like a physical stick,
        including its own direction inversion, so an explicit sign flip here
        would double-invert.  A nonzero command is a stream: the chassis
        stops the wheels when it goes quiet, so keep refreshing it while the
        error remains.  A zero command only needs to be sent once.
        """
        command = (float(left), float(right))
        if command == self.__last_drive and not any(command):
            return
        self.__last_drive = command
        for publisher, value in zip(self.__drive_publishers, command):
            publisher.publish(Float32(data=value))

    def __stop_drive(self):
        """Command zero drive power."""
        self.__publish_drive(0.0, 0.0)

    def __publish_status(self, force=False):
        """Publish progress, throttled unless the state changed."""
        now = time.monotonic()
        key = (self.__keys[self.__index]
               if self.__index < len(self.__keys) else "")
        status_key = (self.__state, key, self.__index, len(self.__keys),
                      self.__detail)
        if (not force and status_key == self.__last_status_key
                and now - self.__last_status_at < TYPING_STATUS_PERIOD_SEC):
            return
        self.__last_status_key = status_key
        self.__last_status_at = now
        self.__status_publisher.publish(ArmTypingStatus(
            state=self.__state,
            key=key,
            index=min(self.__index + 1, len(self.__keys)),
            total=len(self.__keys),
            detail=self.__detail,
        ))
        if self.__state != self.__last_logged_key:
            self.__last_logged_key = self.__state
            self.get_logger().info(
                f"Typing state: {self.__state} | {self.__detail}")

    # ------------------------------------------------------------------
    # State helpers
    # ------------------------------------------------------------------

    def __abort(self, detail: str):
        """Stop driving, drop the sequence, and release ownership."""
        self.__stop_drive()
        self.__keys = []
        self.__index = 0
        self.__fired = False
        self.__state = ERROR
        self.__detail = detail
        self.__resume_state = ALIGNING
        self.__typing_mode_sent = False
        self.__release_control()
        self.__publish_status(force=True)

    def __plane_is_fresh(self) -> bool:
        if self.__plane is None:
            return False
        return (time.monotonic() - self.__plane_received_at
                <= TYPING_PLANE_TIMEOUT_SEC)

    def __advance(self):
        """Move to the next key or finish the sequence."""
        self.__index += 1
        self.__fired = False
        self.__press_started_at = 0.0
        if self.__index >= len(self.__keys):
            self.__stop_drive()
            self.__state = DONE
            self.__detail = "typing complete"
            self.__release_control()
            self.__publish_status(force=True)
            return
        self.__state = ALIGNING
        self.__detail = "aligning next key"
        self.__publish_status(force=True)

    # ------------------------------------------------------------------
    # Control cycle
    # ------------------------------------------------------------------

    def __update(self):
        """Run one typing step."""
        if self.__estop:
            self.__stop_drive()
            return

        if self.__state in (IDLE, DONE, ERROR):
            self.__stop_drive()
            self.__publish_status()
            return

        if not self.__keys:
            self.__abort("no active key sequence")
            return

        if self.__control_source != ArmControlSource.AUTONOMOUS:
            self.__stop_drive()
            self.__state = WAIT_CONTROL
            self.__detail = "waiting for autonomous control"
            self.__request_control()
            self.__publish_status()
            return

        if not self.__planner_ready:
            self.__stop_drive()
            self.__detail = "waiting for route planner"
            self.__publish_status()
            return

        if not self.__typing_mode_sent:
            self.__publish_command("typing_mode")
            self.__typing_mode_sent = True
            self.__typing_mode_at = time.monotonic()
            self.__state = ALIGNING
            self.__detail = "enabling typing limits"
            self.__publish_status(force=True)
            return
        if (time.monotonic() - self.__typing_mode_at
                < TYPING_MODE_SETTLE_SEC):
            self.__publish_status()
            return

        # A press already in progress does not need a new plane sample.
        if self.__state == PRESSING and self.__fired:
            if time.monotonic() - self.__press_started_at >= (
                    TYPING_PRESS_SETTLE_SEC):
                self.__advance()
            else:
                self.__publish_status()
            return

        if not self.__plane_is_fresh():
            self.__stop_drive()
            self.__hold_arm()
            if self.__state != WAIT_PLANE:
                self.__resume_state = self.__state
            self.__state = WAIT_PLANE
            self.__detail = "waiting for keyboard plane"
            self.__publish_status()
            return

        key = self.__keys[self.__index]
        try:
            target = key_target(self.__plane, key)
            error = alignment_error(self.__plane, key)
        except ValueError as error:
            self.__abort(str(error))
            return

        if self.__state == WAIT_PLANE:
            if self.__resume_state in (MOVING, PRESSING):
                self.__state = self.__resume_state
            else:
                self.__state = ALIGNING

        if self.__state == WAIT_CONTROL:
            self.__state = ALIGNING

        # Keep the base near +/-90 deg by driving the key onto the arm's
        # lateral reach line.  Never move the arm while the rover is
        # creeping: hold the previous target and let the planner hold it.
        if self.__state != PRESSING and abs(error) > TYPING_ALIGN_TOL_M:
            power = drive_power_for_error(error)
            self.__publish_drive(power, power)
            self.__hold_arm()
            self.__state = ALIGNING
            self.__detail = f"aligning {key}: {error * 1000.0:+.1f} mm"
            self.__publish_status()
            return

        self.__stop_drive()

        if self.__state == ALIGNING:
            self.__state = MOVING
            self.__detail = f"moving to {key}"

        self.__publish_pose(target)

        if self.__state == MOVING:
            if (self.__current_pose is not None
                    and pose_matches(self.__current_pose, target)):
                self.__publish_command("fire_solenoid")
                self.__fired = True
                self.__press_started_at = time.monotonic()
                self.__state = PRESSING
                self.__detail = f"pressing {key}"
                self.__publish_status(force=True)
            else:
                self.__detail = f"moving to {key}"
                self.__publish_status()
            return

    def run(self):
        """Spin the node until shutdown."""
        self.get_logger().info("Arm typing node started")
        rclpy.spin(self)
        self.get_logger().info("Arm typing node stopped")

    def destroy_node(self):
        """Best effort: leave the drive at zero."""
        try:
            self.__publish_drive(0.0, 0.0)
        except Exception:
            pass
        return super().destroy_node()


def main():
    """Run the arm typing node."""
    rclpy.init(signal_handler_options=SignalHandlerOptions.ALL)
    node = None
    try:
        node = ArmTypingNode()
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
