"""Bridge between the rover arm ROS topics and the Gazebo arm model.

The Gazebo arm is driven by a ``ForwardCommandController`` (position
interface).  This node translates the rover's existing
``/ARM/JOINT/TARGET_*`` topics into that controller's command array, and
republishes Gazebo joint states on the ``/ARM/JOINT/CURRENT_*`` topics so the
arm controller and telemetry see the simulated arm like real motor feedback.

Joint order matches ``rover_simulation/config/my_controllers.yaml``:
    base, shoulder, elbow, wrist bend, wrist twist, gripper
"""

import math
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float64MultiArray

TICK_PERIOD_SEC = 0.05
SETTLE_TOLERANCE_RAD = 0.05
MAX_SETTLE_TICKS = 300

JOINTS = [
    "arm_base_joint",
    "arm_shoulder_joint",
    "arm_elbow_joint",
    "arm_wrist_bend_joint",
    "arm_wrist_twist_joint",
    "arm_gripper_joint",
]

CURRENT_TOPICS = [
    "/ARM/JOINT/CURRENT_BASE",
    "/ARM/JOINT/CURRENT_SHOULDER",
    "/ARM/JOINT/CURRENT_ELBOW",
    "/ARM/JOINT/CURRENT_WRIST_BEND",
    "/ARM/JOINT/CURRENT_WRIST_TWIST",
    "/ARM/JOINT/CURRENT_GRIPPER",
]

# Simulation-only randomized start: base/shoulder/elbow at home +/- 20 deg.
# Wrist bend, wrist twist, and gripper always start at zero.
START_JITTER_RAD = math.radians(20.0)
_HOME = [0.0, math.pi / 4, -math.pi / 4, 0.0, 0.0]
_LIMITS = [
    (-math.pi * 3 / 4, math.pi * 3 / 4),
    (0.0, math.pi / 2),
    (-math.pi * 3 / 4, 0.0),
    (-math.pi / 6, math.pi / 6),
    (-math.pi / 2, math.pi / 2),
]


def random_home_pose():
    """Randomized, valid arm pose for Gazebo simulation only.

    Only the three main joints jitter around home.  Both wrist axes and the
    gripper always start at zero.
    """
    angles = []
    for home, (lo, hi) in zip(_HOME[:3], _LIMITS[:3]):
        angle = home + random.uniform(-START_JITTER_RAD, START_JITTER_RAD)
        angles.append(max(lo, min(hi, angle)))
    angles.extend([0.0, 0.0, 0.0])  # wrist bend, wrist twist, gripper
    return angles


class ArmGazeboBridge(Node):
    """Translate the arm's target/current topics to/from Gazebo joints."""

    def __init__(self):
        super().__init__("arm_gazebo_bridge")

        self.declare_parameter("command_topic", "/GAZEBO/ARM/arm_cont/commands")
        command_topic = self.get_parameter(
            "command_topic").get_parameter_value().string_value

        self.declare_parameter("randomize_start", False)
        self.__randomize_start = self.get_parameter(
            "randomize_start").get_parameter_value().bool_value

        self.__targets = [float("nan")] * len(JOINTS)
        self.__current = [float("nan")] * len(JOINTS)
        self.__estop = False
        self.__started = False
        self.__settling = False
        self.__settle_ticks = 0

        self.__command_publisher = self.create_publisher(
            Float64MultiArray, command_topic, 10)
        self.__current_publishers = [
            self.create_publisher(Float32, topic, 10)
            for topic in CURRENT_TOPICS
        ]

        self.create_subscription(
            JointState, "/GAZEBO/joint_states", self.__on_joint_state, 10)
        self.create_subscription(
            Bool, "/ESTOP", self.__on_estop_received, 10)
        for index, topic in enumerate((
                "/ARM/JOINT/TARGET_BASE",
                "/ARM/JOINT/TARGET_SHOULDER",
                "/ARM/JOINT/TARGET_ELBOW",
                "/ARM/JOINT/TARGET_WRIST_BEND",
                "/ARM/JOINT/TARGET_WRIST_TWIST",
                "/ARM/JOINT/TARGET_GRIPPER")):
            self.create_subscription(
                Float32, topic, self.__target_callback(index), 10)

        self.create_timer(TICK_PERIOD_SEC, self.__tick)

    def __target_callback(self, index):
        def callback(msg: Float32):
            if self.__estop or self.__settling:
                return
            self.__targets[index] = float(msg.data)
        return callback

    def __on_estop_received(self, msg: Bool):
        estop = bool(msg.data)
        if estop and not self.__estop:
            self.get_logger().info(
                "E-STOP received, holding current arm pose")
        elif not estop and self.__estop:
            self.get_logger().info("E-STOP cleared, arm targets re-enabled")
        self.__estop = estop

    def __on_joint_state(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            if name in JOINTS:
                self.__current[JOINTS.index(name)] = float(position)

    def __tick(self):
        if not self.__started:
            if any(math.isnan(value) for value in self.__current):
                return
            if self.__estop:
                # The master latches E-STOP until the basestation heartbeat
                # arrives.  Choosing a start pose now would freeze the arm at
                # Gazebo's zero pose and skip the randomized home pose.
                return
            if self.__randomize_start:
                self.__targets = random_home_pose()
                self.__settling = True
                self.get_logger().info(
                    "Gazebo arm starting from randomized angles "
                    f"{[round(math.degrees(a), 1) for a in self.__targets]}")
            else:
                self.__targets = list(self.__current)
            self.__started = True

        if self.__estop:
            self.__targets = list(self.__current)

        self.__publish_targets()

        if self.__settling:
            self.__settle_ticks += 1
            reached = all(
                abs(current - target) <= SETTLE_TOLERANCE_RAD
                for current, target in zip(self.__current, self.__targets))
            if not reached and self.__settle_ticks < MAX_SETTLE_TICKS:
                return
            self.__settling = False
            self.get_logger().info(
                "Gazebo arm reached its randomized start pose")

        self.__publish_current()

    def __publish_targets(self):
        if any(math.isnan(value) for value in self.__targets):
            return
        targets = list(self.__targets)
        # The base is continuous.  Take the shortest angular path from the
        # current position so a wrapped target does not spin the long way.
        if not math.isnan(self.__current[0]):
            targets[0] = self.__current[0] + math.remainder(
                targets[0] - self.__current[0], 2 * math.pi)
        self.__command_publisher.publish(Float64MultiArray(data=targets))

    def __publish_current(self):
        for publisher, value in zip(self.__current_publishers, self.__current):
            if not math.isnan(value):
                publisher.publish(Float32(data=value))


def main():
    rclpy.init()
    node = ArmGazeboBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
