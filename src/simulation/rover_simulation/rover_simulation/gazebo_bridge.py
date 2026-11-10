"""Bridge between the rover arm ROS topics and the Gazebo arm model.

The Gazebo arm is driven by a ``ForwardCommandController`` (position
interface).  This node translates the rover's existing
``/ARM/JOINT/TARGET_*`` topics into that controller's command array, and
republishes Gazebo joint states on the ``/ARM/JOINT/CURRENT_*`` topics so the
inverse-kinematics node (and telemetry) see the simulated arm exactly like
real motor feedback.

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

# Simulation-only randomized start (home +/- 0.2 rad, Pick & Place limits).
_HOME = [0.0, math.pi / 4, -math.pi / 4, 0.0, 0.0]
_LIMITS = [
    (-math.pi * 3 / 4, math.pi * 3 / 4),
    (0.0, math.pi / 2),
    (-math.pi * 3 / 4, 0.0),
    (-math.pi / 6, math.pi / 6),
    (-math.pi / 2, math.pi / 2),
]


def random_home_pose():
    """Randomized, valid arm pose for Gazebo simulation only."""
    angles = []
    for home, (lo, hi) in zip(_HOME, _LIMITS):
        angle = home + random.uniform(-0.2, 0.2)
        angles.append(max(lo, min(hi, angle)))
    angles.append(0.0)  # gripper
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
        self.__last_published = [None] * len(JOINTS)
        self.__estop = False
        self.__started = False
        self.__settled = not self.__randomize_start
        self.__settle_frames = 0

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
        self.create_subscription(
            Float32, "/ARM/JOINT/TARGET_BASE",
            self.__target_callback(0), 10)
        self.create_subscription(
            Float32, "/ARM/JOINT/TARGET_SHOULDER",
            self.__target_callback(1), 10)
        self.create_subscription(
            Float32, "/ARM/JOINT/TARGET_ELBOW",
            self.__target_callback(2), 10)
        self.create_subscription(
            Float32, "/ARM/JOINT/TARGET_WRIST_BEND",
            self.__target_callback(3), 10)
        self.create_subscription(
            Float32, "/ARM/JOINT/TARGET_WRIST_TWIST",
            self.__target_callback(4), 10)
        self.create_subscription(
            Float32, "/ARM/JOINT/TARGET_GRIPPER",
            self.__target_callback(5), 10)

        # Re-send the held position so a late-starting controller gets it.
        self.create_timer(0.1, self.__publish_command)
        # Republish current angles so a late-starting subscriber (e.g. the IK
        # node) does not miss a stationary arm's only encoder message.
        self.create_timer(0.5, self.__republish_current)

    def __target_callback(self, index):
        def callback(msg: Float32):
            if self.__estop:
                return
            self.__targets[index] = float(msg.data)
            self.__publish_command()
        return callback

    def __on_estop_received(self, msg: Bool):
        estop = bool(msg.data)
        if estop and not self.__estop:
            if all(not math.isnan(value) for value in self.__current):
                self.__targets = list(self.__current)
                self.__publish_command()
            self.get_logger().info(
                "E-STOP received, holding current arm pose")
        elif not estop and self.__estop:
            self.get_logger().info("E-STOP cleared, arm targets re-enabled")
        self.__estop = estop

    def __on_joint_state(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            if name not in JOINTS:
                continue
            index = JOINTS.index(name)
            self.__current[index] = float(position)

            if (self.__settled
                    and self.__last_published[index]
                    != self.__current[index]):
                self.__last_published[index] = self.__current[index]
                self.__current_publishers[index].publish(
                    Float32(data=self.__current[index]))

        if self.__started:
            self.__check_settled()
            return

        if any(math.isnan(value) for value in self.__current):
            return

        if self.__randomize_start and not self.__estop:
            self.__targets = random_home_pose()
            self.get_logger().info(
                "Gazebo arm starting from randomized angles "
                f"{[round(math.degrees(a), 1) for a in self.__targets[:5]]}")
        else:
            self.__targets = list(self.__current)
        self.__started = True
        self.__publish_command()
        self.__check_settled()

    def __check_settled(self):
        """For randomized starts, wait for Gazebo to reach the new pose."""
        if self.__settled:
            return
        self.__settle_frames += 1
        reached = all(
            abs(current - target) <= 0.05
            for current, target in zip(self.__current, self.__targets))
        if not reached and self.__settle_frames < 300:
            return
        self.__settled = True
        for index, value in enumerate(self.__current):
            self.__last_published[index] = value
            self.__current_publishers[index].publish(Float32(data=value))
        self.get_logger().info(
            "Gazebo arm reached its randomized start pose")

    def __republish_current(self):
        if not self.__settled:
            return
        for index, value in enumerate(self.__current):
            if not math.isnan(value):
                self.__current_publishers[index].publish(Float32(data=value))

    def __publish_command(self):
        if any(math.isnan(value) for value in self.__targets):
            return
        targets = list(self.__targets)

        # The base is continuous.  Take the shortest angular path from the
        # current position so a wrapped target (e.g. 225 deg for -135 deg)
        # does not spin the joint the long way around.
        if not math.isnan(self.__current[0]):
            targets[0] = self.__current[0] + math.remainder(
                targets[0] - self.__current[0], 2 * math.pi)

        self.__command_publisher.publish(
            Float64MultiArray(data=targets))


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
