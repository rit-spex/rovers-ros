"""Normalize arm teleop input into controller commands.

This node owns the SpaceMouse input semantics: axis signs, deadzone with
hysteresis, power curve, vector-magnitude limiting, sensitivity, and input
staleness.  It publishes normalized teleop intent so the route planner never
has to know which device produced it.

The gripper and solenoid are deliberately not handled here.  They belong to
``arm_end_effector_node`` and are driven by the basestation's up/down arrow
commands, so the SpaceMouse axes are pure 5-DOF motion input.

Current outputs:

* ``/ARM/MANUAL/CMD_VELOCITY`` (``custom_interfaces/Velocity``) - the raw
  manual intent: end-effector linear velocity [m/s] and angular velocity
  [rad/s] in the arm base frame.  ``arm_master_node`` arbitrates this onto
  ``/ARM/CMD_VELOCITY`` while manual owns the arm, so the planner never has
  to know which source produced it.

Shaping values come from the shared constants, not ROS parameters:

* ``TRANSLATION_SENS`` / ``ROTATION_SENS`` - stick scale.
* ``DEADZONE`` / ``ZERO_HYSTERESIS`` / ``INPUT_EXPONENT`` - shaping curve.
* ``TELEOP_INPUT_TIMEOUT_SEC`` - zero an axis whose raw value stopped
  updating.
* ``TELEOP_PUBLISH_RATE_HZ`` - republish rate.

The pure shaping helpers (``normalize``, ``AxisShaper``,
``clamp_vector_norm``, ``is_stale``) live at module scope below so this one
file owns the whole device contract.

Autonomy and other non-teleop sources should feed their own command topics
through a separate input/mux node.  ``custom_interfaces/Pose`` is defined
for absolute target poses.
"""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from custom_interfaces.msg import Velocity, SpaceMouseAxes

from arm_constants import (
    AXIS_SIGNS,
    DEADZONE,
    DT,
    INPUT_EXPONENT,
    MANUAL_VELOCITY_TOPIC,
    METERS_PER_INCH,
    RAW_MAX,
    ROTATION_SENS,
    SPACEMOUSE_AXES_TOPIC,
    SPACEMOUSE_AXIS_NAMES,
    TELEOP_INPUT_TIMEOUT_SEC,
    TELEOP_PUBLISH_RATE_HZ,
    TRANSLATION_SENS,
    ZERO_HYSTERESIS,
)

TELEOP_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

# ----------------------------------------------------------------------
# Pure input shaping
#
# Kept in this module so the teleop node is the single place that knows the
# device semantics.  The functions and ``AxisShaper`` have no ROS dependency
# and are exercised directly by test/test_teleop_inputs.py.
# ----------------------------------------------------------------------


def normalize(raw_value: float, axis_max: float) -> float:
    """Clamp one raw HID axis into [-1, 1]."""
    if axis_max <= 0:
        return 0.0
    return max(-1.0, min(1.0, float(raw_value) / float(axis_max)))


def is_stale(received_at, now: float, timeout: float) -> bool:
    """True when an axis has no sample newer than ``timeout`` seconds.

    ``timeout <= 0`` disables staleness checking.  ``received_at`` is a
    ``time.monotonic()`` value, or None before the first sample.
    """
    if timeout <= 0:
        return False
    if received_at is None:
        return True
    return (now - received_at) > timeout


class AxisShaper:
    """Deadzone, entry hysteresis, and power curve for one axis.

    Leaving rest requires ``deadzone + hysteresis``; falling back to rest
    only requires ``deadzone``.  The gap stops a stick resting on the
    boundary from flickering between zero and motion.
    """

    def __init__(self, deadzone=DEADZONE,
                 exponent=INPUT_EXPONENT,
                 hysteresis=ZERO_HYSTERESIS):
        self.deadzone = max(0.0, min(1.0, float(deadzone)))
        self.exponent = max(0.01, float(exponent))
        self.hysteresis = max(0.0, float(hysteresis))
        self._active = False

    @property
    def active(self) -> bool:
        """True while the axis is outside the deadzone."""
        return self._active

    def process(self, value: float) -> float:
        """Shape one normalized axis value in [-1, 1]."""
        value = max(-1.0, min(1.0, float(value)))
        magnitude = abs(value)

        if self._active:
            if magnitude <= self.deadzone:
                self._active = False
                return 0.0
        else:
            if magnitude <= self.deadzone + self.hysteresis:
                return 0.0
            self._active = True

        span = 1.0 - self.deadzone
        if span <= 0.0:
            return 0.0
        # Rescale from the plain deadzone so the curve itself does not
        # change when hysteresis is reconfigured.
        rescaled = (min(magnitude, 1.0) - self.deadzone) / span
        shaped = rescaled ** self.exponent
        return math.copysign(shaped, value)

    def reset(self) -> None:
        """Return the axis to rest, dropping the hysteresis state."""
        self._active = False


def clamp_vector_norm(values) -> tuple:
    """Scale a vector so its magnitude never exceeds one.

    A per-axis clamp alone lets a diagonal deflection reach sqrt(3) times
    the single-axis speed.  Scaling the whole vector keeps the maximum
    speed the same in every direction.
    """
    values = [float(value) for value in values]
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0:
        return tuple(values)
    return tuple(value / norm for value in values)


class ArmTeleopNode(Node):
    """Turn raw operator axes into normalized arm commands."""

    def __init__(self):
        """Create the axis subscriptions, publishers, and output timer."""
        super().__init__("arm_teleop_node")

        self.__load_tuning()

        # Latched raw axis values.  The basestation republishes teleop axes
        # every cycle, so the node can tell a held stick apart from a dead
        # link.  The timestamp is what makes an axis go stale.
        self.__axes = dict.fromkeys(SPACEMOUSE_AXIS_NAMES, 0)
        self.__axes_received_at = dict.fromkeys(
            SPACEMOUSE_AXIS_NAMES, None)
        self.__stale_axes = set()
        self.__shapers = {
            axis: AxisShaper(
                self.__deadzone, self.__exponent, self.__hysteresis)
            for axis in SPACEMOUSE_AXIS_NAMES
        }

        self.__velocity_publisher = self.create_publisher(
            msg_type=Velocity,
            topic=MANUAL_VELOCITY_TOPIC,
            qos_profile=10,
        )

        self.create_subscription(
            msg_type=SpaceMouseAxes,
            topic=SPACEMOUSE_AXES_TOPIC,
            callback=self.__on_axes_received,
            qos_profile=TELEOP_QOS,
        )

        self.create_timer(1.0 / self.__rate_hz, self.__publish_commands)

    # ------------------------------------------------------------------
    # Tuning
    # ------------------------------------------------------------------

    def __load_tuning(self):
        """Copy the shared tuning values with the same range guards.

        These are deliberately not ROS parameters: nothing overrides them,
        and keeping one source in ``arm_constants`` avoids a second,
        invisible configuration surface.  The guards still protect the node
        from a bad constant edit (zero rate, negative timeout, and so on).
        """
        self.__rate_hz = max(1.0, float(TELEOP_PUBLISH_RATE_HZ))
        self.__translation_sens = max(0.0, float(TRANSLATION_SENS))
        self.__rotation_sens = max(0.0, float(ROTATION_SENS))
        self.__deadzone = min(0.99, max(0.0, float(DEADZONE)))
        self.__hysteresis = max(0.0, float(ZERO_HYSTERESIS))
        self.__exponent = max(0.01, float(INPUT_EXPONENT))
        self.__input_timeout = max(0.0, float(TELEOP_INPUT_TIMEOUT_SEC))

    # ------------------------------------------------------------------
    # Input callbacks
    # ------------------------------------------------------------------

    def __on_axes_received(self, msg: SpaceMouseAxes):
        """Latch one atomic sample of every raw axis."""
        now = time.monotonic()
        for axis in SPACEMOUSE_AXIS_NAMES:
            self.__axes[axis] = int(getattr(msg, axis))
            self.__axes_received_at[axis] = now
            if axis in self.__stale_axes:
                self.__stale_axes.discard(axis)
                self.get_logger().info(
                    f"Teleop axis '{axis}' refreshed; input live again")

    # ------------------------------------------------------------------
    # Command shaping
    # ------------------------------------------------------------------

    def __fresh_axes(self) -> dict:
        """Return raw axes, zeroing any that have gone stale.

        A zeroed axis keeps flowing through the shaper and the hysteresis
        state machine, so a dead link produces a clean zero command instead
        of latched motion.
        """
        now = time.monotonic()
        incumbents = {}
        for axis in SPACEMOUSE_AXIS_NAMES:
            received_at = self.__axes_received_at[axis]
            if is_stale(received_at, now, self.__input_timeout):
                incumbents[axis] = 0
                # Only announce a live axis going stale; stay quiet before
                # the first sample ever arrives.
                if (received_at is not None
                        and axis not in self.__stale_axes):
                    self.__stale_axes.add(axis)
                    self.get_logger().warning(
                        f"Teleop axis '{axis}' is stale; commanding zero")
            else:
                incumbents[axis] = self.__axes[axis]
        return incumbents

    def __shaped_axes(self) -> dict:
        """Shape every raw axis once for this cycle."""
        raw = self.__fresh_axes()
        return {
            axis: self.__shapers[axis].process(
                normalize(raw[axis] * AXIS_SIGNS[axis], RAW_MAX))
            for axis in SPACEMOUSE_AXIS_NAMES
        }

    def __publish_commands(self):
        """Publish the latest normalized velocity."""
        shaped = self.__shaped_axes()

        linear = clamp_vector_norm(
            [shaped["x"], shaped["y"], shaped["z"]])
        angular = clamp_vector_norm(
            [shaped["rx"], shaped["ry"], shaped["rz"]])

        velocity = Velocity()

        # Sensitivity is defined per control cycle; the ROS command is m/s,
        # so convert at this boundary.
        translation = (
            self.__translation_sens / DT * METERS_PER_INCH)
        rotation = self.__rotation_sens / DT
        velocity.linear_x = linear[0] * translation
        velocity.linear_y = linear[1] * translation
        velocity.linear_z = linear[2] * translation
        velocity.angular_x = angular[0] * rotation
        velocity.angular_y = angular[1] * rotation
        velocity.angular_z = angular[2] * rotation

        self.__velocity_publisher.publish(velocity)

    def run(self):
        """Spin the node until it is shut down."""
        self.get_logger().info("Arm teleop started")
        rclpy.spin(self)
        self.get_logger().info("Arm teleop stopped")


def main():
    """Run the arm teleop node."""
    rclpy.init(signal_handler_options=SignalHandlerOptions.ALL)
    node = None
    try:
        node = ArmTeleopNode()
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
