"""Single source of truth for the SPEX arm constants.

Everything the arm package needs lives here: joint order and ROS topic
names, motor indices and calibration, mechanical limits, the home pose,
geometry, CAN message mappings, controller tuning (IK tolerances, octant
rules, path planning), and SpaceMouse sensitivity.
"""

from __future__ import annotations

import math
from enum import IntEnum
from typing import NamedTuple

# The shared workspace ``constants`` ament package resolves normally now that
# this module has its own name.
from constants.CAN_constants import CAN_MESSAGE_IDS
# ``constants.CommandCodes`` puts the ``lib/rovers-protocol`` submodule on
# sys.path.  Arm nodes import ``rover_protocol.arm_errors`` directly, so
# trigger that bridge before any node reaches its own import.
import constants.CommandCodes  # noqa: E402,F401


# =============================================================================
# Joints and ROS topics
# =============================================================================

ARM_JOINTS = (
    "base",
    "shoulder",
    "elbow",
    "wrist_bend",
    "wrist_twist",
    "gripper",
)
# Joints in the 5-DOF model; the gripper is tracked separately.
ARM_JOINT_NAMES = ARM_JOINTS[:-1]

JOINT_TARGET_TOPICS = {
    name: f"/ARM/JOINT/TARGET_{name.upper()}" for name in ARM_JOINTS
}
JOINT_CURRENT_TOPICS = {
    name: f"/ARM/JOINT/CURRENT_{name.upper()}" for name in ARM_JOINTS
}

ESTOP_TOPIC = "/ESTOP"
ARM_COMMAND_TOPIC = "/ARM/COMMAND"

# Teleop input and the arbitrated output are separate topics.  The teleop node
# publishes MANUAL_VELOCITY_TOPIC; the master forwards it to VELOCITY_TOPIC
# only while manual owns the arm, so the planner never needs to know who the
# source is.
MANUAL_VELOCITY_TOPIC = "/ARM/MANUAL/CMD_VELOCITY"
VELOCITY_TOPIC = "/ARM/CMD_VELOCITY"

# Absolute Cartesian target consumed by the route planner.  The typing node
# is the first producer; the master arbitrates it like every other command.
POSE_TOPIC = "/ARM/CMD_POSE"

SOLENOID_TARGET_TOPIC = "/ARM/JOINT/TARGET_SOLENOID"

# Master <-> functional nodes.  The master forwards every /ARM/COMMAND
# message to the planner so it can validate them; the end effector receives
# only the gripper/solenoid commands it executes.  The planner owns all
# 5-DOF target generation and reports errors back through its status topic.
PLANNER_COMMAND_TOPIC = "/ARM/PLANNER/COMMAND"
PLANNER_STATUS_TOPIC = "/ARM/PLANNER/STATUS"
END_EFFECTOR_COMMAND_TOPIC = "/ARM/END_EFFECTOR/COMMAND"

# Commands owned by the end effector node.  The master routes these directly
# to it and never forwards them to the planner.  The up/down arrows arrive
# as a stream while the operator holds the key; the end effector jogs until
# the stream stops for GRIPPER_JOG_TIMEOUT_SEC.
END_EFFECTOR_COMMANDS = {
    "gripper_up",
    "gripper_down",
    "fire_solenoid",
    "toggle_solenoid",
}
GRIPPER_JOG_COMMANDS = {
    "gripper_up",
    "gripper_down",
}
# End-effector commands that are legal before the 5-DOF model has
# initialized.  The solenoid is a one-shot hardware pulse and does not need
# arm feedback.
PRE_READY_END_EFFECTOR_COMMANDS = {
    "fire_solenoid",
    "toggle_solenoid",
}
# Internal master -> end effector command.  The operator cannot send it: it
# is not part of END_EFFECTOR_COMMANDS.  The master emits it on a control
# handover so a held gripper jog stops immediately instead of coasting to its
# timeout.
END_EFFECTOR_STOP_COMMAND = "stop_jog"

# Internal planner command.  The master emits it whenever a control handover
# or E-stop invalidates the previous owner's target, so the planner holds the
# measured pose instead of chasing an abandoned Cartesian target.
PLANNER_HOLD_COMMAND = "hold"

# Autonomous command inputs.  The arm master forwards these only while the
# AUTONOMOUS control source owns the arm, so functional nodes remain
# source-agnostic.  The typing node is the first user of this surface.
AUTONOMOUS_POSE_TOPIC = "/ARM/AUTONOMOUS/CMD_POSE"
AUTONOMOUS_COMMAND_TOPIC = "/ARM/AUTONOMOUS/COMMAND"

# High-level arm state, published as one Bool topic per flag.
ARM_FLAG_NAMES = (
    "HOMING",
    "FLIPPING",
    "FLIPPED",
    "PLANAR",
    "JOINT_DRIVE",
    "POSITION_LOCK",
    "ORIENTATION_LOCK",
    "ZENITH",
)

ARM_STATE_TOPICS = {
    "enabled": "/ARM/STATE/ENABLED",
    "control_source": "/ARM/STATE/CONTROL_SOURCE",
    "locked_joints": "/ARM/STATE/LOCKED_JOINTS",
    "homing_joints": "/ARM/STATE/HOMING_JOINTS",
    "operation_mode": "/ARM/STATE/OPERATION_MODE",
    "error": "/ARM/STATE/ERROR",
    "pose": "/ARM/STATE/POSE",
    "flags": {name: f"/ARM/STATE/{name}" for name in ARM_FLAG_NAMES},
}
ARM_ENABLED_TOPIC = ARM_STATE_TOPICS["enabled"]
ARM_CONTROL_SOURCE_TOPIC = ARM_STATE_TOPICS["control_source"]

# Control ownership.  Sources request a change on the request topic; the arm
# master validates it and is the single writer of the state topic.  Manual
# always preempts autonomous, and NONE means no source may move the arm.
# The default keeps the pre-autonomy behaviour: the operator owns the arm
# until a source explicitly asks for it.
#
# The master is the mux: it is the only node that reads this state to gate
# commands.  Actuators consume only the master's arbitrated output, and the
# state topic is diagnostic telemetry.
ARM_CONTROL_REQUEST_TOPIC = "/ARM/CONTROL/REQUEST"


class ArmControlSource(IntEnum):
    """Which source currently owns arm commands."""

    NONE = 0
    MANUAL = 1
    AUTONOMOUS = 2


ARM_CONTROL_SOURCE_LABELS = {
    ArmControlSource.NONE: "NONE",
    ArmControlSource.MANUAL: "MANUAL",
    ArmControlSource.AUTONOMOUS: "AUTONOMOUS",
}
DEFAULT_ARM_CONTROL_SOURCE = ArmControlSource.MANUAL


def parse_control_source(value):
    """Return the ArmControlSource for a wire value, or None if unknown."""
    try:
        return ArmControlSource(int(value))
    except (TypeError, ValueError):
        return None


def control_source_label(source) -> str:
    """Return a human-readable name for a control source value."""
    return ARM_CONTROL_SOURCE_LABELS.get(
        parse_control_source(source), f"UNKNOWN({source})")


# Arm operating modes: name -> (wire code, human-readable label).
_ARM_OPERATION_MODES = {
    "PNP": (0, "PICK_AND_PLACE"),
    "PNP_FLIPPED": (1, "PICK_AND_PLACE_FLIPPED"),
    "PLANAR": (2, "PLANAR"),
    "HOMING": (3, "HOMING"),
    "ZENITH": (4, "ZENITH"),
}
ARM_OPERATION_MODES = {
    name: code for name, (code, _) in _ARM_OPERATION_MODES.items()
}
# Human-readable labels for the wire codes (used by debug tools).
ARM_OPERATION_MODE_LABELS = {
    code: label for _, (code, label) in _ARM_OPERATION_MODES.items()
}


# =============================================================================
# Geometry and units
# =============================================================================

A0 = 6.32        # Elevation of shoulder joint from base [in]
A1 = 18.25         # Upper arm length [in]
A2 = 19.5         # Fore-arm length [in]
A3 = 12         # Gripper length [in]

# Key joint indices after each linkage translation.
KEY_JOINT_IDX = [1, 3, 5, 8]

RAD2DEG = 57.2957795786
METERS_PER_INCH = 0.0254


# =============================================================================
# Control timing
# =============================================================================

DT = 0.05                   # Arm control period [s], shared by all nodes

# Stop applying teleop motion once the command source goes quiet.
# The planner uses this for /ARM/CMD_VELOCITY.
VELOCITY_TIMEOUT_SEC = 0.5
# The teleop node zeroes every axis when the consolidated
# /BASESTATION/spacemouse/axes sample has not refreshed for this long.  The
# basestation republishes teleop samples every cycle precisely so a dead link
# can be told apart from a held stick.
TELEOP_INPUT_TIMEOUT_SEC = 0.5
# Treat the route planner as unavailable when its status stops arriving.
# Path planning runs in the planner's command callback, so allow several
# seconds before declaring it dead.
PLANNER_TIMEOUT_SEC = 5.0

# An autonomous target is a stream, not a one-shot.  If the producer dies
# while the arm is chasing a pose, hold the measured pose instead of driving
# on a command nobody is refreshing.
AUTONOMOUS_POSE_TIMEOUT_SEC = 0.5

# Teleop republish rate.  The node exposes this as a parameter, and the
# default lives here so the node and its tests share one source.
TELEOP_PUBLISH_RATE_HZ = 50.0

# A held arrow sends a command every basestation cycle.  The master reports
# a rejected hold at most this often, keeping the log and /ARM/STATE/ERROR
# readable.
JOG_REJECT_INTERVAL_SEC = 1.0

# Homing is a maintained action: the basestation forwards the home key on
# every held state, and the planner stops homing when that stream goes quiet.
HOMING_HOLD_TIMEOUT_SEC = 0.25

# A shaped teleop axis is exactly zero at rest, so a tight epsilon rejects
# noise without demanding bit-exact zeros when re-arming manual motion.
VELOCITY_NEUTRAL_EPSILON = 1e-6

# Debug sniffer print cadence.  Slow enough to read, fast enough to see
# motion.
SNIFFER_PERIOD_SEC = 1.0


# =============================================================================
# Home pose, gripper travel, and mechanical limits
# =============================================================================

HOMING_TARGET = [0, math.pi / 4, -math.pi / 4, 0, 0]   # Home: straight ahead

# Zenith test pose: the shoulder is vertical and the arm is fully extended, so
# the motors can be depowered without the arm falling.  The base is left where
# it is ("any"); the remaining joints are shoulder, elbow, wrist bend, wrist
# twist.  The elbow tolerance includes the singularity deadband because a
# straight-up pose may settle just outside zero.
ZENITH_JOINT_TARGET = [math.pi / 2, 0.0, 0.0, 0.0]
ZENITH_ANGLE_TOL = math.radians(5.0)

# Physical gripper travel: 0 = open, 90 deg = closed.
GRIPPER_MIN_ANGLE = 0.0
GRIPPER_MAX_ANGLE = math.pi / 2

# Mechanical per-joint limits [min, max] in radians - hardware bounds.
MECHANICAL_LIMITS = [
    (-math.pi * 3 / 4, math.pi * 3 / 4),   # base
    (math.radians(2), math.radians(178)),   # shoulder
    (-math.pi * 3 / 4, math.pi * 3 / 4),   # elbow
    (-math.pi / 6, math.pi / 6),            # wrist bend
    (-math.pi / 4, math.pi / 4),            # wrist twist
]

BASE_LIMIT = MECHANICAL_LIMITS[0]
SHOULDER_LIMIT = MECHANICAL_LIMITS[1]
ELBOW_LIMIT = MECHANICAL_LIMITS[2]
WRIST_BEND_LIMIT = MECHANICAL_LIMITS[3]
WRIST_TWIST_LIMIT = MECHANICAL_LIMITS[4]


# =============================================================================
# Motor identity and calibration
# =============================================================================

class ARM_MOTOR_IDX(IntEnum):
    """Canonical motor order used by the arm hardware and topics."""

    BASE = 0
    SHOULDER = 1
    ELBOW = 2
    BEND_WRIST = 3
    TWIST_WRIST = 4
    GRIPPER = 5


class ArmMotorParams(NamedTuple):
    """Per-motor encoder calibration, limits, and speed."""

    ticks_per_rad: float
    tick_offset: int
    tick_lower_limit: float
    tick_upper_limit: float
    max_speed: int  # drive profile velocity on the motor shaft [RPM]

    @property
    def max_output_rad_s(self) -> float:
        """Joint speed at the profile-velocity limit [rad/s]."""
        return motor_rpm_to_joint_rad_s(self.max_speed)

    @property
    def max_ticks_per_s(self) -> float:
        """Absolute encoder ticks per second at that joint speed."""
        return abs(self.max_output_rad_s * self.ticks_per_rad)


# Every arm drive's profile velocity is a motor-shaft RPM.  The output joint
# turns at 1/80 of that because of the gearbox.
ARM_MOTOR_GEAR_RATIO = 80


def motor_rpm_to_joint_rad_s(rpm: float) -> float:
    """Convert motor-shaft RPM to output-joint angular speed [rad/s]."""
    return float(rpm) * 2.0 * math.pi / 60.0 / ARM_MOTOR_GEAR_RATIO


# The base and elbow joints use Tinsmith M4230-series integrated robotic
# joint actuators.  Each has a 32768-count encoder per motor revolution and
# an 80:1 gearbox, giving 32768 * 80 / (2 * pi) ~= 417215 ticks per output
# radian.  The sign below is the joint's tick direction; the magnitude is
# shared by both actuators.
TINSMITH_M4230_COUNTS_PER_REV = 32768
TINSMITH_M4230_GEAR_RATIO = ARM_MOTOR_GEAR_RATIO
TINSMITH_M4230_TICKS_PER_RAD = (
    TINSMITH_M4230_COUNTS_PER_REV * TINSMITH_M4230_GEAR_RATIO
    / (2 * math.pi)
)

# NOTE: The actuators above explain the tick scale, but the per-joint offsets,
# tick limits, and the shoulder calibration are still measured values that
# need hardware verification.  Do not use them as a debugging reference.

# https://docs.google.com/spreadsheets/d/1k4CHPq7_ftuCsQyz7srGTpdhJiTCs1ho/edit?gid=788058726#gid=788058726
ARM_MOTOR_PARAMS = [
    ArmMotorParams(
        ticks_per_rad=-TINSMITH_M4230_TICKS_PER_RAD,
        tick_offset=-43333,
        tick_lower_limit=-1040833,
        tick_upper_limit=954000,
        max_speed=500,
    ),  # base
    ArmMotorParams(
        ticks_per_rad=262144,
        tick_offset=-202000,
        tick_lower_limit=-202000,
        tick_upper_limit=622000,
        max_speed=150,
    ),  # shoulder
    ArmMotorParams(
        ticks_per_rad=-TINSMITH_M4230_TICKS_PER_RAD,
        tick_offset=860000,
        tick_lower_limit=-238000,
        tick_upper_limit=1959000,
        max_speed=500,
    ),  # elbow
    # NOTE: Following limits exist on teensy side, currently redundant,
    # also the teensy expects floats in radians soooo yeah
    ArmMotorParams(
        ticks_per_rad=1,
        tick_offset=0,
        tick_lower_limit=-10,
        tick_upper_limit=10,
        max_speed=100,
    ),  # bend wrist
    ArmMotorParams(
        ticks_per_rad=1,
        tick_offset=0,
        tick_lower_limit=-20,
        tick_upper_limit=20,
        max_speed=100,
    ),  # twist wrist
    ArmMotorParams(
        ticks_per_rad=1,
        tick_offset=0,
        tick_lower_limit=GRIPPER_MIN_ANGLE,
        tick_upper_limit=GRIPPER_MAX_ANGLE,
        max_speed=100,
    ),  # gripper
]

# Per-joint speed cap implied by the drive profile velocities.  The planner's
# target slew and the simulated hardware both use this list.
ARM_MOTOR_MAX_ANG_VEL = [
    params.max_output_rad_s for params in ARM_MOTOR_PARAMS
]


def angle_limits(motor_idx: ARM_MOTOR_IDX) -> tuple[float, float]:
    """Return the (lower, upper) joint angle limits [rad] for a motor.

    Tick limits and the offset are converted through the same calibration
    used to talk to the motor, then ordered because a negative
    ``ticks_per_rad`` flips the tick direction.
    """
    params = ARM_MOTOR_PARAMS[motor_idx]
    lower = (params.tick_lower_limit - params.tick_offset) / params.ticks_per_rad
    upper = (params.tick_upper_limit - params.tick_offset) / params.ticks_per_rad
    return (min(lower, upper), max(lower, upper))


# =============================================================================
# CAN mappings
# =============================================================================

# Command message ID for each OpenCAN motor.
JOINT_MESSAGE_IDS = {
    ARM_MOTOR_IDX.BASE:     CAN_MESSAGE_IDS.SEND_BASE,
    ARM_MOTOR_IDX.SHOULDER: CAN_MESSAGE_IDS.SEND_SHOULDER,
    ARM_MOTOR_IDX.ELBOW:    CAN_MESSAGE_IDS.SEND_ELBOW,
}

# Position response message IDs mapped back to their motor.
MOTOR_FOR_MESSAGE = {
    CAN_MESSAGE_IDS.READ_BASE:     ARM_MOTOR_IDX.BASE,
    CAN_MESSAGE_IDS.READ_SHOULDER: ARM_MOTOR_IDX.SHOULDER,
    CAN_MESSAGE_IDS.READ_ELBOW:    ARM_MOTOR_IDX.ELBOW,
}

# Standard (non-OpenCAN) joints: the command frame and signal for each one.
# The wrist axes share a frame but only ever write their own signal.
STANDARD_CAN_ANGLE_SIGNALS = {
    ARM_MOTOR_IDX.BEND_WRIST:  (CAN_MESSAGE_IDS.MOVE_WRIST, "Position_Bend"),
    ARM_MOTOR_IDX.TWIST_WRIST: (CAN_MESSAGE_IDS.MOVE_WRIST, "Position_Twist"),
    ARM_MOTOR_IDX.GRIPPER:     (CAN_MESSAGE_IDS.MOVE_GRIPPER, "Position"),
}

# Standard-CAN position responses mapped back to the motor they report.
STANDARD_CAN_READ_MOTORS = {
    CAN_MESSAGE_IDS.READ_WRIST_BEND:  ARM_MOTOR_IDX.BEND_WRIST,
    CAN_MESSAGE_IDS.READ_WRIST_TWIST: ARM_MOTOR_IDX.TWIST_WRIST,
    CAN_MESSAGE_IDS.READ_GRIPPER:     ARM_MOTOR_IDX.GRIPPER,
}

# Motor order must match ARM_JOINTS.  The topic maps below depend on it.
MOTOR_JOINT_NAMES = dict(zip(ARM_MOTOR_IDX, ARM_JOINTS))
CURRENT_ANGLE_TOPICS = {
    idx: JOINT_CURRENT_TOPICS[name] for idx, name in MOTOR_JOINT_NAMES.items()
}
TARGET_ANGLE_TOPICS = {
    idx: JOINT_TARGET_TOPICS[name] for idx, name in MOTOR_JOINT_NAMES.items()
}


# =============================================================================
# Motion tuning
# =============================================================================

# Conservative scalar speed cap: the slowest joint implied by the drive
# profile velocities.  Per-joint hardware speeds are in ARM_MOTOR_MAX_ANG_VEL
# and are what the planner's slew model uses.
MAX_ANG_VEL = min(ARM_MOTOR_MAX_ANG_VEL)

# Time to ramp from standstill to MAX_ANG_VEL [s].
ANGULAR_SPEED_RAMP_TIME = 1.0

# Teleop output stage.  The planner ramps the Cartesian velocity and the
# controller integrates a joint reference from it.  These are separate from
# ANGULAR_SPEED_RAMP_TIME so autonomous actions keep their slower ramp.
TELEOP_ACCEL_IN_S2 = 9.0          # Cartesian linear acceleration [in/s^2]
TELEOP_DECEL_IN_S2 = 18.0         # Cartesian linear deceleration [in/s^2]
TELEOP_ANG_ACCEL_RAD_S2 = 3.0     # Cartesian angular acceleration [rad/s^2]
TELEOP_JOINT_ACCEL_TIME_S = 0.25  # joint speed ramp time at its own cap [s]
# The commanded joint reference may lead the measured angle by at most this
# much.  It bounds the drive's tracking error; it is a safety limit, not a
# control input.
TELEOP_LEAD_LIMIT_DEG = 5.0
# Consecutive cycles at the lead bound before the planner reports a stall.
TELEOP_LEAD_STALL_CYCLES = 10
# Below this speed the teleop stage is treated as stopped [in/s, rad/s].
TELEOP_STOP_EPSILON = 1e-3


# =============================================================================
# Homing
# =============================================================================

HOMING_SPEED = 1 / 300                        # Radians per step during homing
HOMING_THRESHOLD = 1 / RAD2DEG                # Convergence threshold (~1 deg)


# =============================================================================
# SpaceMouse input
# =============================================================================

# Every axis the SpaceMouse reports, in linear/angular pairs.  The basestation
# publishes all six as one atomic sample on SPACEMOUSE_AXES_TOPIC; the device
# vocabulary lives here so the teleop node and its tests cannot drift apart.
SPACEMOUSE_AXIS_NAMES = ("x", "y", "z", "rx", "ry", "rz")
SPACEMOUSE_AXES_TOPIC = "/BASESTATION/spacemouse/axes"

# Per-axis sign conventions.  +1 = raw HID value used as-is,
# -1 = raw HID value negated.  Adjust here to match device orientation.
# Canonical reference: invKin202627/src/constants.py.  The basestation
# publishes raw HID values, so these signs are applied exactly once.
AXIS_SIGNS = {
    "x":  1,
    "y": -1,
    "z": -1,
    "rx": 1,
    "ry": -1,
    "rz": -1,
}

TRANSLATION_SENS = 0.22575    # SpaceMouse translation scale [in/unit]
ROTATION_SENS = 0.015         # SpaceMouse rotation scale [rad/unit]

# Gripper jog speed while the operator holds the up/down arrow.  The end
# effector moves by GRIPPER_JOG_RATE * DT each control cycle and stops if no
# arrow command arrives for GRIPPER_JOG_TIMEOUT_SEC.
GRIPPER_JOG_RATE = math.radians(30)    # [rad/s]
GRIPPER_JOG_TIMEOUT_SEC = 0.25

# The SpaceMouse reports raw HID values up to about +/-350 (some models a
# little higher).  Normalizing against 350 uses the full stick throw; values
# above it clamp to full scale in ``normalize()``.
RAW_MAX = 350
# A zero sample shorter than this is a sensor dropout, not the operator
# releasing the stick.  The teleop node republishes the last nonzero value
# until the zero has lasted this long [s].
SPACEMOUSE_DROPOUT_HOLD_SEC = 0.1
DEADZONE = 0.05            # Normalized axis deadzone - kills resting-hand drift
INPUT_EXPONENT = 1.5       # Power curve - mild fine-control bias, not mushy
# Extra travel needed to leave rest once the deadzone has zeroed an axis.
# Prevents a stick resting on the boundary from flickering on and off.
ZERO_HYSTERESIS = 0.02

SPACEMOUSE_VENDOR_ID = 0x256f
SPACEMOUSE_PRODUCT_ID = 0xc63a


# =============================================================================
# Keyboard typing
# =============================================================================

# Inputs owned by arm_typing_node.  The plane is published in the rover base
# frame; the key string is literal text (see parse_key_sequence in
# arm_typing/typing_plan.py for the small token syntax).
TYPING_PLANE_TOPIC = "/ARM/TYPING/PLANE"
TYPING_KEYS_TOPIC = "/ARM/TYPING/KEYS"
TYPING_STATUS_TOPIC = "/ARM/TYPING/STATUS"

# Typing drives by publishing the same Xbox axes the basestation publishes.
# chassis_node already subscribes to those topics and applies its normal
# stick inversion, so the typing node needs no autonomous drive topic and no
# chassis-side changes.  Positive power means "stick forward".

# Reference keyboard geometry in the keyboard-plane frame.  Each value is
# ``[x, y, width, height]`` in key units: ``(0, 0)`` is the keyboard centre,
# ``+x`` runs along the keyboard, and ``+y`` runs up the face.  Multiply by
# TYPING_KEY_PITCH_M to get metres.  Measure the real keyboard and edit these
# entries first when the layout does not line up.
TYPING_KEY_PITCH_M = 0.01905          # 19.05 mm = 0.75 in key pitch
TYPING_KEYBOARD = {
    # Top row
    "`": [-7.125, 2, 1, 1],
    "1": [-6.125, 2, 1, 1],
    "2": [-5.125, 2, 1, 1],
    "3": [-4.125, 2, 1, 1],
    "4": [-3.125, 2, 1, 1],
    "5": [-2.125, 2, 1, 1],
    "6": [-1.125, 2, 1, 1],
    "7": [-0.125, 2, 1, 1],
    "8": [0.875, 2, 1, 1],
    "9": [1.875, 2, 1, 1],
    "0": [2.875, 2, 1, 1],
    "-": [3.875, 2, 1, 1],
    "=": [4.875, 2, 1, 1],
    "backspace": [5.875, 2, 2, 1],
    # Tab row
    "tab": [-6.875, 1, 1.5, 1],
    "q": [-5.875, 1, 1, 1],
    "w": [-4.875, 1, 1, 1],
    "e": [-3.875, 1, 1, 1],
    "r": [-2.875, 1, 1, 1],
    "t": [-1.875, 1, 1, 1],
    "y": [-0.875, 1, 1, 1],
    "u": [0.125, 1, 1, 1],
    "i": [1.125, 1, 1, 1],
    "o": [2.125, 1, 1, 1],
    "p": [3.125, 1, 1, 1],
    "[": [4.125, 1, 1, 1],
    "]": [5.125, 1, 1, 1],
    "\\": [6.125, 1, 1.5, 1],
    # Caps row
    "caps": [-6.625, 0, 1.75, 1],
    "a": [-5.625, 0, 1, 1],
    "s": [-4.625, 0, 1, 1],
    "d": [-3.625, 0, 1, 1],
    "f": [-2.625, 0, 1, 1],
    "g": [-1.625, 0, 1, 1],
    "h": [-0.625, 0, 1, 1],
    "j": [0.375, 0, 1, 1],
    "k": [1.375, 0, 1, 1],
    "l": [2.375, 0, 1, 1],
    ";": [3.375, 0, 1, 1],
    "'": [4.375, 0, 1, 1],
    "enter": [5.375, 0, 2.25, 1],
    # Shift row
    "shift": [-6.375, -1, 2.25, 1],
    "z": [-5.375, -1, 1, 1],
    "x": [-4.375, -1, 1, 1],
    "c": [-3.375, -1, 1, 1],
    "v": [-2.375, -1, 1, 1],
    "b": [-1.375, -1, 1, 1],
    "n": [-0.375, -1, 1, 1],
    "m": [0.625, -1, 1, 1],
    ",": [1.625, -1, 1, 1],
    ".": [2.625, -1, 1, 1],
    "/": [3.625, -1, 1, 1],
    # Bottom row
    "ctrl": [-7.125, -2, 1.25, 1],
    "alt": [-6.125, -2, 1.25, 1],
    "space": [0, -2, 6.25, 1],
}

# Typing control loop and safety.  The drive cap is deliberately below the
# normal teleop authority: the node is only allowed to creep along the
# keyboard edge.
TYPING_RATE_HZ = 10.0
TYPING_DRIVE_MAX_POWER = 0.10
TYPING_DRIVE_GAIN = 1.5               # power per metre of edge error
TYPING_ALIGN_DEADBAND_M = 0.005
TYPING_ALIGN_TOL_M = 0.010
TYPING_STANDOFF_M = 0.06              # tool tip distance from the plane
TYPING_ARM_POS_TOL_M = 0.012
TYPING_ARM_ORI_TOL_RAD = 0.15
TYPING_PRESS_SETTLE_SEC = 0.35
TYPING_MODE_SETTLE_SEC = 0.2
TYPING_PLANE_TIMEOUT_SEC = 0.5
TYPING_CONTROL_RETRY_SEC = 1.0
# A keyboard that is nearly in front of the rover would force the base away
# from +/-90 deg.  Warn below this lateral offset instead of silently
# pretending the side is well defined.
TYPING_MIN_SIDE_OFFSET_M = 0.10
TYPING_STATUS_PERIOD_SEC = 0.5


# =============================================================================
# Target frame behavior
# =============================================================================

MAX_TRANSLATION_DELTA = 1.0  # Max target translation per frame [inches]

# Orientation snapping: every target axis (n, o, a) must have one dominant
# world-axis component above this magnitude and both other components below
# ORIENTATION_SNAP_OFF_AXIS.  Then the desired target is snapped to the
# nearest exact axis-aligned orientation.
ORIENTATION_SNAP_THRESHOLD = 0.95
ORIENTATION_SNAP_OFF_AXIS = 0.25


# =============================================================================
# Inverse-kinematics solver
# =============================================================================

POS_TOL = 0.01                   # Position convergence tolerance [in]
ORI_TOL = 0.1                    # Orientation tolerance [rad]
MAX_STEP = math.pi / 6           # Max angle change per Newton step [rad]
FD_EPS = 1e-6                    # Jacobian finite-difference step [rad]
DAMPING_LAMBDA_SQ = 1e-3         # DLS damping lambda^2
LIMIT_TOL = 1e-9                 # Bound-proximity tolerance [rad]
MAX_BACKTRACK = 6                # Step halvings before rejecting a step
BACKTRACK_TOL = 1e-5             # Improvement threshold for stagnation
MAX_ACTIVE_SET_PASSES = 5        # Extra active-set re-solves


# =============================================================================
# Controller tracking
# =============================================================================

TRACK_POS_TOL = 1e-4             # Position-drive early-exit tolerance [in]
TRACK_ORI_TOL = 1e-4             # Orientation-drive early-exit tolerance [rad]


# =============================================================================
# Workspace and limiter behavior
# =============================================================================

ELBOW_DEADBAND = 0.2               # Radians - keep the elbow away from singularity
O3_RELAX_MAX = math.radians(35)    # Max overshoot excused by octant relief
EXCLUSION_RADIUS = 6.0             # End effector keep-out radius around the base [in]
BASE_ZERO_RANGE = (math.radians(-5), math.radians(5))  # "Base near zero" band
END_EFFECTOR_MIN_Z = -20.0         # Lower height bound for the end effector [in]


# =============================================================================
# Joint limits
# =============================================================================

# Per-mode joint limits [min, max] in radians.
# Modes that use mechanical bounds reference MECHANICAL_LIMITS directly.

# Full Range: mechanical base limits; dynamic overrides (shoulder
# expansion, octant relief, octant curl) are applied at runtime by
# FullRangeMode.get_limits().
FULL_RANGE_LIMITS = list(MECHANICAL_LIMITS)

# Homing: mechanical bounds only - no dynamic overrides
HOMING_LIMITS = list(MECHANICAL_LIMITS)

# Planar: base locked to +/-5 deg, shoulder 0-180 deg, rest mechanical
PLANAR_LIMITS = [
    (-math.radians(5), math.radians(5)),    # base restricted to +/-5 deg
    (0, math.pi),                           # shoulder 0-180 deg
    MECHANICAL_LIMITS[2],                   # elbow (mechanical)
    MECHANICAL_LIMITS[3],                   # wrist bend (mechanical)
    MECHANICAL_LIMITS[4],                   # wrist twist (mechanical)
]

# Pick and Place: base mechanical, shoulder 0-90 deg, elbow -135-0 deg,
# wrist mechanical (reach-down workspace for pick-and-place tasks)
PICK_AND_PLACE_LIMITS = [
    MECHANICAL_LIMITS[0],                   # base (mechanical)
    (0, math.radians(90)),                  # shoulder 0-90 deg
    (-math.pi * 3 / 4, 0),                  # elbow -135-0 deg
    MECHANICAL_LIMITS[3],                   # wrist bend (mechanical)
    MECHANICAL_LIMITS[4],                   # wrist twist (mechanical)
]

# Pick and Place Flipped: mirror of Pick and Place - shoulder 90-180 deg,
# elbow 0-135 deg (reach-down on the opposite side)
PICK_AND_PLACE_FLIPPED_LIMITS = [
    MECHANICAL_LIMITS[0],                   # base (mechanical)
    (math.radians(90), math.pi),            # shoulder 90-180 deg
    (0, math.pi * 3 / 4),                   # elbow 0-135 deg
    MECHANICAL_LIMITS[3],                   # wrist bend (mechanical)
    MECHANICAL_LIMITS[4],                   # wrist twist (mechanical)
]

# Typing: the keyboard sits on the rover's left or right, so the base stays
# within +/-90 deg and the drivebase moves the next key onto the reach line.
# Shoulder, elbow, and wrist use their mechanical travel; the wrist locks that
# guard teleop are released while this mode is active.
TYPING_LIMITS = [
    (-math.pi / 2, math.pi / 2),            # base limited to the sides
    MECHANICAL_LIMITS[1],                   # shoulder (mechanical)
    MECHANICAL_LIMITS[2],                   # elbow (mechanical)
    MECHANICAL_LIMITS[3],                   # wrist bend (mechanical)
    MECHANICAL_LIMITS[4],                   # wrist twist (mechanical)
]


# =============================================================================
# Octant conventions and rules
# =============================================================================
#
# World axes: +Y = front, -Y = back, +X = right, -X = left,
#             +Z = up/high, -Z = down/low.
#
# Numbering:
#   High (z >= 0): 1 Front, 2 Right, 3 Back, 4 Left
#   Low  (z <  0): 5 Front, 6 Right, 7 Back, 8 Left

FRONT_HIGH = 1
RIGHT_HIGH = 2
BACK_HIGH = 3
LEFT_HIGH = 4
FRONT_LOW = 5
RIGHT_LOW = 6
BACK_LOW = 7
LEFT_LOW = 8

OCTANT_NAMES = {
    FRONT_HIGH: "Front High (+Y)",
    RIGHT_HIGH: "Right High (+X)",
    BACK_HIGH: "Back High (-Y)",
    LEFT_HIGH: "Left High (-X)",
    FRONT_LOW: "Front Low (+Y)",
    RIGHT_LOW: "Right Low (+X)",
    BACK_LOW: "Back Low (-Y)",
    LEFT_LOW: "Left Low (-X)",
}

OCTANT_RELIEF = (BACK_HIGH, FRONT_LOW, BACK_LOW)
OCTANT_CURL = (RIGHT_LOW, LEFT_LOW)

# Controller-side octant gates (imported by RoverArmController).
OCTANT_PLANAR_EXIT_LOCK = (BACK_HIGH, FRONT_LOW, BACK_LOW)
OCTANT_DIRECT_FLIP = (
    RIGHT_HIGH, LEFT_HIGH, RIGHT_LOW, LEFT_LOW,
)

# Mode-specific workspace gates.  Normal and Flipped stay out of the
# restricted relief octants; Planar has a single valid octant per side.
OCTANT_PLANAR_ALLOWED = (FRONT_HIGH, FRONT_LOW)
OCTANT_PLANAR_FLIPPED_ALLOWED = (BACK_HIGH, BACK_LOW)
OCTANT_NORMAL_ALLOWED = (
    FRONT_HIGH, RIGHT_HIGH, LEFT_HIGH, RIGHT_LOW, LEFT_LOW,
)

OCTANT_POSITION_LIMITS = {
    FRONT_HIGH: ((-60, 60), (-60, 60), (0, 60)),
    RIGHT_HIGH: ((-60, 60), (-60, 60), (0, 60)),
    BACK_HIGH: ((-60, 60), (-60, 60), (0, 60)),
    LEFT_HIGH: ((-60, 60), (-60, 60), (0, 60)),
    FRONT_LOW: ((-60, 60), (-60, 60), (END_EFFECTOR_MIN_Z, 0)),
    RIGHT_LOW: ((-60, 60), (-60, 60), (END_EFFECTOR_MIN_Z, 0)),
    BACK_LOW: ((-60, 60), (-60, 60), (END_EFFECTOR_MIN_Z, 0)),
    LEFT_LOW: ((-60, 60), (-60, 60), (END_EFFECTOR_MIN_Z, 0)),
}

OCTANT_RULES = {
    FRONT_HIGH: "position limits only",
    RIGHT_HIGH: "position limits only",
    BACK_HIGH: "position limits + upper-limit relief",
    LEFT_HIGH: "position limits only",
    FRONT_LOW: "position limits + upper-limit relief",
    RIGHT_LOW: "position limits + elbow curl",
    BACK_LOW: "position limits + upper-limit relief",
    LEFT_LOW: "position limits + elbow curl",
}


# =============================================================================
# Path planning
# =============================================================================

# Restricted octants are safe while the base stays inside the planar band.
PLANAR_BASE = max(abs(PLANAR_LIMITS[0][0]), abs(PLANAR_LIMITS[0][1]))
PATH_PLANNER_STEP = math.radians(5.0)   # Search-grid step [rad]
PATH_PLANNER_MAX_EXPANSIONS = 200_000   # A* expansion cap

# Waypoint follower tolerance and stall guard.  The follower compares the
# measured pose against a waypoint, so encoder noise and tracking lag mean an
# exact match would never arrive.  Advance once every joint is within the
# homing threshold, and skip a waypoint that makes no progress for this many
# control cycles.
WAYPOINT_TOL = HOMING_THRESHOLD
WAYPOINT_STALL_LIMIT = 40               # ~2 s at DT
# Leave headroom for waypoint-following lag: the arm stops within
# WAYPOINT_TOL of a waypoint, so a plan that hugs the exact planar-band
# boundary can drift outside it while closing on the next point.
PATH_PLANAR_BASE = max(0.0, PLANAR_BASE - WAYPOINT_TOL)
