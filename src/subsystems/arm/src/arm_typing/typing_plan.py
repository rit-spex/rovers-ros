"""Pure geometry and keyboard helpers for ``arm_typing_node``.

Nothing in this module imports ROS.  The node owns subscriptions, timing,
and command publishing; this module owns the parts that are easy to test:
the reference keyboard layout, the key-list parser, the plane-to-key
transform, and the small drive/arrival policies.
"""

from __future__ import annotations

import math
from typing import NamedTuple

from arm_constants import (
    TYPING_ARM_ORI_TOL_RAD,
    TYPING_ARM_POS_TOL_M,
    TYPING_ALIGN_DEADBAND_M,
    TYPING_DRIVE_GAIN,
    TYPING_DRIVE_MAX_POWER,
    TYPING_KEY_PITCH_M,
    TYPING_KEYBOARD,
    TYPING_STANDOFF_M,
    TYPING_MIN_SIDE_OFFSET_M,
)


class PlaneFrame(NamedTuple):
    """Keyboard plane expressed in the rover base frame.

    ``origin`` is the keyboard centre in metres.  The three axis vectors are
    unit columns: ``x_axis`` runs along the keyboard, ``y_axis`` runs up the
    face, and ``z_axis`` points out of the keyboard toward the rover.
    """

    origin: tuple[float, float, float]
    x_axis: tuple[float, float, float]
    y_axis: tuple[float, float, float]
    z_axis: tuple[float, float, float]
    side: int


class KeyTarget(NamedTuple):
    """One key pose in the rover base frame (metres and radians)."""

    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


def clamp(value: float, low: float, high: float) -> float:
    """Clamp ``value`` into the inclusive range ``[low, high]``."""
    return max(low, min(high, float(value)))


def wrap_angle(angle: float) -> float:
    """Wrap an angle to ``[-pi, pi)``."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> tuple:
    """Return ``Rz(yaw) @ Ry(pitch) @ Rx(roll)`` as a row-major tuple."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def rotation_to_rpy(rotation) -> tuple[float, float, float]:
    """Return ``(roll, pitch, yaw)`` for a ``Rz @ Ry @ Rx`` matrix.

    This matches ``arm_route_planner_node.rotation_to_rpy`` so a target built
    here is interpreted identically by the planner.
    """
    r20 = clamp(rotation[2][0], -1.0, 1.0)
    pitch = math.asin(-r20)
    if abs(r20) < 0.999999:
        roll = math.atan2(rotation[2][1], rotation[2][2])
        yaw = math.atan2(rotation[1][0], rotation[0][0])
    else:
        roll = 0.0
        yaw = math.atan2(-rotation[0][1], rotation[1][1])
    return roll, pitch, yaw


def build_key_positions(
    keyboard=TYPING_KEYBOARD,
    pitch=TYPING_KEY_PITCH_M,
) -> dict:
    """Build key centre coordinates ``(u, v)`` in metres.

    The keyboard dictionary stores ``[x, y, width, height]`` in key units,
    with the keyboard centred on ``(0, 0)``.  Multiply the centre by the
    physical key pitch to get the plane offsets used by the pose solver.
    """
    pitch = float(pitch)
    return {
        name: (float(values[0]) * pitch, float(values[1]) * pitch)
        for name, values in keyboard.items()
    }


def build_key_sizes(
    keyboard=TYPING_KEYBOARD,
    pitch=TYPING_KEY_PITCH_M,
) -> dict:
    """Build key ``(width, height)`` extents in metres."""
    pitch = float(pitch)
    return {
        name: (float(values[2]) * pitch, float(values[3]) * pitch)
        for name, values in keyboard.items()
    }


KEY_POSITIONS = build_key_positions()
KEY_SIZES = build_key_sizes()


def parse_key_sequence(text: str,
                       positions=KEY_POSITIONS) -> list[str]:
    """Turn a literal key string into key names from the reference layout.

    Spaces, tabs, and newlines become ``space``, ``tab``, and ``enter``.
    Braced names such as ``{space}`` or ``{backspace}`` are also accepted.
    Uppercase letters and shifted symbols are rejected: the solenoid is a
    momentary striker and cannot hold Shift while another key is pressed.
    """
    keys = []
    index = 0
    while index < len(text):
        char = text[index]
        if char == "{":
            end = text.find("}", index + 1)
            if end < 0:
                raise ValueError(f"unterminated key token in {text!r}")
            name = text[index + 1:end].strip().lower()
            if not name:
                raise ValueError("empty key token")
            _require_key(name, positions)
            keys.append(name)
            index = end + 1
            continue
        if char == "\r":
            index += 1
            continue
        if char == "\n":
            keys.append("enter")
        elif char == "\t":
            keys.append("tab")
        elif char == " ":
            keys.append("space")
        elif char.isalpha() and char.isupper():
            raise ValueError(
                "uppercase letters are not supported; the solenoid cannot "
                "hold Shift")
        else:
            _require_key(char.lower(), positions)
            keys.append(char.lower())
        index += 1
    return keys


def _require_key(name: str, positions) -> None:
    if name not in positions:
        raise ValueError(f"key {name!r} is not in the reference layout")


def plane_frame(x: float, y: float, z: float,
                roll: float, pitch: float, yaw: float,
                side: int = 0) -> PlaneFrame:
    """Build a :class:`PlaneFrame` from the plane pose message fields.

    ``side`` wins when it is non-zero.  Otherwise +X means the keyboard is on
    the rover's right (base near -90 deg) and -X means the left (base near
    +90 deg).  A plane almost straight ahead is still given a side so the
    node can report the geometry; the node warns about it separately.
    """
    rotation = rotation_from_rpy(roll, pitch, yaw)
    if side > 0:
        selected = 1
    elif side < 0:
        selected = -1
    else:
        selected = 1 if x < 0.0 else -1
    return PlaneFrame(
        origin=(float(x), float(y), float(z)),
        x_axis=tuple(rotation[i][0] for i in range(3)),
        y_axis=tuple(rotation[i][1] for i in range(3)),
        z_axis=tuple(rotation[i][2] for i in range(3)),
        side=selected,
    )


def plane_is_sideways(plane: PlaneFrame,
                      min_offset: float = TYPING_MIN_SIDE_OFFSET_M) -> bool:
    """Return True when the plane origin is clearly to the left or right."""
    return abs(float(plane.origin[0])) >= max(0.0, float(min_offset))


def key_offset(key: str, positions=KEY_POSITIONS) -> tuple[float, float]:
    """Return the ``(u, v)`` offset of ``key`` from the keyboard centre."""
    try:
        return positions[key]
    except KeyError as error:
        raise ValueError(f"unknown key {key!r}") from error


def key_point_in_rover(plane: PlaneFrame, key: str,
                       positions=KEY_POSITIONS) -> tuple[float, float, float]:
    """Return the key's centre point in the rover base frame."""
    u, v = key_offset(key, positions)
    origin = plane.origin
    return (
        origin[0] + u * plane.x_axis[0] + v * plane.y_axis[0],
        origin[1] + u * plane.x_axis[1] + v * plane.y_axis[1],
        origin[2] + u * plane.x_axis[2] + v * plane.y_axis[2],
    )


def key_target(plane: PlaneFrame, key: str,
               standoff: float = TYPING_STANDOFF_M,
               positions=KEY_POSITIONS) -> KeyTarget:
    """Return the arm pose that places the tool at ``key``.

    The tool approaches along ``-plane.z_axis`` (into the keyboard), keeps
    the keyboard's horizontal axis as tool +X, and keeps the keyboard's up
    axis as tool +Z.  The tool tip sits ``standoff`` metres off the face so
    the solenoid, not the arm, makes the press.
    """
    u, v = key_offset(key, positions)
    origin = plane.origin
    x_axis = plane.x_axis
    y_axis = plane.y_axis
    z_axis = plane.z_axis
    position = (
        origin[0] + u * x_axis[0] + v * y_axis[0] + standoff * z_axis[0],
        origin[1] + u * x_axis[1] + v * y_axis[1] + standoff * z_axis[1],
        origin[2] + u * x_axis[2] + v * y_axis[2] + standoff * z_axis[2],
    )
    # Tool columns: +X along the keyboard, +Y into the face, +Z up the face.
    rotation = (
        (x_axis[0], -z_axis[0], y_axis[0]),
        (x_axis[1], -z_axis[1], y_axis[1]),
        (x_axis[2], -z_axis[2], y_axis[2]),
    )
    roll, pitch, yaw = rotation_to_rpy(rotation)
    return KeyTarget(position[0], position[1], position[2],
                     roll, pitch, yaw)


def alignment_error(plane: PlaneFrame, key: str,
                    target_y: float = 0.0,
                    positions=KEY_POSITIONS) -> float:
    """Return how far the key is from the arm's lateral reach line.

    The rover drives along its +Y axis, so the error is the key's rover-frame
    Y coordinate minus ``target_y``.  Positive error means the rover should
    move forward.
    """
    return key_point_in_rover(plane, key, positions)[1] - float(target_y)


def drive_power_for_error(error: float,
                          max_power: float = TYPING_DRIVE_MAX_POWER,
                          gain: float = TYPING_DRIVE_GAIN,
                          deadband: float = TYPING_ALIGN_DEADBAND_M) -> float:
    """Map a lateral error in metres to a capped forward/back drive power."""
    max_power = max(0.0, float(max_power))
    if abs(float(error)) <= max(0.0, float(deadband)):
        return 0.0
    power = float(error) * float(gain)
    return clamp(power, -max_power, max_power)


def orientation_error(current, target) -> float:
    """Largest wrapped difference between two roll/pitch/yaw triples."""
    return max(
        abs(wrap_angle(float(a) - float(b)))
        for a, b in zip(current, target)
    )


def pose_matches(current, target,
                 pos_tol: float = TYPING_ARM_POS_TOL_M,
                 ori_tol: float = TYPING_ARM_ORI_TOL_RAD) -> bool:
    """Return True when a measured pose is within the typing tolerances.

    ``current`` and ``target`` only need ``x``, ``y``, ``z``, ``roll``,
    ``pitch``, and ``yaw`` attributes; both ``ArmPose`` and :class:`KeyTarget`
    satisfy that.
    """
    distance = math.sqrt(
        (float(current.x) - float(target.x)) ** 2
        + (float(current.y) - float(target.y)) ** 2
        + (float(current.z) - float(target.z)) ** 2
    )
    if distance > float(pos_tol):
        return False
    current_rpy = (current.roll, current.pitch, current.yaw)
    target_rpy = (target.roll, target.pitch, target.yaw)
    return orientation_error(current_rpy, target_rpy) <= float(ori_tol)
