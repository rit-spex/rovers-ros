"""Tests for the typing geometry and key-list helpers."""

import math

import pytest

from arm_typing.typing_plan import (
    KEY_POSITIONS,
    KEY_SIZES,
    alignment_error,
    build_key_positions,
    drive_power_for_error,
    key_point_in_rover,
    key_target,
    parse_key_sequence,
    plane_frame,
    plane_is_sideways,
    pose_matches,
    rotation_from_rpy,
    rotation_to_rpy,
)
from arm_constants import TYPING_KEY_PITCH_M

# A vertical keyboard on the rover's right: plane +Z points at the rover
# (-X), plane +Y is up (+Z), and plane +X runs backward (-Y).
RIGHT_PLANE = dict(
    x=0.45,
    y=0.0,
    z=0.4,
    roll=math.radians(90.0),
    pitch=0.0,
    yaw=math.radians(-90.0),
)


def test_reference_layout_has_the_usual_keys():
    """The generated layout contains the printable keys."""
    for key in "abcdefghijklmnopqrstuvwxyz0123456789":
        assert key in KEY_POSITIONS
    for key in ("space", "enter", "tab", "backspace", "shift"):
        assert key in KEY_POSITIONS


def test_layout_is_centred_and_unique():
    """The layout is centred and every key name is unique."""
    positions = build_key_positions()
    assert positions["space"] == (0.0, positions["space"][1])
    assert len(set(positions)) == len(positions)
    # Rows are one pitch apart vertically.
    assert positions["q"][1] > positions["a"][1] > positions["z"][1]


def test_layout_sizes_are_in_metres():
    """The size table converts key units through the physical pitch."""
    assert KEY_SIZES["a"] == pytest.approx(
        (TYPING_KEY_PITCH_M, TYPING_KEY_PITCH_M))
    assert KEY_SIZES["space"][0] > KEY_SIZES["a"][0]


def test_parse_literal_text_and_special_tokens():
    """Literal whitespace and braced names map to key names."""
    assert parse_key_sequence("ab c\n") == [
        "a", "b", "space", "c", "enter"]
    assert parse_key_sequence("{tab}{space}{backspace}") == [
        "tab", "space", "backspace"]


def test_parse_rejects_unknown_and_shifted_keys():
    """Unsupported characters fail instead of typing the wrong key."""
    with pytest.raises(ValueError):
        parse_key_sequence("A")
    with pytest.raises(ValueError):
        parse_key_sequence("~")
    with pytest.raises(ValueError):
        parse_key_sequence("{nope}")


def test_plane_side_defaults_from_x():
    """A plane without an explicit side is classified from its x."""
    right = plane_frame(0.45, 0.0, 0.4, math.radians(90), 0.0,
                        math.radians(-90))
    left = plane_frame(-0.45, 0.0, 0.4, math.radians(90), 0.0,
                       math.radians(90))
    assert right.side == -1
    assert left.side == 1
    assert plane_is_sideways(right)
    assert not plane_is_sideways(
        plane_frame(0.02, 0.0, 0.4, 0.0, 0.0, 0.0))


def test_key_target_approaches_the_face():
    """The tool approaches the keyboard along the plane normal."""
    plane = plane_frame(**RIGHT_PLANE)
    target = key_target(plane, "a", standoff=0.06)
    u, v = KEY_POSITIONS["a"]

    assert target.x == pytest.approx(0.45 - 0.06)
    assert target.y == pytest.approx(-u)
    assert target.z == pytest.approx(0.4 + v)

    rotation = rotation_from_rpy(target.roll, target.pitch, target.yaw)
    approach = [rotation[row][1] for row in range(3)]
    assert approach == pytest.approx([1.0, 0.0, 0.0])


def test_alignment_error_is_the_rover_forward_offset():
    """Lateral error is the key offset along the rover forward axis."""
    plane = plane_frame(**RIGHT_PLANE)
    assert alignment_error(plane, "a") == pytest.approx(
        key_point_in_rover(plane, "a")[1])
    assert alignment_error(plane, "a", target_y=0.1) == pytest.approx(
        key_point_in_rover(plane, "a")[1] - 0.1)


def test_drive_power_is_capped_and_deadbanded():
    """Drive power respects the deadband and the 10 percent cap."""
    assert drive_power_for_error(0.001) == 0.0
    assert drive_power_for_error(0.1) == pytest.approx(0.1)
    assert drive_power_for_error(-0.1) == pytest.approx(-0.1)
    assert drive_power_for_error(1.0) == pytest.approx(0.1)
    assert drive_power_for_error(-1.0) == pytest.approx(-0.1)


def test_pose_matching_uses_position_and_orientation():
    """Arrival checks reject a pose outside either tolerance."""
    target = key_target(plane_frame(**RIGHT_PLANE), "a")
    assert pose_matches(target, target)
    assert not pose_matches(
        type(target)(target.x + 0.05, target.y, target.z,
                     target.roll, target.pitch, target.yaw),
        target)


def test_rpy_round_trip():
    """RPY conversion is stable for a non-singular rotation."""
    rotation = rotation_from_rpy(0.2, -0.3, 0.4)
    roll, pitch, yaw = rotation_to_rpy(rotation)
    rebuilt = rotation_from_rpy(roll, pitch, yaw)
    for row in range(3):
        for column in range(3):
            assert rebuilt[row][column] == pytest.approx(
                rotation[row][column])
