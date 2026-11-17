"""Tests for the arm control-ownership policy."""

from arm_master.arm_master_node import select_control_source
from arm_constants import (
    ARM_CONTROL_SOURCE_TOPIC,
    ArmControlSource,
    control_source_label,
    parse_control_source,
)

MANUAL = ArmControlSource.MANUAL
AUTO = ArmControlSource.AUTONOMOUS
NONE = ArmControlSource.NONE


def test_source_topic_is_part_of_the_state_map():
    assert ARM_CONTROL_SOURCE_TOPIC == "/ARM/STATE/CONTROL_SOURCE"


def test_wire_values_round_trip():
    for source in (NONE, MANUAL, AUTO):
        assert parse_control_source(int(source)) is source
        assert control_source_label(source) != ""
    assert parse_control_source(99) is None
    assert parse_control_source("nope") is None
    assert control_source_label(99).startswith("UNKNOWN")


def test_manual_always_preempts_autonomous():
    source, accepted, _ = select_control_source(
        AUTO, MANUAL, planner_alive=True, planner_ready=True, estop=True)
    assert accepted
    assert source is MANUAL


def test_autonomous_needs_a_ready_planner_and_no_estop():
    source, accepted, reason = select_control_source(
        MANUAL, AUTO, planner_alive=True, planner_ready=True, estop=False)
    assert accepted
    assert source is AUTO
    assert reason == "autonomous control granted"

    for alive, ready, estop, expected in (
        (False, True, False, "route planner unavailable"),
        (True, False, False, "arm not ready"),
        (True, True, True, "e-stop active"),
    ):
        source, accepted, reason = select_control_source(
            MANUAL, AUTO, planner_alive=alive,
            planner_ready=ready, estop=estop)
        assert not accepted
        assert source is MANUAL
        assert reason == expected


def test_release_is_always_accepted():
    source, accepted, _ = select_control_source(
        AUTO, NONE, planner_alive=False, planner_ready=False, estop=True)
    assert accepted
    assert source is NONE


def test_unknown_source_is_refused():
    source, accepted, reason = select_control_source(
        AUTO, 99, planner_alive=True, planner_ready=True, estop=False)
    assert not accepted
    assert source is AUTO
    assert "unknown" in reason


def test_repeated_request_is_a_no_op():
    source, accepted, _ = select_control_source(
        MANUAL, MANUAL, planner_alive=False, planner_ready=False, estop=True)
    assert accepted
    assert source is MANUAL
