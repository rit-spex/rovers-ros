"""Tests for the BaseLimiter hierarchy and limit configurations."""

import math
import sys


import numpy as np

from arm_route_planner.kinematics.limiters import (
    BaseLimiter, FullRangeMode, PlanarMode, HomingMode,
    PickAndPlaceMode, PickAndPlaceFlippedMode,
    FULL_RANGE_MODE, PLANAR_MODE, HOMING_MODE,
    PICK_AND_PLACE_MODE, PICK_AND_PLACE_FLIPPED_MODE, LIMIT_MODES,
    arm_constraints, _get_octant,
)
from arm_route_planner.kinematics.path_planner import (
    build_arm_solver_object, extract_key_joints,
)
from arm_route_planner.kinematics.slew import SimulatedMechanicalArm
from arm_route_planner.kinematics.linkage_models import (
    OrientationMatrix, limit_respecting_diff, wrap2pi,
)
from arm_constants import (
    HOMING_TARGET, HOMING_THRESHOLD, BASE_LIMIT, ELBOW_DEADBAND,
    MECHANICAL_LIMITS,
)

PASS = 0
FAIL = 0


def check(label, condition):
    global PASS, FAIL
    if condition:
        PASS += 1
        print(f"  PASS  {label}")
    else:
        FAIL += 1
        print(f"  FAIL  {label}")


def make_frame(pos):
    return OrientationMatrix([
        [1, 0, 0, pos[0]],
        [0, 1, 0, pos[1]],
        [0, 0, 1, pos[2]],
        [0, 0, 0, 1],
    ])


# =============================================================================
# Hierarchy structure
# =============================================================================

def test_hierarchy():
    print("\n--- Limiter hierarchy structure ---")
    check("BaseLimiter is a class", isinstance(BaseLimiter, type))
    check("FullRangeMode subclasses BaseLimiter", issubclass(FullRangeMode, BaseLimiter))
    check("PlanarMode subclasses BaseLimiter", issubclass(PlanarMode, BaseLimiter))
    check("PickAndPlaceMode subclasses BaseLimiter", issubclass(PickAndPlaceMode, BaseLimiter))
    check("PickAndPlaceFlippedMode subclasses BaseLimiter", issubclass(PickAndPlaceFlippedMode, BaseLimiter))
    check("HomingMode subclasses BaseLimiter", issubclass(HomingMode, BaseLimiter))
    check("instances are BaseLimiter", isinstance(FULL_RANGE_MODE, BaseLimiter)
          and isinstance(PLANAR_MODE, BaseLimiter) and isinstance(PICK_AND_PLACE_MODE, BaseLimiter)
          and isinstance(PICK_AND_PLACE_FLIPPED_MODE, BaseLimiter)
          and isinstance(HOMING_MODE, BaseLimiter))
    check("LIMIT_MODES has 1 entry", len(LIMIT_MODES) == 1)
    check("LIMIT_MODES[0] is PickAndPlace", LIMIT_MODES[0] is PICK_AND_PLACE_MODE)
    check("FullRange name", FULL_RANGE_MODE.name == "Full Range")
    check("Planar name", PLANAR_MODE.name == "Planar")
    check("PickAndPlace name", PICK_AND_PLACE_MODE.name == "Pick and Place")
    check("PickAndPlaceFlipped name", PICK_AND_PLACE_FLIPPED_MODE.name == "Pick and Place Flipped")
    check("Planar name", PLANAR_MODE.name == "Planar")
    check("Homing name", HOMING_MODE.name == "Homing")
    check("all modes have 5 joint limits",
          all(len(m.limits) == 5 for m in (FULL_RANGE_MODE, PLANAR_MODE, HOMING_MODE)))


# =============================================================================
# can_set
# =============================================================================

def test_can_set():
    print("\n--- can_set ---")
    check("FullRange always allowed", FULL_RANGE_MODE.can_set([0, 0, 0, 0, 0]))
    check("FullRange allowed at odd angles", FULL_RANGE_MODE.can_set([1.0, 2.0, -1.0, 0.5, 0.5]))
    check("Homing always allowed", HOMING_MODE.can_set([0, 0, 0, 0, 0]))

    check("Planar allowed at base 0", PLANAR_MODE.can_set([0, 0.5, -0.5, 0, 0]))
    check("Planar allowed at base +0.75deg", PLANAR_MODE.can_set([math.radians(0.75), 0.5, -0.5, 0, 0]))
    check("Planar allowed at base -0.75deg", PLANAR_MODE.can_set([math.radians(-0.75), 0.5, -0.5, 0, 0]))
    check("Planar rejected at base +1.5deg", not PLANAR_MODE.can_set([math.radians(1.5), 0.5, -0.5, 0, 0]))
    check("Planar rejected at base -1.5deg", not PLANAR_MODE.can_set([math.radians(-1.5), 0.5, -0.5, 0, 0]))
    check("Planar rejected at base +4deg", not PLANAR_MODE.can_set([math.radians(4), 0.5, -0.5, 0, 0]))
    check("Planar rejected at base -4deg", not PLANAR_MODE.can_set([math.radians(-4), 0.5, -0.5, 0, 0]))
    check("Planar rejected at base +10deg", not PLANAR_MODE.can_set([math.radians(10), 0.5, -0.5, 0, 0]))
    check("Planar rejected at base -10deg", not PLANAR_MODE.can_set([math.radians(-10), 0.5, -0.5, 0, 0]))

    check("PickAndPlace always allowed", PICK_AND_PLACE_MODE.can_set([0, 0.5, -0.5, 0, 0]))
    check("PickAndPlace allowed at odd angles", PICK_AND_PLACE_MODE.can_set([1.0, 0.5, -1.0, 0.5, 0.5]))


# =============================================================================
# PickAndPlace limits
# =============================================================================

def test_pick_and_place_limits():
    print("\n--- Pick and Place limits ---")
    lim = PICK_AND_PLACE_MODE.limits

    # Base: mechanical
    check("P&P base is mechanical", abs(lim[0][0] - (-math.pi * 3/4)) < 1e-12
          and abs(lim[0][1] - (math.pi * 3/4)) < 1e-12)

    # Shoulder: 0 to 90 deg
    check("P&P shoulder lower is 0", abs(lim[1][0]) < 1e-12)
    check("P&P shoulder upper is 90deg", abs(math.degrees(lim[1][1]) - 90) < 1e-6)

    # Elbow: -135 to 0 deg
    check("P&P elbow lower is -135deg", abs(math.degrees(lim[2][0]) + 135) < 1e-6)
    check("P&P elbow upper is 0", abs(lim[2][1]) < 1e-12)

    # Wrist bend: mechanical
    check("P&P wrist bend is mechanical", abs(lim[3][0] - (-math.pi/6)) < 1e-12
          and abs(lim[3][1] - (math.pi/6)) < 1e-12)

    # Wrist twist: mechanical
    check("P&P wrist twist is mechanical", lim[4][0] == MECHANICAL_LIMITS[4][0]
          and lim[4][1] == MECHANICAL_LIMITS[4][1])

    # clamp_angles behaviour
    out = PICK_AND_PLACE_MODE.clamp_angles([0.0, math.radians(120), math.radians(-60), 0, 0], None)
    check("P&P clamps shoulder to 90deg", abs(math.degrees(out[1]) - 90) < 1e-6)
    check("P&P allows negative elbow", abs(math.degrees(out[2]) + 60) < 1e-6)

    out = PICK_AND_PLACE_MODE.clamp_angles([0.0, 0.5, math.radians(30), 0, 0], None)
    check("P&P clamps positive elbow to 0", abs(out[2]) < 1e-9)

    out = PICK_AND_PLACE_MODE.clamp_angles([math.radians(200), 0.5, -0.5, 0, 0], None)
    check("P&P clamps base to 135deg", abs(math.degrees(out[0]) - 135) < 1e-6)

    # The flipped side is a separate modifier, not part of normal P&P
    check("P&P normal rejects flipped side",
          not PICK_AND_PLACE_MODE.can_set(
              [0, math.radians(120), math.radians(45), 0, 0]))


# =============================================================================
# PickAndPlaceFlipped limits + FlipSideController
# =============================================================================

def test_pick_and_place_flipped():
    print("\n--- Pick and Place Flipped limits ---")
    lim = PICK_AND_PLACE_FLIPPED_MODE.limits

    check("P&P flipped base is mechanical", abs(lim[0][0] - (-math.pi * 3/4)) < 1e-12
          and abs(lim[0][1] - (math.pi * 3/4)) < 1e-12)
    check("P&P flipped shoulder lower is 90deg", abs(math.degrees(lim[1][0]) - 90) < 1e-6)
    check("P&P flipped shoulder upper is 180deg", abs(math.degrees(lim[1][1]) - 180) < 1e-6)
    check("P&P flipped elbow lower is 0", abs(lim[2][0]) < 1e-12)
    check("P&P flipped elbow upper is 135deg", abs(math.degrees(lim[2][1]) - 135) < 1e-6)
    check("P&P flipped wrist bend is mechanical", abs(lim[3][0] - (-math.pi/6)) < 1e-12)
    check("P&P flipped wrist twist is mechanical", lim[4][0] == MECHANICAL_LIMITS[4][0]
          and lim[4][1] == MECHANICAL_LIMITS[4][1])

    # can_set
    check("P&P flipped allowed at shoulder=120 elbow=45",
          PICK_AND_PLACE_FLIPPED_MODE.can_set([0, math.radians(120), math.radians(45), 0, 0]))
    check("P&P flipped rejected at shoulder=30 elbow=-45",
          not PICK_AND_PLACE_FLIPPED_MODE.can_set([0, math.radians(30), math.radians(-45), 0, 0]))

    # clamp_angles
    out = PICK_AND_PLACE_FLIPPED_MODE.clamp_angles([0.0, math.radians(30), math.radians(-45), 0, 0], None)
    check("P&P flipped clamps shoulder to 90deg", abs(math.degrees(out[1]) - 90) < 1e-6)
    check("P&P flipped clamps elbow to 0", abs(out[2]) < 1e-9)

    out = PICK_AND_PLACE_FLIPPED_MODE.clamp_angles([0.0, math.radians(120), math.radians(45), 0, 0], None)
    check("P&P flipped allows in-range angles",
          abs(math.degrees(out[1]) - 120) < 1e-6 and abs(math.degrees(out[2]) - 45) < 1e-6)


# =============================================================================
# get_limits — shoulder expansion
# =============================================================================

def test_shoulder_expansion():
    print("\n--- Shoulder expansion (FullRange) ---")
    limits = FULL_RANGE_MODE.get_limits([0.0, 0.5, -0.5, 0, 0], None)
    check("base=0 expands shoulder to pi", abs(limits[1][1] - math.pi) < 1e-9)

    limits = FULL_RANGE_MODE.get_limits(
        [math.radians(1.5), 0.5, -0.5, 0, 0], None)
    check("base=1.5deg expands shoulder to pi",
          abs(limits[1][1] - math.pi) < 1e-9)

    limits = FULL_RANGE_MODE.get_limits(
        [math.radians(4), 0.5, -0.5, 0, 0], None)
    check("base=4deg keeps shoulder at 178deg",
          abs(math.degrees(limits[1][1]) - 178) < 1e-6)

    limits = FULL_RANGE_MODE.get_limits([math.radians(20), 0.5, -0.5, 0, 0], None)
    check("base=20deg keeps shoulder at 178deg",
          abs(math.degrees(limits[1][1]) - 178) < 1e-6)

    limits = HOMING_MODE.get_limits([0.0, 0.5, -0.5, 0, 0], None)
    check("HomingMode does NOT expand shoulder",
          abs(math.degrees(limits[1][1]) - 178) < 1e-6)


# =============================================================================
# get_limits — octant relief / curl
# =============================================================================

def test_octant_limits():
    print("\n--- Octant relief and curl ---")
    # Octant 3 = Back High: -Y dominates, pz >= 0
    back_high = [5.0, -30.0, 10.0]
    check("_get_octant back-high is 3", _get_octant(back_high) == 3)

    # Mild overshoot on elbow (index 2): 10 deg past 135 deg limit
    angles = [0.0, 0.5, math.radians(145), 0, 0]
    limits = FULL_RANGE_MODE.get_limits(angles, back_high)
    check("octant 3 relieves mild elbow overshoot",
          abs(math.degrees(limits[2][1]) - 145) < 1e-6)

    # Large overshoot: 40 deg past limit — exceeds O3_RELAX_MAX, NOT excused
    angles = [0.0, 0.5, math.radians(175), 0, 0]
    limits = FULL_RANGE_MODE.get_limits(angles, back_high)
    check("octant 3 does NOT excuse large overshoot",
          abs(math.degrees(limits[2][1]) - 135) < 1e-6)

    # Octant 6 = Right Low: +X dominates, pz < 0
    right_low = [30.0, 5.0, -10.0]
    check("_get_octant right-low is 6", _get_octant(right_low) == 6)

    angles = [0.0, 0.5, 0.5, 0, 0]
    limits = FULL_RANGE_MODE.get_limits(angles, right_low)
    check("octant 6 curls elbow to <= 0", limits[2][1] == 0.0)
    check("octant 6 keeps elbow lower bound", abs(limits[2][0] - BASE_LIMIT[0]) < 1e-9
          or abs(math.degrees(limits[2][0]) + 135) < 1e-6)

    # Front High — no relief, no curl
    front_high = [5.0, 30.0, 10.0]
    check("_get_octant front-high is 1", _get_octant(front_high) == 1)
    angles = [0.0, 0.5, math.radians(145), 0, 0]
    limits = FULL_RANGE_MODE.get_limits(angles, front_high)
    check("octant 1 does NOT relieve overshoot",
          abs(math.degrees(limits[2][1]) - 135) < 1e-6)


# =============================================================================
# check_cylindrical
# =============================================================================

def test_cylindrical():
    print("\n--- Cylindrical avoidance ---")
    check("FullRange flags near-center", FULL_RANGE_MODE.check_cylindrical([3.0, 3.0, 10.0]))
    check("FullRange allows far target", not FULL_RANGE_MODE.check_cylindrical([20.0, 20.0, 10.0]))
    check("Planar never checks cylindrical", not PLANAR_MODE.check_cylindrical([3.0, 3.0, 10.0]))
    check("None ee_pos is safe", not FULL_RANGE_MODE.check_cylindrical(None))


# =============================================================================
# clamp_target_position
# =============================================================================

def test_clamp_target():
    print("\n--- clamp_target_position ---")
    # Inside all limits: returned unchanged (same object)
    f = make_frame([10.0, 10.0, 20.0])
    out = FULL_RANGE_MODE.clamp_target_position(f, [0.0, 0.5, -0.5, 0, 0])
    check("in-range target unchanged", out is f)

    # z below the height floor: clamped up to -20
    f = make_frame([10.0, 10.0, -50.0])
    out = FULL_RANGE_MODE.clamp_target_position(f, [0.0, 0.5, -0.5, 0, 0])
    pos = out.position()
    check("low-octant z clamped to -20", abs(pos[2] - (-20.0)) < 1e-9)

    # z above ceiling in a high octant: clamped down to 60
    f = make_frame([10.0, 10.0, 90.0])
    out = FULL_RANGE_MODE.clamp_target_position(f, [0.0, 0.5, -0.5, 0, 0])
    pos = out.position()
    check("high-octant z clamped to 60", abs(pos[2] - 60.0) < 1e-9)

    # Cylindrical keep-out: target at radius 3 with base away from zero
    f = make_frame([3.0, 0.0, 10.0])
    out = FULL_RANGE_MODE.clamp_target_position(f, [math.radians(30), 0.5, -0.5, 0, 0])
    pos = out.position()
    radial = math.sqrt(pos[0] ** 2 + pos[1] ** 2)
    check("cylindrical keep-out pushes to radius 6", abs(radial - 6.0) < 1e-6)

    # Cylindrical keep-out skipped when base near zero
    f = make_frame([3.0, 0.0, 10.0])
    out = FULL_RANGE_MODE.clamp_target_position(f, [0.0, 0.5, -0.5, 0, 0])
    pos = out.position()
    check("keep-out skipped when base near zero", abs(pos[0] - 3.0) < 1e-9)

    # Planar mode: no XY workspace clamping, but the height floor applies
    f = make_frame([200.0, 200.0, 200.0])
    out = PLANAR_MODE.clamp_target_position(f, [0.0, 0.5, -0.5, 0, 0])
    check("Planar does not clamp XY target", out is f)

    f = make_frame([10.0, 10.0, -50.0])
    out = PLANAR_MODE.clamp_target_position(f, [0.0, 0.5, -0.5, 0, 0])
    check("Planar z clamped to -20",
          abs(out.position()[2] - (-20.0)) < 1e-9)


# =============================================================================
# clamp_angles
# =============================================================================

def test_clamp_angles():
    print("\n--- clamp_angles ---")
    out = FULL_RANGE_MODE.clamp_angles([0.0, 0.5, -0.5, 0, 0], None)
    check("in-range angles unchanged", all(abs(a - b) < 1e-12 for a, b in
          zip(out, [0.0, 0.5, -0.5, 0, 0])))

    out = FULL_RANGE_MODE.clamp_angles([math.radians(200), 0.5, -0.5, 0, 0], None)
    check("base clamped to 135deg", abs(math.degrees(out[0]) - 135) < 1e-6)

    out = PLANAR_MODE.clamp_angles([math.radians(30), 0.5, -0.5, 0, 0], None)
    check("Planar clamps base to +1deg", abs(math.degrees(out[0]) - 1) < 1e-6)

    out = HOMING_MODE.clamp_angles([math.radians(200), 0.5, -0.5, 0, 0], None)
    check("Homing clamps base to 135deg", abs(math.degrees(out[0]) - 135) < 1e-6)


# =============================================================================
# geometry_constraint
# =============================================================================

def test_geometry_constraint():
    print("\n--- geometry_constraint ---")
    out = FULL_RANGE_MODE.geometry_constraint([0.0, 0.5, 0.05, 0, 0])
    check("positive deadband pushes elbow out", out[2] >= ELBOW_DEADBAND - 1e-12)

    out = FULL_RANGE_MODE.geometry_constraint([0.0, 0.5, -0.05, 0, 0])
    check("negative deadband pushes elbow out", out[2] <= -ELBOW_DEADBAND + 1e-12)

    out = FULL_RANGE_MODE.geometry_constraint([0.0, 0.5, 0.5, 0, 0])
    check("elbow outside deadband untouched", abs(out[2] - 0.5) < 1e-12)

    out = PLANAR_MODE.geometry_constraint([0.0, 0.5, 0.05, 0, 0])
    check("Planar applies same deadband", out[2] >= ELBOW_DEADBAND - 1e-12)

    out = HOMING_MODE.geometry_constraint([0.0, 0.5, -0.05, 0, 0])
    check("Homing applies same deadband", out[2] <= -ELBOW_DEADBAND + 1e-12)

    out = arm_constraints([0.0, 0.5, 0.05, 0, 0])
    check("arm_constraints matches geometry_constraint",
          abs(out[2] - FULL_RANGE_MODE.geometry_constraint([0.0, 0.5, 0.05, 0, 0])[2]) < 1e-12)


# =============================================================================
# Limit-blocked shortest path must commit to the long way around
# =============================================================================

def test_limit_blocked_path_does_not_chatter():
    print("\n--- Limit-blocked path commits direction ---")
    lo, hi = MECHANICAL_LIMITS[2]

    # The elbow at +135 deg cannot take the shortest +95 deg path to -130
    # deg because it would cross the +135 deg stop.  It must take the long
    # way down instead of alternating on the bound.
    diff = limit_respecting_diff(math.radians(134.9), wrap2pi(
        math.radians(-130) - math.radians(134.9)), lo, hi)
    check("helper chooses the in-limit long way", diff < 0)
    check("helper path lands on the target",
          abs(math.radians(134.9) + diff - math.radians(-130)) < 1e-9)

    arm = SimulatedMechanicalArm(
        [0, math.radians(35), math.radians(134.9), 0, 0])
    target = [0, math.radians(35), math.radians(-130), 0, 0]
    trace = []
    for _ in range(400):
        arm.step(target, MECHANICAL_LIMITS)
        trace.append(arm.angles[2])
        if abs(wrap2pi(arm.angles[2] - target[2])) < 1e-6:
            break

    check(f"mechanical arm crosses the limit block "
          f"(elbow {math.degrees(arm.angles[2]):.1f} deg)",
          abs(wrap2pi(arm.angles[2] - target[2])) < 1e-6)
    signs = [math.copysign(1, d) for d in np.diff(trace)
             if abs(d) > 1e-9]
    reversals = sum(1 for a, b in zip(signs, signs[1:]) if a != b)
    check(f"mechanical arm does not chatter on the bound "
          f"({reversals} reversals)", reversals == 0)


# =============================================================================
# _get_octant boundary
# =============================================================================

def test_octant_boundary():
    print("\n--- _get_octant z-boundary ---")
    check("z=0 is high (octant 1-4)", _get_octant([0.0, 10.0, 0.0]) == 1)
    check("z=+eps is high", _get_octant([0.0, 10.0, 0.01]) == 1)
    check("z=-eps is low", _get_octant([0.0, 10.0, -0.01]) == 5)
    check("z=0 matches high-octant z-range (0,60)",
          FULL_RANGE_MODE.position_limits[
              _get_octant([0.0, 10.0, 0.0])][2] == (0, 60))

    # All 8 octants resolvable (+Y front, +X right, z high/low)
    pts = {
        1: [0, 10, 5], 2: [10, 0, 5], 3: [0, -10, 5], 4: [-10, 0, 5],
        5: [0, 10, -5], 6: [10, 0, -5], 7: [0, -10, -5], 8: [-10, 0, -5],
    }
    for octant, pt in pts.items():
        check(f"octant {octant} resolves", _get_octant(pt) == octant)
    check("all 8 octants have position limits",
          len(FULL_RANGE_MODE.position_limits) == 8)


# =============================================================================
# Solver integration
# =============================================================================

def test_solver_geometry_limiter():
    print("\n--- Solver geometry_limiter ---")
    model, solver = build_arm_solver_object()
    check("solver has geometry_limiter", solver.geometry_limiter is arm_constraints)

    target = OrientationMatrix([
        [1, 0, 0, 20],
        [0, 1, 0, 30],
        [0, 0, 1, 15],
        [0, 0, 0, 1],
    ])
    angles, _, _, _ = solver.solve([0.3, 0.6, -0.6, 0.1, 0.0], target)
    elbow = angles[2]
    check("converged elbow respects deadband",
          abs(elbow) >= ELBOW_DEADBAND - 1e-6 or abs(elbow) > 1.0)


def test_extract_key_joints():
    print("\n--- extract_key_joints ---")
    model, _ = build_arm_solver_object()
    positions = model.joint_positions([0.1, 0.2, -0.3, 0.1, 0.0], frame=False)
    frames = model.joint_positions([0.1, 0.2, -0.3, 0.1, 0.0], frame=True)
    pos, frm = extract_key_joints(positions, frames)
    check("returns 4 positions", len(pos) == 4)
    check("returns 4 frames", len(frm) == 4)
    check("position matches source", np.allclose(pos[0], positions[1]))
    check("frame matches source", frm[3] is frames[8])


# =============================================================================
# Run
# =============================================================================

if __name__ == "__main__":
    test_hierarchy()
    test_can_set()
    test_pick_and_place_limits()
    test_pick_and_place_flipped()
    test_shoulder_expansion()
    test_octant_limits()
    test_cylindrical()
    test_clamp_target()
    test_clamp_angles()
    test_geometry_constraint()
    test_limit_blocked_path_does_not_chatter()
    test_octant_boundary()
    test_solver_geometry_limiter()
    test_extract_key_joints()

    print(f"\n{'=' * 44}")
    print(f"Results: {PASS} passed, {FAIL} failed")
    if FAIL:
        sys.exit(1)
    print("All limiter tests passed!")
