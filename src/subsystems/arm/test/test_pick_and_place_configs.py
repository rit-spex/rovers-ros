"""Configuration tests for Pick & Place normal / flipped / planar modes.

Run:
    python tests/test_pick_and_place_configs.py
"""

import math
import random
import sys


from arm_constants import HOMING_TARGET, MECHANICAL_LIMITS
from arm_route_planner.kinematics.simulated_arm import (
    RoverArmController, random_initial_angles,
)
from arm_route_planner.kinematics.limiters import (
    PICK_AND_PLACE_MODE, PICK_AND_PLACE_FLIPPED_MODE, PLANAR_MODE,
    _get_octant,
)
from arm_route_planner.kinematics.generic_frames import wrap2pi

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


def flipped_home():
    return [
        HOMING_TARGET[0],
        math.pi - HOMING_TARGET[1],
        -HOMING_TARGET[2],
        HOMING_TARGET[3],
        HOMING_TARGET[4],
    ]


def set_angles(arm, angles):
    arm.mechanical.angles = list(angles)
    arm.target_angles = list(angles)
    arm.refresh_fk()
    arm._sync_target_to_fk()


def run_to_idle(arm, max_frames=1000):
    for n in range(max_frames):
        arm.update(None)
        if not arm.is_flipping and not arm.is_homing:
            return n + 1
    return None


def close_to(actual, target, tol=0.03):
    return all(abs(wrap2pi(a - b)) <= tol for a, b in zip(actual, target))


def within_limits(angles, limits, tol=1e-6):
    return all(lo - tol <= a <= hi + tol
               for a, (lo, hi) in zip(angles, limits))


def full_range():
    return [tuple(pair) for pair in MECHANICAL_LIMITS]


def unlock_wrists(arm):
    arm.locked_joints[3] = False
    arm.locked_joints[4] = False


def normal_even_pose():
    """A normal-side pose whose end effector is in an even octant."""
    return [-math.radians(130), 0, math.radians(-5), 0, 0]


# One safe start pose per octant.  Restricted octants (3, 5, 7) start in
# the planar band so the action itself is responsible for staying there.
# These poses track the link lengths in arm_constants, so revisit them when
# A0/A1/A2 change: the low octants are close to the z = 0 diagonal.
SAFE_OCTANT_POSES = {
    1: (0, 0, -10, 0, 0),
    2: (-135, 0, 0, 0, 0),
    3: (0, 0, 135, 0, 0),
    4: (50, 45, -75, 0, 0),
    5: (0, 0, -45, 0, 0),
    6: (-90, 0, -100, 0, 0),
    7: (0, 0, -135, 0, 0),
    8: (-120, 0, -135, 0, 0),
}

RESTRICTED = (3, 5, 7)
PLANAR_BAND = math.radians(5)


def octant_start(octant, flipped=False, planar=False):
    arm = RoverArmController(random_initial_angles())
    set_angles(arm, [math.radians(v)
                     for v in SAFE_OCTANT_POSES[octant]])
    arm.flipped = flipped
    arm.planar = planar
    return arm


def run_action_trace(arm, action, max_frames=1500):
    if action == "home":
        arm.toggle_homing()
    elif not arm.activate_flip():
        return None

    trace = []
    for _ in range(max_frames):
        arm.update(None)
        pos = [float(v) for v in
               arm.model.solve_forward_kinematics(arm.angles).position()]
        trace.append((list(arm.angles), pos, _get_octant(pos)))
        if not arm.is_homing and not arm.is_flipping:
            return trace
    return None


def restricted_violation(trace):
    """First frame that is clearly inside a restricted octant.

    Boundary slivers (within 0.5 in of an octant diagonal) are ignored so
    floating-point flicker along a safe path is not reported as a block.
    """
    for angles, pos, octant in trace:
        if octant not in RESTRICTED:
            continue
        if abs(wrap2pi(angles[0])) <= PLANAR_BAND + 1e-6:
            continue
        if abs(pos[2]) <= 0.5:
            continue
        if octant in (3, 7) and -abs(pos[0]) - pos[1] <= 0.5:
            continue
        if octant == 5 and pos[1] - abs(pos[0]) <= 0.5:
            continue
        return angles, pos, octant
    return None


def trace_span(trace):
    xs = [p[0] for _, p, _ in trace]
    ys = [p[1] for _, p, _ in trace]
    zs = [p[2] for _, p, _ in trace]
    return (min(xs), max(xs)), (min(ys), max(ys)), (min(zs), max(zs))


def check_trace(label, trace):
    check(f"{label}: completes", trace is not None)
    if trace is None:
        return
    violation = restricted_violation(trace)
    check(f"{label}: stays out of restricted space",
          violation is None)
    if violation is not None:
        angles, pos, octant = violation
        print("        violation:",
              f"octant {octant}",
              "angles", [round(math.degrees(a), 1) for a in angles],
              "pos", [round(v, 1) for v in pos])

    (x0, x1), (y0, y1), (z0, z1) = trace_span(trace)
    check(f"{label}: X span in workspace",
          max(abs(x0), abs(x1)) <= 60 + 1e-6)
    check(f"{label}: Y span in workspace",
          max(abs(y0), abs(y1)) <= 60 + 1e-6)
    check(f"{label}: Z span in workspace",
          z0 >= -30 - 1e-6 and z1 <= 65 + 1e-6)


# =============================================================================
# Defaults
# =============================================================================

def test_defaults():
    print("\n--- Default configuration ---")
    arm = RoverArmController(random_initial_angles())

    check("starts in normal P&P", not arm.flipped and not arm.planar)
    check("only mode is Pick & Place",
          arm.limit_mode is PICK_AND_PLACE_MODE)
    check("joints 4 and 5 locked",
          arm.locked_joints == [False, False, False, True, True])
    check("starts near normal home",
          max(abs(wrap2pi(a - b))
              for a, b in zip(arm.angles, HOMING_TARGET)) <= 0.2 + 1e-9)
    check("normal limits active",
          arm.effective_limits == [tuple(p) for p in PICK_AND_PLACE_MODE.limits])
    check("actual pose fits normal limits",
          within_limits(arm.angles, PICK_AND_PLACE_MODE.limits))


# =============================================================================
# Normal side
# =============================================================================

def test_normal_home():
    print("\n--- Normal home ---")
    arm = RoverArmController(random_initial_angles())
    unlock_wrists(arm)
    arm.toggle_homing()

    check("homing uses full range", arm.effective_limits == full_range())
    frames = run_to_idle(arm)
    check("normal homing completes", frames is not None)
    check("normal home reached", close_to(arm.angles, HOMING_TARGET))
    check("still normal side", not arm.flipped and not arm.planar)
    check("normal limits return",
          arm.effective_limits == [tuple(p) for p in PICK_AND_PLACE_MODE.limits])
    check("pose fits normal limits",
          within_limits(arm.angles, PICK_AND_PLACE_MODE.limits))


# =============================================================================
# Flipped side
# =============================================================================

def test_flip_to_flipped():
    print("\n--- Flip to flipped side ---")
    arm = RoverArmController(random_initial_angles())
    unlock_wrists(arm)
    set_angles(arm, normal_even_pose())

    check("flip starts", arm.activate_flip())
    check("flip uses full range", arm.effective_limits == full_range())
    frames = run_to_idle(arm)
    check("flip completes", frames is not None)
    check("flipped flag set", arm.flipped)
    check("pose fits flipped limits",
          within_limits(arm.angles, PICK_AND_PLACE_FLIPPED_MODE.limits))
    check("flipped limits active",
          arm.effective_limits
          == [tuple(p) for p in PICK_AND_PLACE_FLIPPED_MODE.limits])

    arm.toggle_homing()
    check("flipped homing uses full range",
          arm.effective_limits == full_range())
    frames = run_to_idle(arm)
    check("flipped homing completes", frames is not None)
    check("flipped home reached", close_to(arm.angles, flipped_home()))
    check("flipped flag stays set", arm.flipped)
    check("flipped limits return after homing",
          arm.effective_limits
          == [tuple(p) for p in PICK_AND_PLACE_FLIPPED_MODE.limits])
    check("pose fits flipped limits after homing",
          within_limits(arm.angles, PICK_AND_PLACE_FLIPPED_MODE.limits))


def test_flip_back_to_normal():
    print("\n--- Flip back to normal ---")
    arm = RoverArmController(random_initial_angles())
    unlock_wrists(arm)
    set_angles(arm, normal_even_pose())
    arm.activate_flip()
    run_to_idle(arm)
    check("first flip lands flipped", arm.flipped)

    arm.activate_flip()
    check("second flip starts", arm.is_flipping)
    frames = run_to_idle(arm)
    check("second flip completes", frames is not None)
    check("back on normal side", not arm.flipped)
    check("pose fits normal limits",
          within_limits(arm.angles, PICK_AND_PLACE_MODE.limits))
    check("normal limits active",
          arm.effective_limits == [tuple(p) for p in PICK_AND_PLACE_MODE.limits])


# =============================================================================
# Planar modifier
# =============================================================================

def test_planar_gating_and_limits():
    print("\n--- Planar gating and limits ---")
    arm = RoverArmController(random_initial_angles())
    set_angles(arm, [math.radians(10), math.radians(45),
                     math.radians(-45), 0, 0])

    check("planar rejected at base 10 deg", not arm.toggle_planar())
    check("planar stays off", not arm.planar)

    set_angles(arm, [math.radians(3), math.radians(45),
                     math.radians(-45), 0, 0])
    check("planar accepted at base 3 deg", arm.toggle_planar())
    check("planar flag on", arm.planar)

    limits = arm.effective_limits
    check("planar base locked to +/-5 deg",
          abs(math.degrees(limits[0][0]) + 5) < 1e-6
          and abs(math.degrees(limits[0][1]) - 5) < 1e-6)
    check("planar shoulder has full vertical travel",
          abs(limits[1][0]) < 1e-12
          and abs(math.degrees(limits[1][1]) - 180) < 1e-6)
    check("planar elbow has full vertical travel",
          abs(math.degrees(limits[2][0]) + 135) < 1e-6
          and abs(math.degrees(limits[2][1]) - 135) < 1e-6)

    for _ in range(40):
        arm.update({"dx": 0.2, "dy": 0.1, "dz": 0.05,
                    "drx": 0.02, "dry": 0.02, "drz": 0.05})
    check("base stays inside planar band",
          abs(math.degrees(arm.angles[0])) <= 5 + 1e-6)
    check("planar limits stay active",
          arm.effective_limits == limits)

    check("planar disables", arm.toggle_planar())
    check("planar flag off", not arm.planar)


def test_planar_flipped_home():
    print("\n--- Planar on flipped side ---")
    arm = RoverArmController(random_initial_angles())
    unlock_wrists(arm)
    set_angles(arm, [math.radians(3), math.radians(120),
                     math.radians(45), 0, 0])
    check("flipped modifier accepts flipped pose", arm.toggle_flipped())
    check("planar accepts base inside band", arm.toggle_planar())
    check("flipped and planar can coexist", arm.flipped and arm.planar)

    arm.toggle_homing()
    check("planar flipped homing uses full range",
          arm.effective_limits == full_range())
    frames = run_to_idle(arm)
    check("planar flipped homing completes", frames is not None)
    check("flipped home reached", close_to(arm.angles, flipped_home()))
    check("planar stays active", arm.planar)
    check("base home inside planar band",
          abs(math.degrees(arm.angles[0])) <= 5 + 1e-6)
    check("planar limits return",
          arm.effective_limits == [tuple(p) for p in PLANAR_MODE.limits])


# =============================================================================
# Limit validity in every configuration
# =============================================================================

def test_planar_octant_exit_lock():
    print("\n--- Planar exit octant lock ---")
    poses = {
        3: (-40, 0, 135, 0, 0),
        5: (-40, 0, -125, 0, 0),
        7: (-40, 0, -135, 0, 0),
        1: (-40, 0, -5, 0, 0),
    }

    for octant, degrees in poses.items():
        arm = RoverArmController(random_initial_angles())
        set_angles(arm, [math.radians(v) for v in degrees])
        actual = _get_octant(
            arm.model.solve_forward_kinematics(arm.angles).position())
        check(f"pose lands in octant {octant}", actual == octant)

        arm.planar = True
        result = arm.toggle_planar()
        if octant in (3, 5, 7):
            check(f"octant {octant} refuses planar exit",
                  not result and arm.planar)
            check(f"octant {octant} refuses Normal selection",
                  not arm.set_pnp_normal() and arm.planar)
            check(f"octant {octant} refuses Flipped selection",
                  not arm.toggle_flipped() and arm.planar)
        else:
            check(f"octant {octant} allows planar exit",
                  result and not arm.planar)


def test_flip_octant_restraints():
    print("\n--- Flip octant restraints ---")
    even_poses = {
        2: (-130, 0, -5, 0, 0),
        4: (-130, 0, 135, 0, 0),
        6: (-130, 0, -125, 0, 0),
        8: (-130, 0, -135, 0, 0),
    }
    odd_poses = {
        1: SAFE_OCTANT_POSES[1],
        3: SAFE_OCTANT_POSES[3],
        5: SAFE_OCTANT_POSES[5],
        7: SAFE_OCTANT_POSES[7],
    }

    for octant, degrees in even_poses.items():
        arm = RoverArmController(random_initial_angles())
        set_angles(arm, [math.radians(v) for v in degrees])
        check(f"pose lands in octant {octant}", arm.ee_octant == octant)
        check(f"octant {octant} flips without planar",
              arm.activate_flip())

    for octant, degrees in odd_poses.items():
        arm = RoverArmController(random_initial_angles())
        set_angles(arm, [math.radians(v) for v in degrees])
        check(f"pose lands in octant {octant}", arm.ee_octant == octant)
        check(f"octant {octant} refuses flip without planar",
              not arm.activate_flip())
        arm.planar = True
        check(f"octant {octant} flips with planar",
              arm.activate_flip())


def test_homing_paths_from_every_octant():
    print("\n--- Homing paths from every octant ---")
    for octant in range(1, 9):
        for flipped in (False, True):
            arm = octant_start(octant, flipped=flipped)
            check(f"home {'flipped' if flipped else 'normal'} "
                  f"octant {octant}: starts in octant {octant}",
                  arm.ee_octant == octant)
            trace = run_action_trace(arm, "home")
            check_trace(
                f"home {'flipped' if flipped else 'normal'} "
                f"octant {octant}", trace)
            expected = flipped_home() if flipped else HOMING_TARGET
            check(f"home {'flipped' if flipped else 'normal'} "
                  f"octant {octant}: reaches target",
                  close_to(arm.angles, expected))


def test_flip_paths_from_every_octant():
    print("\n--- Flip paths from every octant ---")
    for octant in range(1, 9):
        planar = octant in (1, 3, 5, 7)
        arm = octant_start(octant, planar=planar)
        check(f"flip octant {octant}: starts in octant {octant}",
              arm.ee_octant == octant)
        trace = run_action_trace(arm, "flip")
        check_trace(f"flip octant {octant}", trace)


def test_planar_flip_sides():
    print("\n--- Planar flip side transitions ---")

    cases = [
        (1, False, 3, True),
        (5, False, 7, True),
        (3, True, 1, False),
        (7, True, 5, False),
    ]
    for start_octant, start_flipped, mid_octant, mid_flipped in cases:
        arm = octant_start(start_octant, planar=True,
                           flipped=start_flipped)
        check(f"planar flip O{start_octant}: starts in correct side",
              arm.flipped is start_flipped and arm.planar)
        check(f"planar flip O{start_octant}: starts",
              arm.activate_flip())
        frames = run_to_idle(arm)
        check(f"planar flip O{start_octant}: completes",
              frames is not None)
        check(f"planar flip O{start_octant}: stays planar", arm.planar)
        check(f"planar flip O{start_octant}: lands in O{mid_octant}",
              arm.ee_octant == mid_octant)
        check(f"planar flip O{start_octant}: side flag becomes "
              f"{'flipped' if mid_flipped else 'normal'}",
              arm.flipped is mid_flipped)
        check(f"planar flip O{start_octant}: result is allowed",
              not arm._workspace_blocked(arm.angles))

        check(f"planar flip O{start_octant}: flips back",
              arm.activate_flip())
        frames = run_to_idle(arm)
        check(f"planar flip O{start_octant}: return completes",
              frames is not None)
        check(f"planar flip O{start_octant}: returns to O{start_octant}",
              arm.ee_octant == start_octant)
        check(f"planar flip O{start_octant}: original side restored",
              arm.flipped is start_flipped)


def test_planar_flip_at_band_edge():
    print("\n--- Planar flip at the base band edge ---")

    for base_deg in (5.0, -5.0):
        arm = RoverArmController(random_initial_angles())
        set_angles(arm, [math.radians(base_deg), 0, math.radians(-10), 0, 0])
        arm.planar = True
        arm.flipped = False

        label = f"base {base_deg:+.0f} deg"
        check(f"{label}: starts in octant 1", arm.ee_octant == 1)
        check(f"{label}: flip starts", arm.activate_flip())
        frames = run_to_idle(arm)
        check(f"{label}: flip completes", frames is not None)
        check(f"{label}: stays planar", arm.planar)
        check(f"{label}: lands in octant 3", arm.ee_octant == 3)
        check(f"{label}: side becomes flipped", arm.flipped)


def test_planar_flip_from_limit_pose():
    print("\n--- Planar flip from an elbow-limit pose ---")

    # The elbow starts at its +135 deg stop.  The side-flip target is
    # around -130 deg, whose shortest path is blocked by the stop, so the
    # old follower alternated up/down on the boundary and never finished.
    arm = RoverArmController(random_initial_angles())
    set_angles(arm, [math.radians(v) for v in (-5, 10, 135, 0, 0)])
    arm.planar = True
    arm.flipped = True
    check("limit-pose flip starts in octant 3", arm.ee_octant == 3)
    check("limit-pose flip starts", arm.activate_flip())

    trace = []
    for _ in range(1200):
        arm.update(None)
        trace.append(list(arm.angles))
        if not arm.is_flipping:
            break
    completed = not arm.is_flipping
    check("limit-pose flip completes", completed)
    if completed:
        check("limit-pose flip lands in octant 1", arm.ee_octant == 1)
        check("limit-pose flip returns to normal side", not arm.flipped)
        check("limit-pose flip stays planar", arm.planar)

        elbow = [angles[2] for angles in trace]
        steps = [b - a for a, b in zip(elbow, elbow[1:])
                 if abs(b - a) > 1e-9]
        signs = [1 if d > 0 else -1 for d in steps]
        reversals = sum(1 for a, b in zip(signs, signs[1:]) if a != b)
        check(f"limit-pose flip does not chatter on the elbow stop "
              f"({reversals} reversals)", reversals == 0)


def test_mode_octant_gates():
    print("\n--- Mode-specific octant gates ---")

    for flipped in (False, True):
        for octant in range(1, 9):
            arm = octant_start(octant, flipped=flipped)
            blocked = arm._octant_blocked(arm.angles)
            expected = octant in RESTRICTED
            side = "flipped" if flipped else "normal"
            check(f"{side} octant {octant} "
                  f"{'blocked' if expected else 'allowed'}",
                  blocked == expected)
            if blocked:
                message = (arm.take_failure() or "").lower()
                check(f"{side} octant {octant} explains planar requirement",
                      "requires planar" in message)

    for flipped in (False, True):
        allowed = (3, 7) if flipped else (1, 5)
        side = "flipped planar" if flipped else "planar"
        for octant in range(1, 9):
            arm = octant_start(octant, planar=True, flipped=flipped)
            blocked = arm._octant_blocked(arm.angles)
            expected = octant not in allowed
            check(f"{side} octant {octant} "
                  f"{'blocked' if expected else 'allowed'}",
                  blocked == expected)
            if blocked:
                message = (arm.take_failure() or "").lower()
                expected_text = "3, 7" if flipped else "1, 5"
                check(f"{side} octant {octant} explains allowed set",
                      expected_text in message)

    arm = RoverArmController(random_initial_angles())
    set_angles(arm, [0, 0, math.radians(-10), 0, 0])
    check("normal can start in octant 1", arm.ee_octant == 1)
    check("normal refuses octant 5 command",
          not arm.command_joint_angle(2, math.radians(-45)))
    check("normal refusal names octant 5",
          "octant 5" in (arm.take_failure() or "").lower())

    arm = RoverArmController(random_initial_angles())
    set_angles(arm, [0, 0, math.radians(-10), 0, 0])
    arm.planar = True
    check("planar accepts octant 5 command",
          arm.command_joint_angle(2, math.radians(-45)))
    for _ in range(500):
        arm.update(None)
    check("planar command lands in octant 5", arm.ee_octant == 5)


def test_height_bound():
    print("\n--- End-effector height bound ---")

    arm = octant_start(6)
    z = float(arm.model.solve_forward_kinematics(arm.angles).position()[2])
    check("test pose is below -20 in", z < -20)
    check("height gate blocks low pose",
          arm._workspace_blocked(arm.angles))
    check("height refusal names limit",
          "height limit" in (arm.take_failure() or "").lower())

    arm = octant_start(1)
    check("height gate allows pose above floor",
          not arm._workspace_blocked(arm.angles))


def test_configuration_limits():
    print("\n--- Per-configuration limit validity ---")

    configs = [
        ("normal", False, False, PICK_AND_PLACE_MODE.limits),
        ("flipped", True, False, PICK_AND_PLACE_FLIPPED_MODE.limits),
        ("planar", False, True, PLANAR_MODE.limits),
    ]

    for name, flipped, planar, limits in configs:
        arm = RoverArmController(random_initial_angles())
        unlock_wrists(arm)
        if planar:
            set_angles(arm, [0, math.radians(45), math.radians(-45), 0, 0])
            arm.flipped = flipped
            arm.planar = True
        elif flipped:
            set_angles(arm, normal_even_pose())
            arm.activate_flip()
            run_to_idle(arm)
        else:
            arm.flipped = False
            arm.planar = False

        check(f"{name}: pose inside its limits",
              within_limits(arm.angles, limits))
        check(f"{name}: effective limits match",
              arm.effective_limits == [tuple(p) for p in limits])

        for _ in range(15):
            arm.update({"dx": 0.1, "dy": 0.08, "dz": 0.03,
                        "drx": 0.02, "dry": 0.015, "drz": 0.02})
        check(f"{name}: pose stays inside limits after motion",
              within_limits(arm.angles, limits, tol=0.02))


def test_noisy_feedback_actions():
    """Homing and flips finish even when feedback never lands exactly.

    The ROS node syncs measured motor angles every cycle.  Encoder noise
    and tracking lag mean a waypoint is never hit within 1e-9 rad, which
    used to stall the path follower forever.
    """
    print("\n--- Actions with noisy feedback ---")

    def run_with_noise(arm, action, max_frames=1500, noise=1e-3):
        rng = random.Random(7)
        if action == "home":
            arm.toggle_homing()
        elif not arm.activate_flip():
            return None
        for frames in range(max_frames):
            measured = [a + rng.uniform(-noise, noise) for a in arm.angles]
            arm.sync_angles(measured, reset_target=False)
            arm.update(None)
            if not arm.is_homing and not arm.is_flipping:
                return frames + 1
        return None

    home_arm = octant_start(1)
    unlock_wrists(home_arm)
    check("noisy feedback: homing completes",
          run_with_noise(home_arm, "home") is not None)
    check("noisy feedback: homing reaches home",
          close_to(home_arm.angles, HOMING_TARGET, tol=0.05))

    flip_arm = octant_start(2)
    unlock_wrists(flip_arm)
    flip = run_with_noise(flip_arm, "flip")
    check("noisy feedback: flip completes", flip is not None)
    check("noisy feedback: flip stays on a valid side",
          flip_arm.flipped or
          PICK_AND_PLACE_MODE.can_set(flip_arm.angles))


# =============================================================================
# Entry point
# =============================================================================

def main():
    test_defaults()
    test_normal_home()
    test_flip_to_flipped()
    test_flip_back_to_normal()
    test_planar_gating_and_limits()
    test_planar_flipped_home()
    test_planar_octant_exit_lock()
    test_flip_octant_restraints()
    test_homing_paths_from_every_octant()
    test_flip_paths_from_every_octant()
    test_planar_flip_sides()
    test_planar_flip_at_band_edge()
    test_planar_flip_from_limit_pose()
    test_mode_octant_gates()
    test_height_bound()
    test_configuration_limits()
    test_noisy_feedback_actions()

    print("\n========================================")
    print(f"Results: {PASS} passed, {FAIL} failed")
    if FAIL:
        print("Some configuration tests failed.")
        sys.exit(1)
    print("All configuration tests passed!")


if __name__ == "__main__":
    main()
