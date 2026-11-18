"""Tests for the teleop input shaping rules.

Run either:

    pytest test/test_teleop_inputs.py
    PYTHONPATH=.:src:$PYTHONPATH python3 test/test_teleop_inputs.py
"""

import math

from arm_teleop.arm_teleop_node import (
    AxisShaper,
    clamp_vector_norm,
    is_stale,
    normalize,
)


def close(a, b, tol=1e-9):
    return math.isclose(a, b, rel_tol=0.0, abs_tol=tol)


def test_normalize():
    assert close(normalize(32767, 127), 1.0)
    assert close(normalize(-32767, 127), -1.0)
    assert close(normalize(127, 127), 1.0)
    assert close(normalize(5, 0), 0.0)


def test_deadzone_and_hysteresis():
    shaper = AxisShaper(deadzone=0.05, exponent=1.0, hysteresis=0.02)
    assert close(shaper.process(0.0), 0.0)
    assert close(shaper.process(0.04), 0.0)
    assert close(shaper.process(0.06), 0.0)
    assert not shaper.active

    enter = shaper.process(0.08)
    assert enter > 0.0
    assert shaper.active

    hold = shaper.process(0.06)
    assert hold > 0.0
    assert shaper.active

    drop = shaper.process(0.04)
    assert close(drop, 0.0)
    assert not shaper.active

    shaper.process(0.08)
    shaper.reset()
    assert not shaper.active


def test_curve():
    shaper = AxisShaper(deadzone=0.05, exponent=2.0, hysteresis=0.0)
    assert close(shaper.process(1.0), 1.0)

    shaper.reset()
    half = shaper.process(0.5)
    expected = ((0.5 - 0.05) / 0.95) ** 2
    assert close(half, expected)
    assert close(shaper.process(-0.5), -half)


def test_vector_norm():
    assert close(clamp_vector_norm([1.0, 0.0, 0.0])[0], 1.0)
    assert clamp_vector_norm([0.0, 0.0, 0.0]) == (0.0, 0.0, 0.0)

    diagonal = clamp_vector_norm([1.0, 1.0, 0.0])
    assert close(math.hypot(diagonal[0], diagonal[1]), 1.0)
    assert close(diagonal[0], diagonal[1])

    small = clamp_vector_norm([0.3, 0.4, 0.0])
    assert close(math.hypot(small[0], small[1]), 0.5)


def test_staleness():
    assert is_stale(None, 10.0, 0.5)
    assert not is_stale(9.8, 10.0, 0.5)
    assert is_stale(9.4, 10.0, 0.5)
    assert not is_stale(None, 10.0, 0.0)


def main():
    tests = (
        test_normalize,
        test_deadzone_and_hysteresis,
        test_curve,
        test_vector_norm,
        test_staleness,
    )
    for test in tests:
        test()
        print(f"PASS  {test.__name__}")
    print(f"\nResults: {len(tests)} passed, 0 failed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
