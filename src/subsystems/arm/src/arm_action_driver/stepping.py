"""Shared positional stepping profile for the arm action driver.

One trapezoidal velocity profile serves homing, flip, zenith, and resting
moves.  The profile is positional: every cycle it is recomputed from the
remaining distance and the joints' current velocity state, so pause,
resume, and E-stop freeze all work without a precomputed trajectory.

Discrete-time braking uses the standard clamp

    v_brake = sqrt(2*a*d + (a*dt/2)^2) - a*dt/2

and every step is clamped to the remaining distance, so the joints never
overshoot the goal at the 20 Hz control rate.
"""

import math

_EPS = 1e-12


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


class JointRamp:
    """Rate-limited, synchronized stepping for a fixed number of joints."""

    def __init__(self, count, accel, decel, dt, stop_tol=1e-4):
        self.count = int(count)
        self.accel = max(1e-9, float(accel))
        self.decel = max(1e-9, float(decel))
        self.dt = float(dt)
        self.stop_tol = float(stop_tol)
        self.velocity = [0.0] * self.count

    def reset(self):
        """Drop the velocity state (start of a new move)."""
        self.velocity = [0.0] * self.count

    def stop(self):
        """Begin a controlled stop; no new cruise velocity is requested."""
        return [0.0] * self.count

    def step(self, deltas, caps, brake=True):
        """Advance one cycle.

        Args:
            deltas: Signed remaining angle per joint [rad].  The caller has
                already chosen the direction (including limit-respecting
                long ways around).
            caps: Per-joint cruise speed cap [rad/s].  A cap of zero holds
                that joint and decelerates it to rest.
            brake: When False the joint cruises straight through a
                waypoint instead of braking for it.  The step is still
                clamped to the remaining distance, so the waypoint itself
                is never overshot.

        Returns:
            (steps, done): per-joint angle step for this cycle [rad], and
            True when every joint is within ``stop_tol`` of its goal and
            at rest.
        """
        steps = [0.0] * self.count
        done = True
        for i in range(self.count):
            d = float(deltas[i]) if i < len(deltas) else 0.0
            cap = max(0.0, float(caps[i])) if i < len(caps) else 0.0
            v = self.velocity[i]

            if abs(d) <= self.stop_tol:
                steps[i] = d
                self.velocity[i] = 0.0
                continue
            done = False

            # Requested speed: cruise cap, limited by the braking distance
            # on the final leg only.
            if brake:
                v_cruise = min(cap, abs(d) / self.dt)
                v_brake = math.sqrt(
                    2.0 * self.decel * abs(d)
                    + (self.decel * self.dt / 2.0) ** 2
                ) - self.decel * self.dt / 2.0
                target = max(0.0, min(v_cruise, v_brake))
            else:
                target = cap
            target = math.copysign(target, d)

            # Ramp toward the request: accelerate when speeding up,
            # decelerate otherwise.  A direction change decays through
            # zero before the joint turns around.
            if v * target < 0.0:
                rate = self.decel
                magnitude = min(abs(v), rate * self.dt)
                v = math.copysign(abs(v) - magnitude, v)
            elif abs(target) >= abs(v):
                rate = self.accel
                magnitude = min(abs(target) - abs(v), rate * self.dt)
                v = math.copysign(abs(v) + magnitude, v if v else target)
            else:
                rate = self.decel
                magnitude = min(abs(v) - abs(target), rate * self.dt)
                v = math.copysign(max(0.0, abs(v) - magnitude),
                                  v if v else target)

            step = v * self.dt
            if abs(step) >= abs(d):
                step = d
                if brake:
                    v = 0.0
                else:
                    # Keep cruising through the waypoint; only the sign
                    # follows the next segment.
                    v = math.copysign(abs(v), d)
                self.velocity[i] = v
            else:
                self.velocity[i] = v
            steps[i] = step

        return steps, done

    def is_stopped(self, eps=1e-6):
        """True when every joint velocity is effectively zero."""
        return all(abs(v) <= eps for v in self.velocity)


class LimitedRamp:
    """Convenience wrapper that clamps stepping to per-joint limits.

    The caller supplies the signed remaining deltas and the active limits;
    the wrapper produces a target angle list and reports arrival.
    """

    def __init__(self, ramp, limits):
        self.ramp = ramp
        self.limits = [tuple(pair) for pair in limits]

    def step(self, angles, deltas, caps, mask=None):
        steps, done = self.ramp.step(deltas, caps)
        targets = list(angles)
        for i in range(len(targets)):
            allowed = mask is None or i >= len(mask) or mask[i]
            if not allowed:
                continue
            lo, hi = self.limits[i]
            targets[i] = _clamp(angles[i] + steps[i], lo, hi)
        return targets, done


def remaining_delta(current, goal):
    """Shortest signed angular distance from ``current`` to ``goal``."""
    return (goal - current + math.pi) % (2.0 * math.pi) - math.pi
