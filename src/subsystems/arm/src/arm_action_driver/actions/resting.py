"""RESTING collapse / de-torque / restore action controller.

Pure library: no ROS imports.  The action driver feeds measured angles and
planned-path waypoints; the controller returns the joint target to
slow-step toward, a per-cycle step cap, and phase-transition events.

Phases:

    IDLE -> CROUCH -> PUSH_SHOULDER -> BACKOFF_SHOULDER
         -> PUSH_ELBOW -> BACKOFF_ELBOW -> COLLAPSED (torque off)
    COLLAPSED / HOLD --R--> TORQUE_ON -> MOVE_OUT -> AUTO_HOME -> DONE

The stall windowing runs inside each push phase (the plan's WAIT_* phases are
the same observation): after a start grace, a joint that has not advanced by
``RESTING_STALL_MOVE_EPS`` over ``RESTING_STALL_WINDOW_SEC`` is declared
stalled and backed off ``RESTING_BACKOFF_ANGLE``.
"""

from arm_constants import (
    DT,
    RESTING_BACKOFF_ANGLE,
    RESTING_CROUCH_TARGET,
    RESTING_ELBOW_PUSH,
    RESTING_LIMITS,
    RESTING_MOVE_ANG_VEL,
    RESTING_MOVE_TIMEOUT_SEC,
    RESTING_PUSH_ARRIVE_TOL,
    RESTING_PUSH_RATE,
    RESTING_PUSH_TIMEOUT_SEC,
    RESTING_SHOULDER_PUSH,
    RESTING_STALL_MOVE_EPS,
    RESTING_STALL_WINDOW_SEC,
    RESTING_START_GRACE_SEC,
    WAYPOINT_TOL,
)
from arm_route_planner.kinematics.linkage_models import wrap2pi

# Joint indices.
SHOULDER = 1
ELBOW = 2


def _clamp_step(target, angle, lo, hi, max_step):
    """Step *angle* toward *target* by at most *max_step*, inside limits."""
    diff = wrap2pi(target - angle)
    step = max(-max_step, min(max_step, diff))
    next_angle = angle + step
    return max(lo, min(hi, next_angle))


class RestingController:
    """State machine and slow-step generator for the RESTING operation."""

    name = "Resting"

    PHASE_IDLE = "IDLE"
    PHASE_CROUCH = "CROUCH"
    PHASE_PUSH_SHOULDER = "PUSH_SHOULDER"
    PHASE_BACKOFF_SHOULDER = "BACKOFF_SHOULDER"
    PHASE_PUSH_ELBOW = "PUSH_ELBOW"
    PHASE_BACKOFF_ELBOW = "BACKOFF_ELBOW"
    PHASE_COLLAPSED = "COLLAPSED"
    PHASE_HOLD = "HOLD"
    PHASE_TORQUE_ON = "TORQUE_ON"
    PHASE_MOVE_OUT = "MOVE_OUT"
    PHASE_AUTO_HOME = "AUTO_HOME"
    PHASE_DONE = "DONE"

    # Phases during which the arm is physically being moved.
    _MOVE_PHASES = (
        PHASE_CROUCH, PHASE_PUSH_SHOULDER, PHASE_BACKOFF_SHOULDER,
        PHASE_PUSH_ELBOW, PHASE_BACKOFF_ELBOW, PHASE_MOVE_OUT,
    )
    # Phases that push a joint into its physical stop.
    _PUSH_PHASES = (PHASE_PUSH_SHOULDER, PHASE_PUSH_ELBOW)

    def __init__(self, crouch=RESTING_CROUCH_TARGET, limits=RESTING_LIMITS):
        self.crouch = [float(a) for a in crouch]
        self.limits = [tuple(pair) for pair in limits]
        self.phase = self.PHASE_IDLE
        self.error: str | None = None

        # Push state
        self._push_joint: int | None = None
        self._push_target = 0.0
        self._push_start_angle = 0.0
        self._push_elapsed = 0.0
        self._window: list[tuple[float, float]] = []
        self._backoff_target: float | None = None

        # Planned-path following (crouch and move-out)
        self._path: list[list[float]] | None = None
        self._path_idx = 0
        self._path_max_steps: list[float] | None = None

        self._torque_on_emitted = False
        self._phase_elapsed = 0.0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    @property
    def active(self) -> bool:
        """True while the resting operation owns the arm."""
        return self.phase not in (self.PHASE_IDLE, self.PHASE_DONE)

    @property
    def collapsed(self) -> bool:
        """True while torque has been removed (COLLAPSED)."""
        return self.phase == self.PHASE_COLLAPSED

    @property
    def holding(self) -> bool:
        """True after an abort (torque stays on, pose held)."""
        return self.phase == self.PHASE_HOLD

    @property
    def in_restore(self) -> bool:
        """True while the restore half of the sequence is running."""
        return self.phase in (
            self.PHASE_TORQUE_ON, self.PHASE_MOVE_OUT, self.PHASE_AUTO_HOME)

    def _reset(self):
        self.error = None
        self._push_joint = None
        self._push_target = 0.0
        self._push_start_angle = 0.0
        self._push_elapsed = 0.0
        self._window = []
        self._backoff_target = None
        self._path = None
        self._path_idx = 0
        self._path_max_steps = None
        self._torque_on_emitted = False
        self._phase_elapsed = 0.0

    def begin_collapse(self) -> bool:
        """Start the collapse sequence.  Returns False if already active."""
        if self.active:
            return False
        self._reset()
        self.phase = self.PHASE_CROUCH
        return True

    def request_restore(self) -> bool:
        """Begin restore from COLLAPSED or HOLD.  Returns False otherwise."""
        if self.phase not in (self.PHASE_COLLAPSED, self.PHASE_HOLD):
            return False
        self._reset()
        self.phase = self.PHASE_TORQUE_ON
        return True

    def abort(self, message: str) -> None:
        """Abort an active sequence: hold the pose, torque stays on."""
        self.error = message
        self.phase = self.PHASE_HOLD

    def torque_confirmed(self) -> bool:
        """Move TORQUE_ON -> MOVE_OUT once the hardware re-enabled torque."""
        if self.phase != self.PHASE_TORQUE_ON:
            return False
        self.phase = self.PHASE_MOVE_OUT
        self._phase_elapsed = 0.0
        return True

    def homing_finished(self) -> bool:
        """Move AUTO_HOME -> DONE once the homing action completes."""
        if self.phase != self.PHASE_AUTO_HOME:
            return False
        self.phase = self.PHASE_DONE
        return True

    def set_path(self, waypoints: list[list[float]],
                 max_steps: list[float]) -> None:
        """Give the controller a planned joint path to follow.

        ``waypoints`` must not include the start pose; ``max_steps`` is the
        per-joint rad step for one control cycle.
        """
        self._path = [list(p) for p in waypoints]
        self._path_idx = 0
        self._path_max_steps = list(max_steps)

    # ------------------------------------------------------------------
    # Cycle update
    # ------------------------------------------------------------------

    def update(self, angles, dt=DT, feedback_fresh=True, enabled=True):
        """Advance one control cycle.

        Args:
            angles: Measured joint angles [rad].
            dt: Control period [s].
            feedback_fresh: False when no fresh measured sample arrived.
            enabled: False when the arm has been disabled.

        Returns:
            (target, max_step, events): target is the joint target to step
            toward or None to hold; max_step is a scalar rad step cap or
            None; events is a list of transition events ("torque_off",
            "torque_on", "home", "done", "abort:<message>").
        """
        if not self.active:
            return None, None, []
        events: list[str] = []
        self._phase_elapsed += dt

        if self.phase in (self.PHASE_COLLAPSED, self.PHASE_HOLD,
                          self.PHASE_AUTO_HOME):
            return None, None, events

        if self.phase == self.PHASE_TORQUE_ON:
            if not self._torque_on_emitted:
                self._torque_on_emitted = True
                events.append("torque_on")
            return None, None, events

        # Push phases need live feedback and an enabled arm.  A disabled arm
        # or dead feedback must abort instead of being read as a stall.
        if self.phase in self._PUSH_PHASES:
            if not feedback_fresh:
                self.abort("Resting aborted: position feedback lost")
                return None, None, ["abort:" + self.error]
            if not enabled:
                self.abort("Resting aborted: arm disabled")
                return None, None, ["abort:" + self.error]

        if self.phase in (self.PHASE_CROUCH, self.PHASE_MOVE_OUT):
            target, max_step, phase_events = self._step_waypoint_move(
                angles, dt, self.crouch)
            events.extend(phase_events)
            return target, max_step, events

        if self.phase in self._PUSH_PHASES:
            target, max_step, phase_events = self._step_push(angles, dt)
            events.extend(phase_events)
            return target, max_step, events

        if self.phase in (self.PHASE_BACKOFF_SHOULDER,
                          self.PHASE_BACKOFF_ELBOW):
            target, max_step, phase_events = self._step_backoff(angles, dt)
            events.extend(phase_events)
            return target, max_step, events

        return None, None, events

    # ------------------------------------------------------------------
    # Phase helpers
    # ------------------------------------------------------------------

    def _start_push(self, joint: int, push_target: float, angles):
        self._push_joint = joint
        self._push_target = push_target
        self._push_start_angle = float(angles[joint])
        self._push_elapsed = 0.0
        self._window = []
        self._phase_elapsed = 0.0

    def _slow_target(self, angles, joint, target, rate, dt):
        """Step one joint toward *target* at *rate* and return the list."""
        result = list(angles)
        lo, hi = self.limits[joint]
        clamped = max(lo, min(hi, float(target)))
        result[joint] = _clamp_step(
            clamped, result[joint], lo, hi, rate * dt)
        return result

    def _step_push(self, angles, dt):
        """Step the pushed joint and run the stall/abort windowing."""
        joint = self._push_joint
        measured = float(angles[joint])
        self._push_elapsed += dt
        self._window.append((self._push_elapsed, measured))
        self._window = [
            (t, a) for t, a in self._window
            if self._push_elapsed - t <= RESTING_STALL_WINDOW_SEC
        ]

        lo, hi = self.limits[joint]
        direction = 1.0 if self._push_target >= self._push_start_angle \
            else -1.0

        # Grace: verify the joint actually moved the right way before the
        # stall windowing arms.
        if self._push_elapsed >= RESTING_START_GRACE_SEC:
            progress = abs(measured - self._push_start_angle)
            if progress < RESTING_STALL_MOVE_EPS:
                self.abort(
                    "Resting aborted: no motion during the start grace "
                    f"(joint {joint} may be disabled or feedback is dead)")
                return None, None, ["abort:" + self.error]
            if (measured - self._push_start_angle) * direction < \
                    -RESTING_STALL_MOVE_EPS:
                self.abort(
                    "Resting aborted: joint moved the wrong direction "
                    "during the start grace")
                return None, None, ["abort:" + self.error]

            if self._push_elapsed >= RESTING_PUSH_TIMEOUT_SEC:
                self.abort(
                    f"Resting aborted: push timeout on joint {joint}")
                return None, None, ["abort:" + self.error]

            # Reached the push target without stalling: the physical stop is
            # not where it was expected.  Checked before the stall window so
            # a joint that drove all the way to the target aborts instead of
            # being misread as stalled.
            if abs(self._push_target - measured) <= RESTING_PUSH_ARRIVE_TOL:
                self.abort(
                    "Resting aborted: reached the push target without "
                    f"stalling (stop missing on joint {joint})")
                return None, None, ["abort:" + self.error]

            # Reached the padded limit without stalling (the elbow push
            # target equals the padded bound, so this is the same check).
            if measured <= lo + RESTING_PUSH_ARRIVE_TOL or \
                    measured >= hi - RESTING_PUSH_ARRIVE_TOL:
                self.abort(
                    "Resting aborted: reached the padded limit without "
                    f"stalling (joint {joint})")
                return None, None, ["abort:" + self.error]

            # Windowed stall: no advance over the observation window.
            if (len(self._window) >= 2
                    and self._window[-1][0] - self._window[0][0]
                    >= RESTING_STALL_WINDOW_SEC):
                moved = abs(self._window[-1][1] - self._window[0][1])
                if moved < RESTING_STALL_MOVE_EPS:
                    self._backoff_target = measured + (
                        RESTING_BACKOFF_ANGLE if joint == SHOULDER
                        else -RESTING_BACKOFF_ANGLE)
                    self.phase = (
                        self.PHASE_BACKOFF_SHOULDER if joint == SHOULDER
                        else self.PHASE_BACKOFF_ELBOW)
                    self._phase_elapsed = 0.0
                    return None, None, []

        target = self._slow_target(
            angles, joint, self._push_target, RESTING_PUSH_RATE, dt)
        return target, RESTING_PUSH_RATE * dt, []

    def _step_backoff(self, angles, dt):
        """Step to the backoff angle and hand over to the next phase."""
        joint = self._push_joint
        target = self._backoff_target
        if self._phase_elapsed >= RESTING_PUSH_TIMEOUT_SEC:
            self.abort(
                f"Resting aborted: backoff timeout on joint {joint}")
            return None, None, ["abort:" + self.error]

        result = self._slow_target(
            angles, joint, target, RESTING_PUSH_RATE, dt)
        if abs(wrap2pi(target - float(angles[joint]))) \
                <= RESTING_PUSH_ARRIVE_TOL:
            if joint == SHOULDER:
                self._start_push(
                    ELBOW, RESTING_ELBOW_PUSH, angles)
                self.phase = self.PHASE_PUSH_ELBOW
            else:
                self.phase = self.PHASE_COLLAPSED
                return None, None, ["torque_off"]
        return result, RESTING_PUSH_RATE * dt, []

    def _step_waypoint_move(self, angles, dt, goal):
        """Follow the planned path, then step directly to *goal*."""
        tol = WAYPOINT_TOL

        if self._path is not None:
            if self._phase_elapsed >= RESTING_MOVE_TIMEOUT_SEC:
                self.abort("Resting aborted: move timeout")
                return None, None, ["abort:" + self.error]
            target = list(self._path[self._path_idx])
            arrived = all(
                abs(wrap2pi(t - a)) <= tol
                for t, a in zip(target, angles))
            if arrived:
                self._path_idx += 1
                if self._path_idx >= len(self._path):
                    self._path = None
                    self._path_max_steps = None
                else:
                    target = list(self._path[self._path_idx])
            if self._path is not None:
                max_step = self._path_max_steps
                if max_step is None:
                    max_step = RESTING_MOVE_ANG_VEL * dt
                return target, max_step, []

        # Direct slow step: every joint moves at the resting speed.
        result = list(angles)
        for i in range(len(result)):
            lo, hi = self.limits[i]
            result[i] = _clamp_step(
                max(lo, min(hi, goal[i])), result[i], lo, hi,
                RESTING_MOVE_ANG_VEL * dt)

        if all(abs(wrap2pi(g - a)) <= tol
               for g, a in zip(goal, result)):
            if self.phase == self.PHASE_CROUCH:
                self._start_push(
                    SHOULDER, RESTING_SHOULDER_PUSH, angles)
                self.phase = self.PHASE_PUSH_SHOULDER
            else:
                self.phase = self.PHASE_AUTO_HOME
                return None, None, ["home"]
        return result, RESTING_MOVE_ANG_VEL * dt, []
