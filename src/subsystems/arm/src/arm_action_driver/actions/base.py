"""Declarative action configuration base for the arm action driver.

Each action is a parameter block.  The driver node owns all of the motion
behaviour - target computation, path planning, ramp stepping, arrival
detection, pause/resume, hold-key timeouts, release modes, startup
adoption, gating, progress - and reads these parameters to decide *how*
to move.  Adding an action = declare the parameters (+ optionally one
``target`` helper) and append the class to the registry; the node is
never touched.

Parameters:

Identity and routing:
    name:               human-readable name used in logs and errors.
    active_value:       /ARM/ACTION/ACTIVE enum while the action owns
                        the arm (never ACTION_ACTIVE_IDLE).
    trigger_commands:   command strings that start the action.
    release_commands:   command strings that pause/release it (the flip,
                        zenith, and homing key-up events).
    single_joint_prefix:  "home_joint:"-style prefix: the trailing index
                        selects a single joint to move.
    adopt_modes:        startup inference modes this action adopts
                        ("resting", "zenith") via ``adopt:<mode>``.

Movement:
    target_angles:      static target pose, or override ``target()``.
    target(driver):     computed target pose (defaults to target_angles).
    exit_target(driver): toggle-exit pose for toggle_on_repeat actions.
    limits:             limits applied while stepping.
    waypoint_tol:       per-waypoint arrival tolerance [rad].
    arrive_tol:         final arrival tolerance (defaults to waypoint_tol).
    exit_arrive_tol:    toggle-exit arrival tolerance.
    deadband_target:    pin the elbow to the FINAL target's side of the
                        singularity deadband (homing).
    deadband_ok:        skip the deadband pin entirely (zenith hold).
    use_safe_path:      plan a joint-space path when unlocked.
    safe_path_required: refuse to start when no safe path exists (flip);
                        otherwise fall back to a direct move (homing).
    hold_mode:          keep holding the target; never auto-finish (the
                        zenith holding pose).

Behaviour:
    requires_planar:            refuse to start outside planar mode.
    planar_except_inferred:     ...unless the pose is already resting.
    octant_requires_planar:     refuse the flip in a restricted octant
                                unless planar mode is active.
    hold_key:                   hold-to-run action: re-triggering keeps it
                                running and refreshes the timer.
    hold_timeout_sec:           release this long after the last trigger.
    restart_on_repeat:          re-trigger while active refreshes the hold
                                timer and keeps the move running; a new
                                press after release restarts from the
                                current pose (homing).
    pause_on_release:           release command decelerates to a paused
                                idle with the goal remembered (flip).
    stop_on_release:            release command decelerates to idle and
                                drops the goal (homing, zenith).
    resume_on_repeat:           re-trigger resumes the remembered goal.
    cancel_on_manual:           SpaceMouse motion drops the goal.
    toggle_on_repeat:           re-trigger while active runs the exit
                                move (zenith) / restore (resting).
    preempts:                   active values this action may start over
                                (homing may preempt zenith).
    blocked_reasons:            {active value: reason} custom rejection
                                text when this action refuses to preempt.
    report_homing_mask:         report the joint mask in status.

Engine:
    engine:             "move" (the generic waypoint engine) or
                        "resting" (the collapse/restore engine).
    controller_class:   resting: the RestingController subclass to drive.
    crouch_target:      resting: the crouch pose the collapse moves
                        through before the pushes.
"""

from arm_constants import (
    ACTION_ACTIVE_IDLE,
    ARM_JOINT_NAMES,
    MECHANICAL_LIMITS,
    WAYPOINT_TOL,
)

# Joint indices, shared by every action.
BASE = 0
SHOULDER = 1
ELBOW = 2
WRIST_BEND = 3
WRIST_TWIST = 4

JOINT_INDEX = {joint: index for index, joint in enumerate(ARM_JOINT_NAMES)}


def _clamp(value):
    """Clamp a 0..1 progress estimate."""
    return max(0.0, min(1.0, float(value)))


class ActionConfiguration:
    """Parameter block describing one arm action.  See module docstring."""

    # Identity and routing
    name = "action"
    active_value = ACTION_ACTIVE_IDLE
    trigger_commands: tuple = ()
    release_commands: tuple = ()
    single_joint_prefix = None
    adopt_modes: tuple = ()

    # Movement
    target_angles = None
    limits = MECHANICAL_LIMITS
    waypoint_tol = WAYPOINT_TOL
    arrive_tol = None
    exit_arrive_tol = None
    deadband_target = False
    deadband_ok = False
    use_safe_path = True
    safe_path_required = True
    hold_mode = False

    # Behaviour
    requires_planar = False
    planar_except_inferred = False
    octant_requires_planar = False
    hold_key = False
    hold_timeout_sec = 0.0
    restart_on_repeat = False
    pause_on_release = False
    stop_on_release = False
    resume_on_repeat = False
    cancel_on_manual = False
    toggle_on_repeat = False
    preempts: tuple = ()
    blocked_reasons: dict = {}
    report_homing_mask = False
    remember_entry_side = False

    # Engine selection
    engine = "move"
    controller_class = None
    crouch_target = None

    # ------------------------------------------------------------------
    # Command routing (all declarative)
    # ------------------------------------------------------------------

    def matches(self, command: str) -> bool:
        """True when this action handles ``command``."""
        return command in self.trigger_commands \
            or command in self.release_commands \
            or (self.single_joint_prefix is not None
                and command.startswith(self.single_joint_prefix))

    def is_release(self, command: str) -> bool:
        """True when ``command`` is a release/pause event."""
        return command in self.release_commands

    def is_active(self, driver) -> bool:
        """True when this action currently owns the arm."""
        return driver.active == self.active_value

    # ------------------------------------------------------------------
    # Target helpers (override only when the pose is computed)
    # ------------------------------------------------------------------

    def target(self, driver):
        """Final target pose for the action."""
        if self.target_angles is None:
            raise NotImplementedError(
                f"{self.name}: declare target_angles or override target()")
        return list(self.target_angles)

    def exit_target(self, driver):
        """Toggle-exit pose; None when the action has no exit move."""
        return None
