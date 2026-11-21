"""Arm action driver node.

Owns the discrete, mostly kinematic-free arm actions that used to run inside
``arm_route_planner_node``: homing, side flips, the zenith pose, and the
resting collapse/restore sequence.  It consumes the measured joint angles and
publishes the five joint targets only while an action is active; the planner
suppresses its own targets while ``/ARM/ACTION/ACTIVE`` is nonzero.

All motion behaviour lives here.  Each action is an ``ActionConfiguration``
parameter block (``arm_action_driver/actions/*.py``) that tells the node how
to move: target pose, cruise speed, limits, tolerances, and the
hold/pause/resume/toggle/adopt flags.  Two engines turn those parameters
into motion:

- the generic waypoint engine (``_begin_move`` / ``_step_move``) drives
  homing, flips, and the zenith hold/exit through the shared ramp;
- the resting engine (``_step_resting``) drives the collapse/restore
  state machine (``RestingController``) with its torque events and the
  homing handoff.

Adding an action means adding a parameter block and appending it to the
registry (``arm_action_driver/actions/__init__.py``); this file is not
touched.

Design details and the behaviour rules live in
``planz/ACTION_DRIVER_PLAN.md``.  The planner keeps modes, joint locks, and
limit selection; this node owns motion, the action state machine, and the
resting hardware latches.

Inputs:
    /ARM/JOINT/CURRENT_*       std_msgs/Float32  (5-DOF joints)
    /ARM/ACTION/COMMAND        custom_interfaces/ArmCommand
    /ARM/MANUAL/CMD_VELOCITY   custom_interfaces/Velocity
    /ARM/STATE/{ENABLED,PLANAR,FLIPPED,LOCKED_JOINTS,TORQUE_ENABLED}
    /ESTOP                     std_msgs/Bool

Outputs:
    /ARM/JOINT/TARGET_*        std_msgs/Float32
    /ARM/ACTION/ACTIVE         std_msgs/Int8     (IDLE/HOMING/FLIPPING/...)
    /ARM/ACTION/STATUS         custom_interfaces/ArmActionStatus
    /ARM/HARDWARE/RESTING      std_msgs/Bool
    /ARM/HARDWARE/TORQUE_ENABLED std_msgs/Bool
"""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from custom_interfaces.msg import ArmActionStatus, ArmCommand, Velocity
from std_msgs.msg import Bool, Float32, Int8, UInt8

from arm_constants import (
    ACTION_ACCEL_RAD_S2,
    ACTION_ACTIVE_IDLE,
    ACTION_ACTIVE_RESTING,
    ACTION_ACTIVE_TOPIC,
    ACTION_ACTIVE_ZENITH,
    ACTION_COMMAND_TOPIC,
    ACTION_DECEL_RAD_S2,
    ACTION_STATUS_TOPIC,
    ARM_ENABLED_TOPIC,
    ARM_JOINT_NAMES,
    ARM_RESTING_TOPIC,
    ARM_STATE_TOPICS,
    ARM_TORQUE_ENABLED_TOPIC,
    ARM_TORQUE_STATE_TOPIC,
    DT,
    ELBOW_DEADBAND,
    ESTOP_TOPIC,
    JOINT_CURRENT_TOPICS,
    JOINT_TARGET_TOPICS,
    OCTANT_DIRECT_FLIP,
    PHASE_FLIP_PAUSING,
    PHASE_FLIP_RUNNING,
    PHASE_ZENITH_EXITING,
    RESTING_FEEDBACK_TIMEOUT_SEC,
    RESTING_LATCH_REFRESH_SEC,
    RESTING_PUSH_RATE,
)
from arm_action_driver.actions import ACTION_CONFIG_CLASSES
from arm_action_driver.actions.base import JOINT_INDEX, _clamp
from arm_action_driver.actions.homing import HomingConfiguration
from arm_action_driver.actions.resting import RestingConfiguration
from arm_action_driver.actions.zenith import ZenithConfiguration
from arm_action_driver.stepping import JointRamp
from arm_route_planner.kinematics.limiters import _get_octant
from arm_route_planner.kinematics.linkage_models import (
    limit_respecting_diff, wrap2pi,
)
from arm_route_planner.kinematics.path_planner import (
    build_arm_solver_object, plan_safe_path,
)
from arm_route_planner.kinematics.regions import (
    is_resting_pose, is_zenith_pose,
)
from rover_protocol.arm_errors import (
    ARM_ERROR_ESTOP,
    ARM_ERROR_NONE,
    classify_failure,
)

MANUAL_MOTION_EPS = 1e-3

# Single cruise speed shared by every action (homing, flip, zenith, resting).
# The node owns motion, so action parameter blocks do not carry speeds.
MOVE_SPEED_RAD_S = math.radians(25.0)

# Sentinel returned by the single-joint command parser for invalid input.
_BAD_INDEX = -1


class _MoveState:
    """Shared waypoint-move state for the generic action engine."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.goal = None        # remembered final goal (flip pause/resume)
        self.target = None      # final target (deadband + progress)
        self.path = None        # remaining waypoints
        self.idx = 0
        self.mask = None
        self.total = None       # remaining distance at start (progress)
        self.command_at = 0.0   # hold-key refresh stamp
        self.single_index = None  # single-joint index, or None for all
        self.pausing = False    # release paused the move in place
        self.exiting = False    # toggle-exit sub-mode (zenith)
        self.exiting_target = None


class ArmActionDriverNode(Node):
    """Drive the arm's discrete actions and publish their joint targets.

    The attributes and methods without a leading underscore form the action
    API used by the ``ActionConfiguration`` target helpers (see
    ``arm_action_driver/actions/base.py`` for the contract).
    """

    def __init__(self):
        super().__init__("arm_action_driver_node")

        # FK model for octant gates and safe joint-space paths.  No IK
        # solving happens here.
        self.model, _ = build_arm_solver_object()

        # Measured joint angles and the wall time of each sample.
        self.measured = [float("nan")] * len(ARM_JOINT_NAMES)
        self.measured_at = [0.0] * len(ARM_JOINT_NAMES)

        # Mirrored arm state.
        self.estop = False
        self.enabled = False
        self.planar = False
        self.flipped = False
        self.locked = [False] * len(ARM_JOINT_NAMES)
        self.torque_state = False

        # Action state.  ``active`` is the published enum; ``phase`` is the
        # per-action substate carried in ArmActionStatus.
        self.active = ACTION_ACTIVE_IDLE
        self.phase = PHASE_FLIP_RUNNING

        # Shared stepping profile and waypoint-move state.
        self.ramp = JointRamp(
            len(ARM_JOINT_NAMES), ACTION_ACCEL_RAD_S2,
            ACTION_DECEL_RAD_S2, DT)
        self._move = _MoveState()

        # Resting-owned torque intent (the hardware latch level).
        self.torque_intent = True
        # Homing handoff from a resting restore.
        self.restore_after_homing = False

        # Manual (SpaceMouse) motion cancels a remembered flip goal.
        self._manual_motion = False

        # Startup adoption state.
        self._adopted = False
        self._pending_adopt = None

        # Error event bookkeeping for ArmActionStatus.
        self._error = (ARM_ERROR_NONE, 0)
        self._error_seq = 0

        self._last_targets = None
        self._latch_at = 0.0

        # Action registry: one parameter block per action.  The node does
        # not know what any action does beyond these parameters.
        self._actions = [cls() for cls in ACTION_CONFIG_CLASSES]
        self._by_active = {
            config.active_value: config for config in self._actions
        }
        self.homing = self._find(HomingConfiguration)
        self.resting = self._find(RestingConfiguration)
        self._zenith = self._find(ZenithConfiguration)

        # Resting engine state.
        # Crouch and restore travel use the shared action speed; only the
        # limit pushes (and their backoffs) are deliberately slow.
        self._resting_controller = self.resting.controller_class(
            crouch=self.resting.crouch_target,
            move_speed=MOVE_SPEED_RAD_S,
            push_speed=RESTING_PUSH_RATE)
        self._resting_path_phase = None

        self._create_interfaces()
        self.create_timer(DT, self._update)
        self.get_logger().info("Arm action driver started")

    def _find(self, cls):
        for config in self._actions:
            if isinstance(config, cls):
                return config
        raise RuntimeError(f"Action configuration {cls.__name__} "
                           "is not registered")

    # ------------------------------------------------------------------
    # ROS interfaces
    # ------------------------------------------------------------------

    def _create_interfaces(self):
        self._target_publishers = {
            joint: self.create_publisher(
                Float32, JOINT_TARGET_TOPICS[joint], 10)
            for joint in ARM_JOINT_NAMES
        }
        self._active_publisher = self.create_publisher(
            Int8, ACTION_ACTIVE_TOPIC, 10)
        self._status_publisher = self.create_publisher(
            ArmActionStatus, ACTION_STATUS_TOPIC, 10)
        self._resting_publisher = self.create_publisher(
            Bool, ARM_RESTING_TOPIC, 10)
        self._torque_publisher = self.create_publisher(
            Bool, ARM_TORQUE_ENABLED_TOPIC, 10)

        self.create_subscription(
            ArmCommand, ACTION_COMMAND_TOPIC, self._on_command, 10)
        self.create_subscription(
            Velocity, "/ARM/MANUAL/CMD_VELOCITY",
            self._on_manual_velocity, 10)
        self.create_subscription(
            Bool, ESTOP_TOPIC, self._on_estop, 10)
        self.create_subscription(
            Bool, ARM_ENABLED_TOPIC, self._on_enabled, 10)
        self.create_subscription(
            Bool, ARM_STATE_TOPICS["flags"]["PLANAR"],
            self._on_planar, 10)
        self.create_subscription(
            Bool, ARM_STATE_TOPICS["flags"]["FLIPPED"],
            self._on_flipped, 10)
        self.create_subscription(
            UInt8, ARM_STATE_TOPICS["locked_joints"],
            self._on_locked, 10)
        self.create_subscription(
            Bool, ARM_TORQUE_STATE_TOPIC, self._on_torque_state, 10)
        for joint in ARM_JOINT_NAMES:
            self.create_subscription(
                Float32,
                JOINT_CURRENT_TOPICS[joint],
                self._on_measured(joint),
                10,
            )

    # ------------------------------------------------------------------
    # Input callbacks
    # ------------------------------------------------------------------

    def _on_measured(self, joint: str):
        index = JOINT_INDEX[joint]

        def callback(msg: Float32):
            self.measured[index] = wrap2pi(msg.data)
            self.measured_at[index] = time.monotonic()
        return callback

    def _on_estop(self, msg: Bool):
        self.estop = bool(msg.data)
        if not self.estop:
            # Resume from rest instead of jumping back into the old speed.
            self.ramp.reset()

    def _on_enabled(self, msg: Bool):
        self.enabled = bool(msg.data)

    def _on_planar(self, msg: Bool):
        self.planar = bool(msg.data)

    def _on_flipped(self, msg: Bool):
        self.flipped = bool(msg.data)

    def _on_locked(self, msg: UInt8):
        self.locked = [bool(msg.data & (1 << i))
                       for i in range(len(ARM_JOINT_NAMES))]

    def _on_torque_state(self, msg: Bool):
        self.torque_state = bool(msg.data)

    def _on_manual_velocity(self, msg: Velocity):
        values = (
            msg.linear_x, msg.linear_y, msg.linear_z,
            msg.angular_x, msg.angular_y, msg.angular_z,
        )
        if any(abs(float(v)) > MANUAL_MOTION_EPS for v in values):
            self._manual_motion = True

    def _on_command(self, msg: ArmCommand):
        command = msg.command.strip()
        if not command:
            return
        self._handle_command(command)

    # ------------------------------------------------------------------
    # Command handling
    # ------------------------------------------------------------------

    def _handle_command(self, command: str):
        if command == "adopt" or command.startswith("adopt:"):
            mode = command.partition(":")[2] or None
            if not self._initialized():
                # The driver may still be compiling its model; apply the
                # planner's inference as soon as feedback is complete.
                self._pending_adopt = mode or "auto"
                return
            self._adopt_startup_pose(mode)
            return
        if not self._initialized():
            self.reject(f"Action command '{command}' ignored: "
                        "arm not initialized")
            return
        if self.estop:
            self.emit_error(
                ARM_ERROR_ESTOP,
                reason=f"Action command '{command}' ignored: e-stop active")
            return

        if command in ("hold", "stop"):
            self._abort_action("Action held by command")
            return

        config = self._match_command(command)
        if config is None:
            self.reject(f"Unknown action command '{command}'")
            return
        if config.is_release(command):
            self._handle_release(config)
            return
        self.trigger_action(config, command)

    def _match_command(self, command: str):
        for config in self._actions:
            if config.matches(command):
                return config
        return None

    def trigger_action(self, config, command=None):
        """Start (or re-trigger) one action via the normal gating path."""
        if config.is_active(self):
            self._repeat(config, command)
            return
        ok, reason = self._can_trigger(config)
        if not ok:
            self.reject(reason)
            return
        self._begin(config, command)

    def _can_trigger(self, config):
        if self.active == ACTION_ACTIVE_IDLE:
            return True, ""
        if self.active in config.preempts:
            return True, ""
        reason = config.blocked_reasons.get(self.active)
        if reason is not None:
            return False, reason
        return False, f"{config.name} blocked: another action is running"

    def _handle_release(self, config):
        if self.active != config.active_value:
            # Releasing while idle keeps any remembered goal for the next
            # press.
            return
        if config.stop_on_release:
            # A resting restore drives homing without a held key; a stray
            # key-up must not abandon the restore halfway through.
            if not self.restore_after_homing:
                self._stop_move(config)
            return
        if config.pause_on_release \
                and not self._move.pausing and not self._move.exiting:
            self._move.pausing = True
            self.phase = PHASE_FLIP_PAUSING

    def _repeat(self, config, command=None):
        """Handle a trigger command while this action is already active."""
        if config.engine == "resting":
            self._repeat_resting(config)
            return
        if config.toggle_on_repeat:
            self._begin_exit(config)
            return
        if config.restart_on_repeat:
            self._refresh_move(config, command)

    def _refresh_move(self, config, command=None):
        """Handle a held-key repeat without restarting the motion.

        The basestation forwards hold-to-run commands on every held state
        (tens of Hz).  Re-planning and resetting the ramp on each heartbeat
        would restart the acceleration profile continuously, so homing
        would crawl.  An identical repeat only refreshes the hold timer;
        a different joint or lock set still restarts the move.
        """
        if not self._move.pausing and not self._move.exiting \
                and self._move.target is not None:
            index = self._single_joint_index(config, command)
            # Latch the target chosen when the move began: a held-key
            # repeat must not re-evaluate side- or pose-dependent targets
            # (homing from zenith always returns to the non-flipped home).
            unchanged = (
                index == self._move.single_index
                and (index is not None
                     or self.unlocked_mask() == self._move.mask))
            if unchanged:
                self._move.command_at = time.monotonic()
                return
        self._begin(config, command)

    # ------------------------------------------------------------------
    # Startup adoption
    # ------------------------------------------------------------------

    def _adopt_startup_pose(self, mode=None):
        """Adopt an inferred zenith/resting pose.  Runs at most once.

        ``mode`` is the planner's inferred mode from ``adopt:<mode>``.  With
        no mode it falls back to classifying the measured pose, but only
        when the planner has not already chosen a normal/planar start.
        """
        if self._adopted:
            return
        if self.active != ACTION_ACTIVE_IDLE:
            self._adopted = True
            return
        self._adopted = True
        if mode == "resting" or (mode is None and is_resting_pose(
                self.measured)):
            self._adopt_resting()
            return
        if mode == "zenith" or (mode is None and is_zenith_pose(
                self.measured)):
            self._begin(self._zenith, adopted=True)

    # ------------------------------------------------------------------
    # Generic waypoint engine (homing, flip, zenith)
    # ------------------------------------------------------------------

    def _begin(self, config, command=None, adopted=False):
        if config.engine == "resting":
            self._begin_resting(config)
            return
        self._begin_move(config, command=command, adopted=adopted)

    def _begin_move(self, config, command=None, adopted=False):
        index = self._single_joint_index(config, command)
        if index == _BAD_INDEX:
            self.reject(f"Unknown action command '{command}'")
            return

        start = list(self.measured)
        mask = self.unlocked_mask()
        if index is not None:
            mask = [i == index for i in range(len(mask))]

        resumed = False
        if not adopted and config.resume_on_repeat \
                and self._move.goal is not None:
            # Resume keeps the original goal; the path is re-planned from
            # wherever the arm is now.
            target = list(self._move.goal)
            resumed = True
        else:
            if config.octant_requires_planar:
                ok, reason = self._octant_gate(config)
                if not ok:
                    self.reject(reason)
                    return
            target = config.target(self)

        goal = [target[i] if mask[i] else start[i]
                for i in range(len(target))]
        path = self._plan_path(config, start, goal, index)
        if path is None:
            self.reject(
                f"{config.name} blocked: no safe path through "
                "restricted space")
            return

        self._move.goal = goal if config.resume_on_repeat else None
        self._move.target = target
        self._move.path = [list(p) for p in path[1:]] or [goal]
        self._move.idx = 0
        self._move.mask = mask
        self._move.total = self._masked_max_remaining(target, mask)
        self._move.command_at = time.monotonic()
        self._move.single_index = index
        self._move.pausing = False
        self._move.exiting = False
        self._move.exiting_target = None
        if config.remember_entry_side:
            config.entry_side = self.flipped

        self.active = config.active_value
        self.phase = PHASE_FLIP_RUNNING
        self.ramp.reset()

        if adopted:
            self.get_logger().info(
                f"{config.name} inferred from startup pose")
        elif resumed:
            self.get_logger().info(f"{config.name} resumed")
        elif index is not None:
            self.get_logger().info(f"{config.name} joint {index}")
        else:
            self.get_logger().info(f"{config.name} started")

    def _single_joint_index(self, config, command):
        if config.single_joint_prefix is None or not command \
                or not command.startswith(config.single_joint_prefix):
            return None
        try:
            index = int(command.partition(":")[2])
        except ValueError:
            return _BAD_INDEX
        if not 0 <= index < len(ARM_JOINT_NAMES):
            return _BAD_INDEX
        return index

    def _octant_gate(self, config):
        octant = self.octant()
        if octant is not None and octant not in OCTANT_DIRECT_FLIP:
            if not self.planar:
                return False, (
                    f"{config.name} blocked: octant {octant} "
                    "requires planar")
        return True, ""

    def _plan_path(self, config, start, goal, single_index=None):
        if not config.use_safe_path or any(self.locked[:3]) \
                or (single_index is not None and single_index < 3):
            return [start, goal]
        path = plan_safe_path(self.model, start, goal)
        if path is None and not config.safe_path_required:
            # Refuse to lose a homing move over a planner miss; the ramp
            # and mechanical limits still bound the direct move.
            path = [start, goal]
        return path

    def _begin_exit(self, config):
        if self._move.exiting:
            return
        exit_target = config.exit_target(self)
        if exit_target is None:
            return
        self._move.exiting = True
        self._move.pausing = False
        self._move.exiting_target = exit_target
        self.phase = PHASE_ZENITH_EXITING
        self.ramp.reset()
        self.get_logger().info(f"{config.name} exit started")

    def _step_move(self, config):
        if config.hold_key and config.hold_timeout_sec \
                and not self.restore_after_homing:
            if time.monotonic() - self._move.command_at \
                    > config.hold_timeout_sec:
                self._stop_move(config)
                return

        if self._move.pausing:
            self._step_pause(config)
            return

        exiting = self._move.exiting
        target = self._move.exiting_target if exiting \
            else self._move.path[self._move.idx]
        final_leg = exiting or self._move.idx + 1 >= len(self._move.path)
        cmd = self.ramp_command(
            target, [MOVE_SPEED_RAD_S] * len(ARM_JOINT_NAMES), config.limits,
            mask=self._move.mask,
            deadband_ok=config.deadband_ok and not exiting,
            deadband_target=target if config.deadband_target else None,
            brake=final_leg)
        self.publish_targets(cmd)

        if exiting:
            tol = config.exit_arrive_tol \
                if config.exit_arrive_tol is not None \
                else config.waypoint_tol
            if all(abs(wrap2pi(target[i] - self.measured[i])) <= tol
                   for i in range(len(target))):
                self._finish_move(config)
            return

        if config.hold_mode:
            return

        mask = self._move.mask
        if all(not mask[i] or abs(wrap2pi(target[i] - self.measured[i]))
               <= config.waypoint_tol for i in range(len(target))):
            if self._move.idx + 1 < len(self._move.path):
                self._move.idx += 1
            else:
                final_tol = config.arrive_tol \
                    if config.arrive_tol is not None \
                    else config.waypoint_tol
                if all(not mask[i]
                       or abs(wrap2pi(self._move.target[i]
                                      - self.measured[i])) <= final_tol
                       for i in range(len(self._move.target))):
                    self._finish_move(config)

    def _step_pause(self, config):
        target = self._move.path[self._move.idx] \
            if self._move.path else self.measured
        cmd = self.ramp_command(
            target, [0.0] * len(ARM_JOINT_NAMES), config.limits,
            mask=self._move.mask)
        self.publish_targets(cmd)
        if self.ramp.is_stopped():
            self.publish_targets(self.measured)
            self._move.pausing = False
            self.active = ACTION_ACTIVE_IDLE
            self.phase = PHASE_FLIP_RUNNING
            if self._move.goal is None:
                self._move.reset()
            self.get_logger().info(f"{config.name} paused")

    def _stop_move(self, config):
        self.publish_targets(self.measured)
        self._move.reset()
        self.active = ACTION_ACTIVE_IDLE
        self.get_logger().info(f"{config.name} released; stopped")

    def _finish_move(self, config):
        self.publish_targets(self.measured)
        self._move.reset()
        self.phase = PHASE_FLIP_RUNNING
        # A resting restore hands off to homing; completing it finishes
        # the restore too.
        if not self.homing_completed():
            self.active = ACTION_ACTIVE_IDLE
        self.get_logger().info(f"{config.name} complete")

    # ------------------------------------------------------------------
    # Resting engine
    # ------------------------------------------------------------------

    def _begin_resting(self, config):
        inferred = is_resting_pose(self.measured)
        if config.requires_planar and not self.planar:
            if not (config.planar_except_inferred and inferred):
                self.reject(
                    f"{config.name} blocked: planar mode is required")
                return

        if not self._resting_controller.begin_collapse():
            self.reject(f"{config.name} blocked: sequence already running")
            return

        if inferred:
            # Startup pose: stay de-torqued and wait for the restore.
            self._resting_controller.abort(
                "Resting mode inferred from startup pose")
            self.torque_intent = False
            self.publish_torque(False)
            self.get_logger().info(
                "Resting inferred from startup pose; torque stays off")
        else:
            self.torque_intent = True
            self.get_logger().info("Resting collapse started")

        self._resting_path_phase = None
        self.active = ACTION_ACTIVE_RESTING
        self.ramp.reset()

    def _repeat_resting(self, config):
        controller = self._resting_controller
        if controller.phase in (controller.PHASE_COLLAPSED,
                                controller.PHASE_HOLD):
            if controller.request_restore():
                self.torque_intent = True
                self.publish_torque(True)
                self.get_logger().info("Resting restore started")
        elif controller.active:
            controller.abort("Resting sequence aborted by operator")
            self.get_logger().warning("Resting sequence aborted")

    def _adopt_resting(self):
        self._resting_controller.begin_collapse()
        self._resting_controller.abort(
            "Resting mode inferred from startup pose")
        self.torque_intent = False
        self.publish_torque(False)
        self._resting_path_phase = None
        self.active = ACTION_ACTIVE_RESTING
        self.ramp.reset()
        self.get_logger().info(
            "Resting inferred from startup pose; torque stays off")

    def _step_resting(self, config):
        controller = self._resting_controller
        fresh = all(
            time.monotonic() - at <= RESTING_FEEDBACK_TIMEOUT_SEC
            for at in self.measured_at)
        self._plan_resting_path(config)
        target, max_step, events = controller.update(
            self.measured, DT, feedback_fresh=fresh,
            enabled=self.enabled)
        for event in events:
            self._handle_resting_event(config, event)

        if controller.phase == controller.PHASE_TORQUE_ON \
                and self.torque_state:
            controller.torque_confirmed()

        if target is None:
            self.publish_targets(self.measured)
        else:
            caps = [MOVE_SPEED_RAD_S] * len(ARM_JOINT_NAMES)
            if isinstance(max_step, (list, tuple)):
                caps = [min(caps[i], float(step) / DT)
                        for i, step in enumerate(max_step)]
            elif max_step is not None:
                caps = [min(cap, float(max_step) / DT) for cap in caps]
            cmd = self.ramp_command(
                target, caps, config.limits, deadband_ok=True)
            self.publish_targets(cmd)

        if controller.phase == controller.PHASE_DONE:
            self._finish_resting_restore()

    def _handle_resting_event(self, config, event: str):
        if event == "torque_off":
            self.torque_intent = False
            self.publish_torque(False)
            self.get_logger().info("Resting: collapsed, torque removed")
        elif event == "torque_on":
            self.torque_intent = True
            self.publish_torque(True)
            self.get_logger().info("Resting: restoring torque")
        elif event == "home":
            self.get_logger().info(
                "Resting: crouch reached, handing off to homing")
            self.active = ACTION_ACTIVE_IDLE
            self.restore_after_homing = True
            self.trigger_action(self.homing, "homing")
        elif event.startswith("abort:"):
            self.get_logger().warning(f"Resting: {event[len('abort:'):]}")

    def _plan_resting_path(self, config):
        controller = self._resting_controller
        phase = controller.phase
        if phase not in (controller.PHASE_CROUCH, controller.PHASE_MOVE_OUT):
            self._resting_path_phase = None
            return
        if self._resting_path_phase == phase:
            return
        self._resting_path_phase = phase
        if controller._path is not None:
            return
        held = list(self.measured)
        target = [config.crouch_target[i] if not self.locked[i]
                  else held[i] for i in range(len(held))]
        path = plan_safe_path(self.model, held, target)
        if path and len(path) > 1:
            controller.set_path(
                path[1:],
                [controller.move_speed * DT] * len(held))

    def _finish_resting_restore(self):
        self.active = ACTION_ACTIVE_IDLE
        self._resting_path_phase = None
        self.torque_intent = True
        self.get_logger().info("Resting restore complete")

    def homing_completed(self):
        """Homing completion hook; True when a resting restore finished."""
        if not self.restore_after_homing:
            return False
        self.restore_after_homing = False
        if not self._resting_controller.homing_finished():
            return False
        self._finish_resting_restore()
        return True

    # ------------------------------------------------------------------
    # Action API shared by every configuration
    # ------------------------------------------------------------------

    def ramp_command(self, target, caps, limits, mask=None,
                     deadband_ok=False, deadband_target=None, brake=True):
        """Step the measured pose toward ``target`` with the shared ramp."""
        count = len(ARM_JOINT_NAMES)
        if mask is None:
            mask = [True] * count
        deltas = []
        effective_caps = []
        for i in range(count):
            if not mask[i]:
                deltas.append(0.0)
                effective_caps.append(0.0)
                continue
            lo, hi = limits[i]
            delta = wrap2pi(target[i] - self.measured[i])
            delta = limit_respecting_diff(
                self.measured[i], delta, lo, hi)
            deltas.append(delta)
            effective_caps.append(caps[i])
        steps, _ = self.ramp.step(deltas, effective_caps, brake=brake)

        command = list(self.measured)
        for i in range(count):
            if not mask[i]:
                continue
            lo, hi = limits[i]
            command[i] = max(
                lo, min(hi, self.measured[i] + steps[i]))

        if not deadband_ok:
            self._apply_deadband(
                command, deadband_target if deadband_target is not None
                else target, mask)
        return command

    def _apply_deadband(self, command, target, mask):
        """Pin the elbow to the target side of the singularity deadband."""
        elbow = JOINT_INDEX["elbow"]
        if not mask[elbow]:
            return
        if abs(target[elbow]) <= ELBOW_DEADBAND:
            return
        if abs(command[elbow]) < ELBOW_DEADBAND:
            command[elbow] = math.copysign(
                ELBOW_DEADBAND, target[elbow])

    def octant(self):
        """FK octant of the measured pose, or None when unavailable."""
        try:
            pos = self.model.solve_forward_kinematics(
                self.measured).position()
        except Exception:
            return None
        return _get_octant(pos)

    def in_zenith(self):
        """True while the zenith holding pose owns the arm.

        Homing preempts a zenith hold; the homing target uses this to
        return to the non-flipped home instead of the side the zenith
        was entered from.
        """
        return self.active == ACTION_ACTIVE_ZENITH

    def unlocked_mask(self):
        return [not locked for locked in self.locked]

    def _masked_max_remaining(self, target, mask):
        remaining = [
            abs(wrap2pi(target[i] - self.measured[i]))
            for i in range(len(ARM_JOINT_NAMES))
            if mask[i]]
        # Every joint can be locked (hardware e-stop, maintenance mode):
        # there is no remaining travel to report.
        return max(remaining) if remaining else 0.0

    # ------------------------------------------------------------------
    # Control cycle
    # ------------------------------------------------------------------

    def _update(self):
        now = time.monotonic()
        if now - self._latch_at >= RESTING_LATCH_REFRESH_SEC:
            self._latch_at = now
            self._publish_latches()

        if self.estop:
            # Freeze: stop publishing targets; the hardware E-stop holds the
            # measured pose.  Status keeps flowing so the master sees us.
            self._publish_status()
            return

        if not self._initialized():
            self._publish_status()
            return

        if self._pending_adopt is not None:
            pending = self._pending_adopt
            self._pending_adopt = None
            self._adopt_startup_pose(
                None if pending == "auto" else pending)

        self._consume_manual_motion()

        config = self._by_active.get(self.active)
        if config is not None:
            if config.engine == "resting":
                self._step_resting(config)
            else:
                self._step_move(config)

        self._publish_status()

    def _consume_manual_motion(self):
        if not self._manual_motion:
            return
        self._manual_motion = False
        config = self._by_active.get(self.active)
        if config is not None and config.cancel_on_manual:
            self._move.goal = None
            if not self._move.pausing and not self._move.exiting:
                self._move.pausing = True
                self.phase = PHASE_FLIP_PAUSING
                self.get_logger().info(
                    f"{config.name} goal cancelled by manual input")
        elif self._move.goal is not None:
            self._move.goal = None
            self._move.path = None
            owner = next(
                (c for c in self._actions if c.resume_on_repeat), None)
            self.get_logger().info(
                f"Remembered {owner.name if owner else 'action'} "
                "goal cleared")

    def _initialized(self):
        return all(not math.isnan(a) for a in self.measured)

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def publish_targets(self, angles):
        payload = tuple(float(wrap2pi(a)) for a in angles)
        if payload == self._last_targets:
            return
        self._last_targets = payload
        for joint, value in zip(ARM_JOINT_NAMES, payload):
            self._target_publishers[joint].publish(Float32(data=value))

    def publish_torque(self, enabled: bool):
        self._torque_publisher.publish(Bool(data=bool(enabled)))

    def _publish_latches(self):
        self._resting_publisher.publish(
            Bool(data=self.active == ACTION_ACTIVE_RESTING))
        self._torque_publisher.publish(
            Bool(data=bool(self.torque_intent)))

    def _publish_status(self):
        self._active_publisher.publish(Int8(data=int(self.active)))
        config = self._by_active.get(self.active)
        msg = ArmActionStatus()
        msg.ready = bool(self._initialized())
        msg.active = int(self.active)
        msg.phase = int(self.phase)
        msg.error_code = int(self._error[0])
        msg.error_arg = int(self._error[1])
        msg.error_seq = int(self._error_seq)
        if config is not None:
            msg.progress = float(self._progress(config))
            if config.report_homing_mask and self._move.mask is not None:
                msg.homing_mask = self._mask_bits(self._move.mask)
        self._status_publisher.publish(msg)

    def _progress(self, config):
        if config.engine == "resting":
            return 0.0
        move = self._move
        if move.target is None or not move.total:
            return 0.0
        remaining = self._masked_max_remaining(move.target, move.mask)
        return _clamp(1.0 - remaining / move.total)

    @staticmethod
    def _mask_bits(mask):
        value = 0
        for index, allowed in enumerate(mask):
            if allowed:
                value |= 1 << index
        return value

    # ------------------------------------------------------------------
    # Hold / stop
    # ------------------------------------------------------------------

    def _abort_action(self, reason: str):
        """Release target ownership and hold the measured pose."""
        if self.active == ACTION_ACTIVE_RESTING:
            # Resting keeps its latches; a hold must not silently restore
            # torque on a collapsed arm.
            return
        self.publish_targets(self.measured)
        self.active = ACTION_ACTIVE_IDLE
        self._move.reset()
        self.restore_after_homing = False
        self.get_logger().info(reason)

    # ------------------------------------------------------------------
    # Errors
    # ------------------------------------------------------------------

    def reject(self, reason: str):
        code, arg = classify_failure(reason)
        self.emit_error(code, arg, reason=reason, force=True)

    def emit_error(self, code: int, arg: int = 0, reason: str = "",
                   force: bool = False):
        code = int(code) & 0xFF
        arg = int(arg) & 0xFF
        if code == ARM_ERROR_NONE:
            if self._error == (ARM_ERROR_NONE, 0):
                return
        elif self._error == (code, arg) and not force:
            return
        self._error = (code, arg)
        self._error_seq = (self._error_seq + 1) & 0xFF
        if code != ARM_ERROR_NONE:
            self.get_logger().warning(
                f"Action driver rejected: {reason}")


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.ALL)
    driver: ArmActionDriverNode | None = None
    try:
        driver = ArmActionDriverNode()
        rclpy.spin(driver)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if driver is not None:
            try:
                driver.destroy_node()
            except Exception:
                pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
