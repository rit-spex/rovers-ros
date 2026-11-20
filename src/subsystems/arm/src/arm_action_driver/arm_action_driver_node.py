"""Arm action driver node.

Owns the discrete, mostly kinematic-free arm actions that used to run inside
``arm_route_planner_node``: homing, side flips, the zenith pose, and the
resting collapse/restore sequence.  It consumes the measured joint angles and
publishes the five joint targets only while an action is active; the planner
suppresses its own targets while ``/ARM/ACTION/ACTIVE`` is nonzero.

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
    ACTION_ACTIVE_FLIPPING,
    ACTION_ACTIVE_HOMING,
    ACTION_ACTIVE_IDLE,
    ACTION_ACTIVE_RESTING,
    ACTION_ACTIVE_TOPIC,
    ACTION_ACTIVE_ZENITH,
    ACTION_COMMAND_TOPIC,
    ACTION_DECEL_RAD_S2,
    ACTION_FLIP_CRUISE_RAD_S,
    ACTION_HOMING_CRUISE_RAD_S,
    ACTION_STATUS_TOPIC,
    ACTION_ZENITH_CRUISE_RAD_S,
    ACTION_ZENITH_EXIT_CRUISE_RAD_S,
    ARM_ENABLED_TOPIC,
    ARM_JOINT_NAMES,
    ARM_RESTING_TOPIC,
    ARM_STATE_TOPICS,
    ARM_TORQUE_ENABLED_TOPIC,
    ARM_TORQUE_STATE_TOPIC,
    DT,
    ELBOW_DEADBAND,
    ESTOP_TOPIC,
    HOMING_HOLD_TIMEOUT_SEC,
    HOMING_TARGET,
    HOMING_THRESHOLD,
    JOINT_CURRENT_TOPICS,
    JOINT_TARGET_TOPICS,
    MECHANICAL_LIMITS,
    OCTANT_DIRECT_FLIP,
    PHASE_FLIP_PAUSING,
    PHASE_FLIP_RUNNING,
    PHASE_ZENITH_EXITING,
    PHASE_ZENITH_HOLDING,
    PICK_AND_PLACE_FLIPPED_LIMITS,
    PICK_AND_PLACE_LIMITS,
    RESTING_CROUCH_TARGET,
    RESTING_FEEDBACK_TIMEOUT_SEC,
    RESTING_LATCH_REFRESH_SEC,
    RESTING_MOVE_ANG_VEL,
    WAYPOINT_TOL,
)
from arm_action_driver.actions.resting import RestingController
from arm_route_planner.kinematics.limiters import RESTING_MODE, _get_octant
from arm_route_planner.kinematics.linkage_models import (
    limit_respecting_diff, wrap2pi,
)
from arm_route_planner.kinematics.path_planner import (
    build_arm_solver_object, plan_safe_path,
)
from arm_route_planner.kinematics.regions import (
    is_resting_pose, is_zenith_pose,
)
from arm_action_driver.stepping import JointRamp
from rover_protocol.arm_errors import (
    ARM_ERROR_ESTOP,
    ARM_ERROR_NONE,
    classify_failure,
)

JOINT_INDEX = {
    "base": 0,
    "shoulder": 1,
    "elbow": 2,
    "wrist_bend": 3,
    "wrist_twist": 4,
}

BASE = 0
SHOULDER = 1
ELBOW = 2
WRIST_BEND = 3
WRIST_TWIST = 4

MANUAL_MOTION_EPS = 1e-3
ARRIVE_TOL = 1e-3


class ArmActionDriverNode(Node):
    """Drive the arm's discrete actions and publish their joint targets."""

    def __init__(self):
        super().__init__("arm_action_driver_node")

        # FK model for octant gates and safe joint-space paths.  No IK
        # solving happens here.
        self.__model, _ = build_arm_solver_object()

        self.__measured = [float("nan")] * len(ARM_JOINT_NAMES)
        self.__measured_at = [0.0] * len(ARM_JOINT_NAMES)

        self.__estop = False
        self.__enabled = False
        self.__planar = False
        self.__flipped = False
        self.__adopted = False
        self.__pending_adopt = None
        self.__locked = [False] * len(ARM_JOINT_NAMES)
        self.__torque_state = False

        # Action state.  ``active`` is the published enum; ``phase`` is the
        # per-action substate carried in ArmActionStatus.
        self.__active = ACTION_ACTIVE_IDLE
        self.__phase = PHASE_FLIP_RUNNING

        # Shared stepping profile.
        self.__ramp = JointRamp(
            len(ARM_JOINT_NAMES), ACTION_ACCEL_RAD_S2,
            ACTION_DECEL_RAD_S2, DT)

        # Flip state.  A paused flip is IDLE with a remembered goal.
        self.__flip_goal = None
        self.__flip_path = None
        self.__flip_idx = 0
        self.__flip_move_total = None

        # Homing state.
        self.__homing_target = None
        self.__homing_mask = None
        self.__homing_path = None
        self.__homing_idx = 0
        self.__homing_command_at = 0.0
        self.__restore_after_homing = False

        # Zenith state.
        self.__zenith_target = None
        self.__zenith_exit_target = None
        self.__zenith_side = False

        # Resting state.
        self.__resting = RestingController()
        self.__resting_path_phase = None
        self.__torque_intent = True
        self.__resting_hold = False

        # Manual (SpaceMouse) motion cancels a remembered flip goal.
        self.__manual_motion = False

        # Error event bookkeeping for ArmActionStatus.
        self.__error = (ARM_ERROR_NONE, 0)
        self.__error_seq = 0

        self.__last_targets = None
        self.__latch_at = 0.0

        self.__create_interfaces()
        self.create_timer(DT, self.__update)
        self.get_logger().info("Arm action driver started")

    # ------------------------------------------------------------------
    # ROS interfaces
    # ------------------------------------------------------------------

    def __create_interfaces(self):
        self.__target_publishers = {
            joint: self.create_publisher(
                Float32, JOINT_TARGET_TOPICS[joint], 10)
            for joint in ARM_JOINT_NAMES
        }
        self.__active_publisher = self.create_publisher(
            Int8, ACTION_ACTIVE_TOPIC, 10)
        self.__status_publisher = self.create_publisher(
            ArmActionStatus, ACTION_STATUS_TOPIC, 10)
        self.__resting_publisher = self.create_publisher(
            Bool, ARM_RESTING_TOPIC, 10)
        self.__torque_publisher = self.create_publisher(
            Bool, ARM_TORQUE_ENABLED_TOPIC, 10)

        self.create_subscription(
            ArmCommand, ACTION_COMMAND_TOPIC, self.__on_command, 10)
        self.create_subscription(
            Velocity, "/ARM/MANUAL/CMD_VELOCITY",
            self.__on_manual_velocity, 10)
        self.create_subscription(
            Bool, ESTOP_TOPIC, self.__on_estop, 10)
        self.create_subscription(
            Bool, ARM_ENABLED_TOPIC, self.__on_enabled, 10)
        self.create_subscription(
            Bool, ARM_STATE_TOPICS["flags"]["PLANAR"],
            self.__on_planar, 10)
        self.create_subscription(
            Bool, ARM_STATE_TOPICS["flags"]["FLIPPED"],
            self.__on_flipped, 10)
        self.create_subscription(
            UInt8, ARM_STATE_TOPICS["locked_joints"],
            self.__on_locked, 10)
        self.create_subscription(
            Bool, ARM_TORQUE_STATE_TOPIC, self.__on_torque_state, 10)
        for joint in ARM_JOINT_NAMES:
            self.create_subscription(
                Float32,
                JOINT_CURRENT_TOPICS[joint],
                self.__on_measured(joint),
                10,
            )

    # ------------------------------------------------------------------
    # Input callbacks
    # ------------------------------------------------------------------

    def __on_measured(self, joint: str):
        index = JOINT_INDEX[joint]

        def callback(msg: Float32):
            self.__measured[index] = wrap2pi(msg.data)
            self.__measured_at[index] = time.monotonic()
        return callback

    def __on_estop(self, msg: Bool):
        self.__estop = bool(msg.data)
        if not self.__estop:
            # Resume from rest instead of jumping back into the old speed.
            self.__ramp.reset()

    def __on_enabled(self, msg: Bool):
        self.__enabled = bool(msg.data)

    def __on_planar(self, msg: Bool):
        self.__planar = bool(msg.data)

    def __on_flipped(self, msg: Bool):
        self.__flipped = bool(msg.data)

    def __on_locked(self, msg: UInt8):
        self.__locked = [bool(msg.data & (1 << i))
                         for i in range(len(ARM_JOINT_NAMES))]

    def __on_torque_state(self, msg: Bool):
        self.__torque_state = bool(msg.data)

    def __on_manual_velocity(self, msg: Velocity):
        values = (
            msg.linear_x, msg.linear_y, msg.linear_z,
            msg.angular_x, msg.angular_y, msg.angular_z,
        )
        if any(abs(float(v)) > MANUAL_MOTION_EPS for v in values):
            self.__manual_motion = True

    def __on_command(self, msg: ArmCommand):
        command = msg.command.strip()
        if not command:
            return
        self.__handle_command(command)

    # ------------------------------------------------------------------
    # Command handling
    # ------------------------------------------------------------------

    def __handle_command(self, command: str):
        if command == "adopt" or command.startswith("adopt:"):
            mode = command.partition(":")[2] or None
            if not self.__initialized():
                # The driver may still be compiling its model; apply the
                # planner's inference as soon as feedback is complete.
                self.__pending_adopt = mode or "auto"
                return
            self.__adopt_startup_pose(mode)
            return
        if not self.__initialized():
            self.__reject(f"Action command '{command}' ignored: "
                          "arm not initialized")
            return
        if self.__estop:
            self.__emit_error(
                ARM_ERROR_ESTOP,
                reason=f"Action command '{command}' ignored: e-stop active")
            return

        if command in ("hold", "stop"):
            self.__abort_action("Action held by command")
            return
        if command == "flip":
            self.__start_flip()
            return
        if command == "flip_release":
            self.__release_flip()
            return
        if command in ("homing", "home"):
            self.__start_homing()
            return
        if command in ("zenith", "zenith_mode", "toggle_zenith"):
            self.__toggle_zenith()
            return
        if command in ("toggle_resting", "rest", "resting"):
            self.__toggle_resting()
            return
        if command.startswith("home_joint:"):
            try:
                index = int(command.partition(":")[2])
            except ValueError:
                self.__reject(f"Unknown action command '{command}'")
                return
            if not 0 <= index < len(ARM_JOINT_NAMES):
                self.__reject(f"Unknown action command '{command}'")
                return
            self.__start_homing(index=index)
            return

        self.__reject(f"Unknown action command '{command}'")

    # ------------------------------------------------------------------
    # Startup adoption
    # ------------------------------------------------------------------

    def __adopt_startup_pose(self, mode=None):
        """Adopt an inferred zenith/resting pose.  Runs at most once.

        ``mode`` is the planner's inferred mode from ``adopt:<mode>``.  With
        no mode it falls back to classifying the measured pose, but only
        when the planner has not already chosen a normal/planar start.
        """
        if self.__adopted:
            return
        if self.__active != ACTION_ACTIVE_IDLE:
            self.__adopted = True
            return
        self.__adopted = True
        if mode == "resting" or (mode is None and is_resting_pose(
                self.__measured)):
            self.__resting.begin_collapse()
            self.__resting.abort("Resting mode inferred from startup pose")
            self.__torque_intent = False
            self.__publish_torque(False)
            self.__resting_hold = True
            self.__resting_path_phase = None
            self.__active = ACTION_ACTIVE_RESTING
            self.__ramp.reset()
            self.get_logger().info(
                "Resting inferred from startup pose; torque stays off")
            return
        if mode == "zenith" or (mode is None and self.__zenith_pose()):
            self.__zenith_side = self.__flipped
            self.__zenith_target = [
                self.__measured[BASE],
                math.pi / 2,
                0.0,
                0.0,
                self.__measured[WRIST_TWIST],
            ]
            self.__zenith_exit_target = None
            self.__active = ACTION_ACTIVE_ZENITH
            self.__phase = PHASE_ZENITH_HOLDING
            self.__ramp.reset()
            self.get_logger().info("Zenith inferred from startup pose")

    def __zenith_pose(self):
        return is_zenith_pose(self.__measured)

    # ------------------------------------------------------------------
    # Flip
    # ------------------------------------------------------------------

    def __start_flip(self):
        if self.__active == ACTION_ACTIVE_FLIPPING:
            return
        if self.__active != ACTION_ACTIVE_IDLE:
            self.__reject("Flip blocked: another action is running")
            return

        start = list(self.__measured)
        mask = [not locked for locked in self.__locked]
        resumed = self.__flip_goal is not None

        if not resumed:
            if not self.__flip_allowed():
                return
            goal = self.__side_flip_target(start)
        else:
            # Resume keeps the original goal; the path is re-planned from
            # wherever the arm is now.
            goal = list(self.__flip_goal)

        goal = [goal[i] if mask[i] else start[i]
                for i in range(len(goal))]

        if any(self.__locked[:3]):
            path = [start, goal]
        else:
            path = plan_safe_path(self.__model, start, goal)
        if path is None:
            self.__reject(
                "Flip blocked: no safe path through restricted space")
            return

        self.__flip_goal = goal
        self.__flip_path = [list(p) for p in path[1:]] or [goal]
        self.__flip_idx = 0
        self.__flip_move_total = self.__max_remaining(self.__flip_path[0])
        self.__active = ACTION_ACTIVE_FLIPPING
        self.__phase = PHASE_FLIP_RUNNING
        self.__ramp.reset()
        self.get_logger().info(
            "Flip resumed" if resumed else "Flip started")

    def __flip_allowed(self):
        octant = self.__octant()
        if octant is not None and octant not in OCTANT_DIRECT_FLIP:
            if not self.__planar:
                self.__reject(
                    f"Flip blocked: octant {octant} requires planar")
                return False
        return True

    def __side_flip_target(self, angles):
        target = [
            angles[BASE],
            math.pi - angles[SHOULDER],
            -angles[ELBOW],
            angles[WRIST_BEND],
            angles[WRIST_TWIST],
        ]
        return [max(lo, min(hi, wrap2pi(a)))
                for a, (lo, hi) in zip(target, MECHANICAL_LIMITS)]

    def __release_flip(self):
        if self.__active == ACTION_ACTIVE_FLIPPING \
                and self.__phase == PHASE_FLIP_RUNNING:
            self.__phase = PHASE_FLIP_PAUSING
        # Releasing while idle keeps any remembered goal for the next press.

    def __step_flip(self):
        if self.__phase == PHASE_FLIP_PAUSING:
            target = self.__flip_path[self.__flip_idx] \
                if self.__flip_path else self.__measured
            cmd = self.__ramp_command(
                target, [0.0] * len(ARM_JOINT_NAMES), MECHANICAL_LIMITS,
                mask=self.__unlocked_mask())
            self.__publish_targets(cmd)
            if self.__ramp.is_stopped():
                self.__publish_targets(self.__measured)
                self.__active = ACTION_ACTIVE_IDLE
                self.__phase = PHASE_FLIP_RUNNING
                if self.__flip_goal is None:
                    self.__flip_path = None
                self.get_logger().info("Flip paused")
            return

        if not self.__flip_path:
            self.__finish_flip()
            return

        target = self.__flip_path[self.__flip_idx]
        cmd = self.__ramp_command(
            target, [ACTION_FLIP_CRUISE_RAD_S] * len(ARM_JOINT_NAMES),
            MECHANICAL_LIMITS, mask=self.__unlocked_mask())
        self.__publish_targets(cmd)

        mask = self.__unlocked_mask()
        if all(not mask[i] or abs(wrap2pi(target[i] - self.__measured[i]))
               <= WAYPOINT_TOL for i in range(len(target))):
            if self.__flip_idx + 1 < len(self.__flip_path):
                self.__flip_idx += 1
                self.__flip_move_total = self.__max_remaining(
                    self.__flip_path[self.__flip_idx])
            else:
                self.__finish_flip()

    def __finish_flip(self):
        self.__publish_targets(self.__measured)
        self.__active = ACTION_ACTIVE_IDLE
        self.__phase = PHASE_FLIP_RUNNING
        self.__flip_goal = None
        self.__flip_path = None
        self.__flip_idx = 0
        self.get_logger().info("Flip complete")

    # ------------------------------------------------------------------
    # Homing
    # ------------------------------------------------------------------

    def __start_homing(self, index=None):
        if self.__active == ACTION_ACTIVE_FLIPPING:
            self.__reject("Homing blocked: a flip is running")
            return
        if self.__active == ACTION_ACTIVE_RESTING:
            self.__reject("Homing blocked: resting is active")
            return
        # From zenith, skip the exit move and home directly.
        self.__zenith_exit_target = None
        self.__zenith_target = None

        target = self.__home_target()
        mask = self.__unlocked_mask()
        if index is not None:
            mask = [i == index for i in range(len(mask))]

        start = list(self.__measured)
        goal = [target[i] if mask[i] else start[i]
                for i in range(len(target))]
        if any(self.__locked[:3]) or (index is not None and index < 3):
            path = [start, goal]
        else:
            path = plan_safe_path(self.__model, start, goal)
            if path is None:
                # Refuse to lose homing over a planner miss; the ramp and
                # mechanical limits still bound the direct move.
                path = [start, goal]

        self.__homing_target = target
        self.__homing_mask = mask
        self.__homing_path = [list(p) for p in path[1:]] or [goal]
        self.__homing_idx = 0
        self.__homing_command_at = time.monotonic()
        self.__active = ACTION_ACTIVE_HOMING
        self.__ramp.reset()
        self.get_logger().info(
            "Homing started" if index is None
            else f"Homing joint {index}")

    def __home_target(self):
        if self.__flipped:
            return [
                HOMING_TARGET[BASE],
                math.pi - HOMING_TARGET[SHOULDER],
                -HOMING_TARGET[ELBOW],
                HOMING_TARGET[WRIST_BEND],
                HOMING_TARGET[WRIST_TWIST],
            ]
        return list(HOMING_TARGET)

    def __step_homing(self):
        now = time.monotonic()
        if now - self.__homing_command_at > HOMING_HOLD_TIMEOUT_SEC:
            self.__stop_homing()
            return
        mask = self.__homing_mask
        target = self.__homing_path[self.__homing_idx]
        cmd = self.__ramp_command(
            target, [ACTION_HOMING_CRUISE_RAD_S] * len(ARM_JOINT_NAMES),
            MECHANICAL_LIMITS, mask=mask,
            deadband_target=self.__homing_target)
        self.__publish_targets(cmd)

        if all(not mask[i] or abs(wrap2pi(target[i] - self.__measured[i]))
               <= WAYPOINT_TOL for i in range(len(target))):
            if self.__homing_idx + 1 < len(self.__homing_path):
                self.__homing_idx += 1
            elif all(not mask[i]
                     or abs(wrap2pi(self.__homing_target[i]
                                    - self.__measured[i]))
                     <= HOMING_THRESHOLD
                     for i in range(len(self.__homing_target))):
                self.__finish_homing()

    def __stop_homing(self):
        self.__publish_targets(self.__measured)
        self.__homing_target = None
        self.__homing_mask = None
        self.__homing_path = None
        self.__active = ACTION_ACTIVE_IDLE
        self.get_logger().info("Homing released; stopped")

    def __finish_homing(self):
        self.__publish_targets(self.__measured)
        self.__homing_target = None
        self.__homing_mask = None
        self.__homing_path = None
        if self.__restore_after_homing:
            self.__restore_after_homing = False
            if self.__resting.homing_finished():
                self.__finish_resting()
            else:
                self.__active = ACTION_ACTIVE_IDLE
        else:
            self.__active = ACTION_ACTIVE_IDLE
        self.get_logger().info("Homing complete")

    # ------------------------------------------------------------------
    # Zenith
    # ------------------------------------------------------------------

    def __toggle_zenith(self):
        if self.__active == ACTION_ACTIVE_ZENITH:
            if self.__phase == PHASE_ZENITH_HOLDING:
                self.__start_zenith_exit()
            return
        if self.__active != ACTION_ACTIVE_IDLE:
            self.__reject("Zenith blocked: another action is running")
            return

        self.__zenith_side = self.__flipped
        self.__zenith_target = [
            self.__measured[BASE],
            math.pi / 2,
            0.0,
            0.0,
            self.__measured[WRIST_TWIST],
        ]
        self.__zenith_exit_target = None
        self.__active = ACTION_ACTIVE_ZENITH
        self.__phase = PHASE_ZENITH_HOLDING
        self.__ramp.reset()
        self.get_logger().info("Zenith entered")

    def __start_zenith_exit(self):
        angles = list(self.__measured)
        if self.__zenith_side:
            _, shoulder_hi = PICK_AND_PLACE_FLIPPED_LIMITS[SHOULDER]
            elbow = max(angles[ELBOW], ELBOW_DEADBAND)
        else:
            _, shoulder_hi = PICK_AND_PLACE_LIMITS[SHOULDER]
            elbow = min(angles[ELBOW], -ELBOW_DEADBAND)
        shoulder_lo = PICK_AND_PLACE_FLIPPED_LIMITS[SHOULDER][0] \
            if self.__zenith_side else PICK_AND_PLACE_LIMITS[SHOULDER][0]
        wrist_lo, wrist_hi = MECHANICAL_LIMITS[WRIST_BEND]
        twist_lo, twist_hi = MECHANICAL_LIMITS[WRIST_TWIST]
        self.__zenith_exit_target = [
            angles[BASE],
            max(shoulder_lo, min(shoulder_hi, angles[SHOULDER])),
            elbow,
            max(wrist_lo, min(wrist_hi, angles[WRIST_BEND])),
            max(twist_lo, min(twist_hi, angles[WRIST_TWIST])),
        ]
        self.__phase = PHASE_ZENITH_EXITING
        self.__ramp.reset()
        self.get_logger().info("Zenith exit started")

    def __step_zenith(self):
        exiting = self.__phase == PHASE_ZENITH_EXITING
        target = self.__zenith_exit_target if exiting \
            else self.__zenith_target
        cruise = ACTION_ZENITH_EXIT_CRUISE_RAD_S if exiting \
            else ACTION_ZENITH_CRUISE_RAD_S
        cmd = self.__ramp_command(
            target, [cruise] * len(ARM_JOINT_NAMES),
            MECHANICAL_LIMITS, deadband_ok=not exiting)
        self.__publish_targets(cmd)

        if exiting and all(
                abs(wrap2pi(target[i] - self.__measured[i])) <= ARRIVE_TOL
                for i in range(len(target))):
            self.__active = ACTION_ACTIVE_IDLE
            self.__phase = PHASE_ZENITH_HOLDING
            self.__zenith_target = None
            self.__zenith_exit_target = None
            self.get_logger().info("Zenith exit complete")

    # ------------------------------------------------------------------
    # Resting
    # ------------------------------------------------------------------

    def __toggle_resting(self):
        if self.__active == ACTION_ACTIVE_RESTING:
            if self.__resting.phase in (
                    self.__resting.PHASE_COLLAPSED,
                    self.__resting.PHASE_HOLD):
                if self.__resting.request_restore():
                    self.__torque_intent = True
                    self.__publish_torque(True)
                    self.get_logger().info("Resting restore started")
            elif self.__resting.active:
                self.__resting.abort("Resting sequence aborted by operator")
                self.get_logger().warning("Resting sequence aborted")
            return

        if self.__active != ACTION_ACTIVE_IDLE:
            self.__reject("Resting blocked: another action is running")
            return

        inferred = is_resting_pose(self.__measured)
        if not self.__planar and not inferred:
            self.__reject(
                "Resting blocked: planar mode is required")
            return

        if not self.__resting.begin_collapse():
            self.__reject("Resting blocked: sequence already running")
            return

        if inferred:
            # Startup pose: stay de-torqued and wait for the restore.
            self.__resting.abort("Resting mode inferred from startup pose")
            self.__torque_intent = False
            self.__publish_torque(False)
            self.__resting_hold = True
            self.get_logger().info(
                "Resting inferred from startup pose; torque stays off")
        else:
            self.__torque_intent = True
            self.__resting_hold = False
            self.get_logger().info("Resting collapse started")

        self.__resting_path_phase = None
        self.__active = ACTION_ACTIVE_RESTING
        self.__ramp.reset()

    def __step_resting(self, now):
        fresh = all(now - at <= RESTING_FEEDBACK_TIMEOUT_SEC
                    for at in self.__measured_at)
        self.__plan_resting_path()
        target, max_step, events = self.__resting.update(
            self.__measured, DT, feedback_fresh=fresh,
            enabled=self.__enabled)
        for event in events:
            self.__handle_resting_event(event)

        if self.__resting.phase == self.__resting.PHASE_TORQUE_ON \
                and self.__torque_state:
            self.__resting.torque_confirmed()

        if target is None:
            self.__publish_targets(self.__measured)
        else:
            caps = [RESTING_MOVE_ANG_VEL] * len(ARM_JOINT_NAMES)
            if isinstance(max_step, (list, tuple)):
                caps = [float(step) / DT for step in max_step]
            elif max_step is not None:
                caps = [float(max_step) / DT] * len(ARM_JOINT_NAMES)
            cmd = self.__ramp_command(
                target, caps, RESTING_MODE.limits, deadband_ok=True)
            self.__publish_targets(cmd)

        if self.__resting.phase == self.__resting.PHASE_DONE:
            self.__finish_resting()

    def __handle_resting_event(self, event: str):
        if event == "torque_off":
            self.__torque_intent = False
            self.__publish_torque(False)
            self.get_logger().info("Resting: collapsed, torque removed")
        elif event == "torque_on":
            self.__torque_intent = True
            self.__publish_torque(True)
            self.get_logger().info("Resting: restoring torque")
        elif event == "home":
            self.get_logger().info(
                "Resting: crouch reached, handing off to homing")
            self.__active = ACTION_ACTIVE_IDLE
            self.__restore_after_homing = True
            self.__start_homing()
        elif event.startswith("abort:"):
            self.get_logger().warning(f"Resting: {event[len('abort:'):]}")

    def __finish_resting(self):
        self.__active = ACTION_ACTIVE_IDLE
        self.__resting_path_phase = None
        self.__resting_hold = False
        self.__torque_intent = True
        self.get_logger().info("Resting restore complete")

    def __plan_resting_path(self):
        phase = self.__resting.phase
        if phase not in (self.__resting.PHASE_CROUCH,
                         self.__resting.PHASE_MOVE_OUT):
            self.__resting_path_phase = None
            return
        if self.__resting_path_phase == phase:
            return
        self.__resting_path_phase = phase
        if self.__resting._path is not None:
            return
        held = list(self.__measured)
        target = [RESTING_CROUCH_TARGET[i] if not self.__locked[i]
                  else held[i] for i in range(len(held))]
        path = plan_safe_path(self.__model, held, target)
        if path and len(path) > 1:
            self.__resting.set_path(
                path[1:], [RESTING_MOVE_ANG_VEL * DT] * len(held))

    # ------------------------------------------------------------------
    # Shared stepping helpers
    # ------------------------------------------------------------------

    def __ramp_command(self, target, caps, limits, mask=None,
                       deadband_ok=False, deadband_target=None):
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
            delta = wrap2pi(target[i] - self.__measured[i])
            delta = limit_respecting_diff(
                self.__measured[i], delta, lo, hi)
            deltas.append(delta)
            effective_caps.append(caps[i])
        steps, _ = self.__ramp.step(deltas, effective_caps)

        command = list(self.__measured)
        for i in range(count):
            if not mask[i]:
                continue
            lo, hi = limits[i]
            command[i] = max(
                lo, min(hi, self.__measured[i] + steps[i]))

        if not deadband_ok:
            self.__apply_deadband(
                command, deadband_target if deadband_target is not None
                else target, mask)
        return command

    def __apply_deadband(self, command, target, mask):
        """Pin the elbow to the target side of the singularity deadband."""
        if not mask[ELBOW]:
            return
        if abs(target[ELBOW]) <= ELBOW_DEADBAND:
            return
        if abs(command[ELBOW]) < ELBOW_DEADBAND:
            command[ELBOW] = math.copysign(
                ELBOW_DEADBAND, target[ELBOW])

    def __abort_action(self, reason: str):
        """Release target ownership and hold the measured pose."""
        if self.__active == ACTION_ACTIVE_RESTING:
            # Resting keeps its latches; a hold must not silently restore
            # torque on a collapsed arm.
            return
        self.__publish_targets(self.__measured)
        self.__active = ACTION_ACTIVE_IDLE
        self.__flip_goal = None
        self.__flip_path = None
        self.__homing_target = None
        self.__homing_mask = None
        self.__homing_path = None
        self.__zenith_target = None
        self.__zenith_exit_target = None
        self.__restore_after_homing = False
        self.get_logger().info(reason)

    def __unlocked_mask(self):
        return [not locked for locked in self.__locked]

    def __max_remaining(self, target):
        return max(
            abs(wrap2pi(target[i] - self.__measured[i]))
            for i in range(len(ARM_JOINT_NAMES)))

    def __octant(self):
        try:
            pos = self.__model.solve_forward_kinematics(
                self.__measured).position()
        except Exception:
            return None
        return _get_octant(pos)

    # ------------------------------------------------------------------
    # Control cycle
    # ------------------------------------------------------------------

    def __update(self):
        now = time.monotonic()
        if now - self.__latch_at >= RESTING_LATCH_REFRESH_SEC:
            self.__latch_at = now
            self.__publish_latches()

        if self.__estop:
            # Freeze: stop publishing targets; the hardware E-stop holds the
            # measured pose.  Status keeps flowing so the master sees us.
            self.__publish_status()
            return

        if not self.__initialized():
            self.__publish_status()
            return

        if self.__pending_adopt is not None:
            pending = self.__pending_adopt
            self.__pending_adopt = None
            self.__adopt_startup_pose(
                None if pending == "auto" else pending)

        self.__consume_manual_motion()

        if self.__active == ACTION_ACTIVE_HOMING:
            self.__step_homing()
        elif self.__active == ACTION_ACTIVE_FLIPPING:
            self.__step_flip()
        elif self.__active == ACTION_ACTIVE_ZENITH:
            self.__step_zenith()
        elif self.__active == ACTION_ACTIVE_RESTING:
            self.__step_resting(now)

        self.__publish_status()

    def __consume_manual_motion(self):
        if not self.__manual_motion:
            return
        self.__manual_motion = False
        if self.__active == ACTION_ACTIVE_FLIPPING:
            self.__flip_goal = None
            if self.__phase == PHASE_FLIP_RUNNING:
                self.__phase = PHASE_FLIP_PAUSING
                self.get_logger().info(
                    "Flip goal cancelled by manual input")
        elif self.__flip_goal is not None:
            self.__flip_goal = None
            self.__flip_path = None
            self.get_logger().info("Remembered flip goal cleared")

    def __initialized(self):
        return all(not math.isnan(a) for a in self.__measured)

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def __publish_targets(self, angles):
        payload = tuple(float(wrap2pi(a)) for a in angles)
        if payload == self.__last_targets:
            return
        self.__last_targets = payload
        for joint, value in zip(ARM_JOINT_NAMES, payload):
            self.__target_publishers[joint].publish(Float32(data=value))

    def __publish_torque(self, enabled: bool):
        self.__torque_publisher.publish(Bool(data=bool(enabled)))

    def __publish_latches(self):
        self.__resting_publisher.publish(
            Bool(data=self.__active == ACTION_ACTIVE_RESTING))
        self.__torque_publisher.publish(
            Bool(data=bool(self.__torque_intent)))

    def __publish_status(self):
        self.__active_publisher.publish(Int8(data=int(self.__active)))
        msg = ArmActionStatus()
        msg.ready = bool(self.__initialized())
        msg.active = int(self.__active)
        msg.phase = int(self.__phase)
        msg.error_code = int(self.__error[0])
        msg.error_arg = int(self.__error[1])
        msg.error_seq = int(self.__error_seq)
        msg.progress = float(self.__progress())
        msg.homing_mask = int(self.__homing_mask_value())
        self.__status_publisher.publish(msg)

    def __progress(self):
        if self.__active == ACTION_ACTIVE_FLIPPING \
                and self.__flip_move_total:
            remaining = self.__max_remaining(
                self.__flip_path[self.__flip_idx])
            return _clamp(1.0 - remaining / self.__flip_move_total)
        if self.__active == ACTION_ACTIVE_HOMING \
                and self.__homing_target is not None:
            remaining = max(
                abs(wrap2pi(self.__homing_target[i] - self.__measured[i]))
                for i in range(len(ARM_JOINT_NAMES))
                if self.__homing_mask[i])
            return _clamp(1.0 - remaining / math.pi)
        return 0.0

    def __homing_mask_value(self):
        if self.__active != ACTION_ACTIVE_HOMING or self.__homing_mask is None:
            return 0
        value = 0
        for index, allowed in enumerate(self.__homing_mask):
            if allowed:
                value |= 1 << index
        return value

    # ------------------------------------------------------------------
    # Errors
    # ------------------------------------------------------------------

    def __reject(self, reason: str):
        code, arg = classify_failure(reason)
        self.__emit_error(code, arg, reason=reason, force=True)

    def __emit_error(self, code: int, arg: int = 0, reason: str = "",
                     force: bool = False):
        code = int(code) & 0xFF
        arg = int(arg) & 0xFF
        if code == ARM_ERROR_NONE:
            if self.__error == (ARM_ERROR_NONE, 0):
                return
        elif self.__error == (code, arg) and not force:
            return
        self.__error = (code, arg)
        self.__error_seq = (self.__error_seq + 1) & 0xFF
        if code != ARM_ERROR_NONE:
            self.get_logger().warning(
                f"Action driver rejected: {reason}")


def _clamp(value):
    return max(0.0, min(1.0, float(value)))


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
