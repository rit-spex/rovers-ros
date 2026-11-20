# Arm Resting Feature — Implementation Plan

Status: planned, not implemented.

## 1. Goal

Add a two-way **RESTING** operation, triggered by the basestation **R** key.

First press:

1. Slowly move the arm to the crouch pose `[0, 0, 135, 0, 0]` (degrees).
2. Drive the shoulder slowly toward its physical lower stop.
3. When the shoulder stalls, back it off 0.5 degrees.
4. Drive the elbow slowly toward its physical upper stop.
5. When the elbow stalls, back it off 0.5 degrees.
6. Remove torque from the three OpenCAN motors (base, shoulder, elbow).
7. Report operation mode `RESTING`.

Second press:

1. Re-enable torque on the three OpenCAN motors.
2. Slowly move back to `[0, 0, 135, 0, 0]`.
3. Leave `RESTING` at the crouch and automatically home the arm to
   `[0, 45, -45, 0, 0]`.
4. End in the normal Pick-and-Place mode.

R during a running sequence aborts it: hold the measured pose, keep torque on,
and remain in `RESTING`. The next R starts the restore sequence.

Restore is the only exit from `RESTING`; an abort leaves the arm in `RESTING`.

While `RESTING` is active, an E-stop removes OpenCAN torque and keeps it off
after the E-stop clears; only the explicit restore re-enables torque. E-stop
behaviour outside `RESTING` is unchanged.

## 2. Decisions (locked)

| Topic | Decision |
| --- | --- |
| Zenith | Kept. Both ZENITH (Z) and RESTING (R) exist. |
| Normal limits | `MECHANICAL_LIMITS` are **not** changed. |
| Resting limits | Shoulder/elbow may exceed normal limits by up to **30°**, hard-clamped. Base/wrists unchanged. |
| Push mechanism | Position mode only. Step the position target; do not use OpenCAN velocity mode. |
| Push speed | **1°/s** for both shoulder and elbow (0.05° per 50 ms control cycle). |
| Push target | 5° past the expected stop: shoulder `-15°`, elbow `+165°` (equals the padded limit, so a missing stop aborts immediately). |
| Stall detection | Windowed position check is primary; the rolling velocity average is advisory only (see §7). |
| Backoff | 0.5° after the stall; wait for the backoff move to finish before the next joint. |
| Torque off | Controlword 0 on base/shoulder/elbow via the existing `set_enabled(False)` path (same as E-stop), latched. |
| E-stop | While RESTING is active: forces torque off and keeps it off until restore. Outside RESTING: unchanged. |
| Restore end | Leave RESTING at `[0,0,135,0,0]`, then auto-home. |
| R mid-sequence | Abort and hold, torque stays on, remain RESTING. |
| Command gating | R is rejected during ZENITH; all other mode/motion commands and teleop are rejected while RESTING is active. |
| Master mux | `toggle_resting` and `toggle_zenith` bypass the MANUAL-only command gate; the planner stays the authority. |
| Joint locks | Released on entering RESTING (zenith pattern), restored at `DONE`. |
| Protocol | `lock_all_joints` is basestation-only. Wire order ends `... solenoid_strike, rest, zenith`. |

## 3. Existing-state conflicts to resolve first

1. **Protocol mismatch (half-resolved).** The rover's `lib/rovers-protocol`
   `arm_state` already ends `solenoid_strike, rest, zenith` and the submodule
   is clean. The external basestation app copy still ends
   `lock_all_joints, rest`. The rover decoder reads by position, so today the
   external `lock_all_joints` byte is decoded as `rest` and its `rest` byte
   as `zenith`. Sync the external copy to `rest, zenith`, drop
   `lock_all_joints` from the wire, and rebuild the basestation firmware.
2. **Zenith is wired rover-side, but R is not.** Mode 4, flag bit 7, the
   `ARM_STATE_COMMANDS["zenith"]` mapping, and the planner command dispatch
   all exist. The `rest` signal exists in `protocol.yaml` but has **no**
   entry in `ARM_STATE_COMMANDS`, so R is currently a no-op. Add
   `"rest": "toggle_resting"`.
3. **Limits.** Normal mechanical limits are shoulder `2..178°`, elbow
   `-135..135°`. The requested stalls (`-10°`, `+160°`) are outside them by
   12° and 25° respectively, both within the agreed 30° allowance.
4. **Hardware validation is placeholder-driven.** `angle_limits()` is derived
   from `ARM_MOTOR_PARAMS`, which the constants file marks as incorrect. It
   must not gate resting pushes; use `RESTING_LIMITS` while resting. The
   provisional tick limits are also *tighter* than the resting region in
   places (shoulder `0..180.1°`, elbow `-150.9..150.8°`), so the resting
   acceptance must explicitly override them, not just widen them.
5. **Enable re-assert.** `/ARM/STATE/ENABLED` is republished continuously, and
   `arm_hardware_node` re-sends enable frames on every message. A torque-off
   latch is required or the drives will be re-enabled immediately.
6. **Submodule state.** The rover copy of `lib/rovers-protocol` is already
   committed with `rest, zenith`. Only the external basestation app copy and
   the firmware rebuild remain.

## 4. Architecture

- `arm_route_planner_node` / `RoverArmController` owns the RESTING state
  machine, mode reporting, slow target stepping, and stall decisions.
- `arm_hardware_node` owns CAN primitives: velocity feedback, resting target
  acceptance, and the torque latch.
- `arm_sim_node` implements the same primitives against Gazebo.
- `arm_master_node` gets one change: `toggle_resting` (and `toggle_zenith`)
  bypass the MANUAL-only command gate so the toggles reach the planner under
  any control source.
- No new motion path: the planner keeps publishing the normal
  `/ARM/JOINT/TARGET_*` topics. RESTING only changes which limits are used,
  which hardware latches are active, and which commands the planner accepts:
  teleop and autonomous target inputs are ignored while resting, the same
  ownership model zenith uses.

### New topics

| Topic | Type | Direction | Purpose |
| --- | --- | --- | --- |
| `/ARM/HARDWARE/RESTING` | `std_msgs/Bool` | planner → hardware | Widen shoulder/elbow target limits to `RESTING_LIMITS` and poll velocity. |
| `/ARM/HARDWARE/TORQUE_ENABLED` | `std_msgs/Bool` | planner → hardware | Enable/disable OpenCAN torque. |
| `/ARM/STATE/TORQUE_ENABLED` | `std_msgs/Bool` | hardware → diagnostics | Actual torque latch state. |
| `/ARM/JOINT/CURRENT_VELOCITY_BASE` | `std_msgs/Float32` | hardware → planner | Drive velocity units (optional; shoulder/elbow are required). |
| `/ARM/JOINT/CURRENT_VELOCITY_SHOULDER` | `std_msgs/Float32` | hardware → planner | Actual velocity for stall detection. |
| `/ARM/JOINT/CURRENT_VELOCITY_ELBOW` | `std_msgs/Float32` | hardware → planner | Actual velocity for stall detection. |

## 5. Protocol and keys

### `lib/rovers-protocol/protocol.yaml` (both copies)

The rover copy already ends `arm_state` with `rest, zenith`. Remove the
`lock_all_joints` signal from the external basestation app copy so both end
with:

```yaml
      gripper_up: { type: UINT_8, default: 0 }
      gripper_down: { type: UINT_8, default: 0 }
      solenoid_strike: { type: UINT_8, default: 0 }
      rest: { type: UINT_8, default: 0 }
      zenith: { type: UINT_8, default: 0 }
```

`rest` and `zenith` are edge-triggered press states (0/1/2/3), like the
existing arm actions.

### Basestation

- `basestation/keyboard.py`: R already maps to `rest`; add Z → `zenith`.
- `basestation/app.py`: keep the L macro. It must continue to expand
  `lock_all_joints` into `toggle_joint_lock_N` pulses locally and never put
  `lock_all_joints` on the wire.
- Update the keyboard/app tests for the new wire length and signal names.

### Rover

- The command-key single source is
  `src/communications/basestation/basestation/basestation_node.py`. Add
  `"rest": "toggle_resting"` to `ARM_STATE_COMMANDS` (`"zenith": "zenith"`
  is already present). Do **not** add `lock_all_joints`.
- The master's manual-command mux (`arm_master_node.__handle_command`) drops
  `/ARM/COMMAND` whenever the MANUAL control source does not own the arm, so
  R would never reach the planner while AUTONOMOUS is active. Special-case
  `toggle_resting` (and `toggle_zenith`, which has the same defect) in the
  mux so these mode toggles always reach the planner; the planner's own
  gating decides whether they are accepted.
- `scripts`/firmware: the basestation firmware must be rebuilt because the
  last two `arm_state` bytes change from `lock_all_joints, rest` to
  `rest, zenith`.

## 6. Constants (`arm_constants.py`)

```python
# Operation modes
_ARM_OPERATION_MODES = {
    ...
    "ZENITH": (4, "ZENITH"),
    "RESTING": (5, "RESTING"),
}

# Flags
ARM_FLAG_NAMES = (
    ..., "ZENITH", "RESTING",
)

# Resting geometry
RESTING_LIMIT_PADDING = math.radians(30.0)
RESTING_CROUCH_TARGET = [0.0, 0.0, math.radians(135.0), 0.0, 0.0]

# Slow moves
RESTING_MOVE_ANG_VEL = math.radians(2.0)   # crouch / restore
RESTING_PUSH_RATE = math.radians(1.0)      # shoulder and elbow push [rad/s]

# Push targets (hard-clamped by RESTING_LIMITS)
RESTING_PUSH_MARGIN = math.radians(5.0)
RESTING_EXPECTED_SHOULDER_STALL = math.radians(-10.0)
RESTING_EXPECTED_ELBOW_STALL = math.radians(160.0)
RESTING_SHOULDER_PUSH = math.radians(-15.0)
# Elbow: expected stall (160 deg) + margin = 165 deg, exactly the padded
# upper limit.  A stop that is missing entirely therefore aborts on the
# boundary instead of idling until the push timeout.
RESTING_ELBOW_PUSH = RESTING_EXPECTED_ELBOW_STALL + RESTING_PUSH_MARGIN

# Stall detection.  The position window is the primary check; the rolling
# velocity average is advisory and must never be required to declare a stall
# (0x606C scale is unverified).
RESTING_START_GRACE_SEC = 1.0
RESTING_STALL_WINDOW_SEC = 0.5
RESTING_STALL_MOVE_EPS = math.radians(0.05)   # progress over the window
RESTING_STALL_VELOCITY_EPS = 3.0              # drive velocity units (advisory)
RESTING_PUSH_TIMEOUT_SEC = 60.0
RESTING_PUSH_ARRIVE_TOL = math.radians(0.1)   # "reached the push target"

# Backoff
RESTING_BACKOFF_ANGLE = math.radians(0.5)

# Topics
ARM_RESTING_TOPIC = "/ARM/HARDWARE/RESTING"
ARM_TORQUE_ENABLED_TOPIC = "/ARM/HARDWARE/TORQUE_ENABLED"
ARM_TORQUE_STATE_TOPIC = "/ARM/STATE/TORQUE_ENABLED"
```

`RESTING_LIMITS` is derived, not copied:

```python
RESTING_LIMITS = [
    MECHANICAL_LIMITS[0],                            # base unchanged
    (MECHANICAL_LIMITS[1][0] - RESTING_LIMIT_PADDING,  # shoulder lower
     MECHANICAL_LIMITS[1][1]),                       # shoulder upper
    (MECHANICAL_LIMITS[2][0],                        # elbow lower
     MECHANICAL_LIMITS[2][1] + RESTING_LIMIT_PADDING),  # elbow upper
    MECHANICAL_LIMITS[3],                            # wrist bend unchanged
    MECHANICAL_LIMITS[4],                            # wrist twist unchanged
]
```

Result: shoulder `-28..178°`, elbow `-135..165°`. The hard clamp is this
table; the push targets and backoff must stay inside it. Note the elbow push
target (165°) equals the padded upper limit, so a stop that is missing
entirely trips the padded-limit abort instead of idling until the timeout.

## 7. Planner / controller state machine

Add `kinematics/actions/resting.py` with a pure `RestingController`. The
controller exposes phases and returns intents (`resting`, `torque_enabled`,
`hold`) that the planner publishes.

### Collapse phases

| Phase | Planner | Hardware |
| --- | --- | --- |
| `MOVE_TO_CROUCH` | Plan a joint path to `RESTING_CROUCH_TARGET` with the same planner homing uses (restricted-octant-safe, elbow-deadband crossing included) and slow-step along it at `RESTING_MOVE_ANG_VEL` under `RESTING_LIMITS`. | Normal position mode. Resting latch on. |
| `PUSH_SHOULDER` | Step the shoulder target toward `-15°` at 1°/s. Elbow/base/wrists hold. | Poll `0x606C` and publish velocity. |
| `WAIT_SHOULDER_STALL` | After the start grace, declare a stall when the measured angle has not advanced by `RESTING_STALL_MOVE_EPS` over `RESTING_STALL_WINDOW_SEC`. The rolling velocity average is advisory: log it, but never require it to declare a stall. | Continue polling. |
| `BACKOFF_SHOULDER` | Set the target to `measured_shoulder + RADIANS(0.5)` and slow-step there. Wait for arrival. | Position mode. |
| `PUSH_ELBOW` | Step the elbow target toward `+165°` at 1°/s. | Poll velocity. |
| `WAIT_ELBOW_STALL` | Same windowed check. | Continue polling. |
| `BACKOFF_ELBOW` | Set the target to `measured_elbow - RADIANS(0.5)` and slow-step there. Wait for arrival. | Position mode. |
| `TORQUE_OFF` | Publish `torque_enabled=False`, mark `COLLAPSED`. The planner keeps publishing the normal target topics and its status (the master's 5 s watchdog is fed by planner status, so it stays satisfied); the hardware latch suppresses the CAN frames. | Controlword 0 on base/shoulder/elbow via `set_enabled(False)`, latch, clear target cache. |

Abort conditions during a push, checked in this order:

1. **No motion at all during the start grace** (less than
   `RESTING_STALL_MOVE_EPS` of progress while the target has been stepping):
   abort with a no-motion error. A stalled drive cannot be distinguished
   from a disabled arm or dead feedback, and declaring a stall here would
   back off from a pose the arm never reached.
2. **Feedback or enable lost mid-push**: no fresh measured-position sample
   for the joint being pushed (e.g., nothing for 5 control cycles) or the
   arm became disabled: abort with a feedback-lost/disabled error. This
   must never be declared a stall, which would back off and de-torque at
   the wrong pose. The resting controller checks the sample age of
   `/ARM/JOINT/CURRENT_*` and the master's `/ARM/STATE/ENABLED` level.
3. **Reached the push target without stalling** (measured angle within
   `RESTING_PUSH_ARRIVE_TOL` of the push target while the stall condition
   never fired): abort — the physical stop is not where it was expected.
4. **Reached the padded `RESTING_LIMITS` boundary before the stall was
   detected**: abort. (For the elbow, the push target *is* the boundary, so
   this is the same check as 3.)
5. **Push timeout** (`RESTING_PUSH_TIMEOUT_SEC`): abort.

On any abort: hold the measured pose, keep torque on, report the error, and
enter `HOLD`.

### Restore phases

| Phase | Planner | Hardware |
| --- | --- | --- |
| `TORQUE_ON` | Publish `torque_enabled=True`, wait for feedback. | Write measured positions as targets, enable, clear cache. |
| `MOVE_OUT` | Plan a path back to `RESTING_CROUCH_TARGET` (same planner) and slow-step at `RESTING_MOVE_ANG_VEL`. | Position mode. |
| `AUTO_HOME` | Exit RESTING (`resting=False`), start the normal homing action, and refresh the homing hold timestamp every cycle until `is_homing` becomes false. | Normal operation. |
| `DONE` | Normal PNP mode at the home pose. | Torque on. |

`AUTO_HOME` uses the existing `HomingController` and `_update_homing`, so the
elbow deadband crossing is handled by the current code. Known minor issue: the
crouch shoulder is 0° while the homing mechanical lower bound is 2°; the first
homing step clamps to 2°. Accept for v1 or add a `start_homing(skip_path=True)`
variant later.

The crouch moves (collapse and restore) must **not** step joints linearly: the
crouch elbow (+135°) is on the opposite side of the elbow deadband from a
normal-side start, and a straight move from a flipped or planar pose can sweep
restricted octants. Both `MOVE_TO_CROUCH` and `MOVE_OUT` reuse the planned
joint-path machinery homing already has (waypointed `_homing_path` with stall
limits plus the pin-and-cross deadband logic).

### Abort / E-stop

- **R during a sequence**: stop stepping, set the target to the measured pose,
  keep torque on, set phase `HOLD`, keep `resting=True`. The next R starts the
  restore sequence.
- **E-stop**: immediately publish `torque_enabled=False`; the hardware also
  sets its own latch on `/ESTOP`. On E-stop clear, torque stays off until the
  restore path publishes `torque_enabled=True`. This override is scoped to
  RESTING; outside RESTING the E-stop behaviour is unchanged.
- Do not let the master's handover `hold` or E-stop `hold` disturb an active
  resting phase; the resting action owns the target while it is active.
- The planner keeps publishing its status and the target topics while torque
  is off, so the master's 5 s planner watchdog (`PLANNER_TIMEOUT_SEC`) stays
  satisfied; only the hardware latch suppresses the CAN frames.
- **Latch lifecycle**: the planner republishes `/ARM/HARDWARE/RESTING` and
  `/ARM/HARDWARE/TORQUE_ENABLED` at 1 Hz (the same pattern as the
  continuously republished `/ARM/STATE/ENABLED`). Hardware clears the
  resting latch if no fresh `/ARM/HARDWARE/RESTING` message arrives for
  ~1.5 s, so a dead planner cannot keep the widened limits active; targets
  then revert to the normal limits. The torque latch is sticky in the other
  direction: once `torque_enabled=False`, only an explicit `True`
  re-enables the drives.

### Command gating

RESTING uses the same planner-owned gating model as ZENITH. The master's
command mux must let `toggle_resting` through regardless of control source
(see §5); everything below is enforced by the planner, which remains the
authority.

- **R during ZENITH is rejected** with an error until zenith exits.
- **While RESTING is active**, reject `flip`, `home`, `home_joint_N`,
  `position_drive`, `joint_drive`, `pnp_normal`, `toggle_planar`,
  `toggle_joint_lock_N`, `toggle_position_lock`, `toggle_orientation_lock`,
  `guide_orientation`, and `Z`; teleop and autonomous target inputs are
  ignored. R (abort/restore) is the only accepted operator input.
  `gripper_up`, `gripper_down`, and `solenoid_strike` stay allowed (they
  touch only the Teensy side and nothing the resting action owns).
- **R while the arm is disabled is rejected** with an error; the push relies
  on the drive actually moving during the start grace (see the no-motion
  abort above).
- **Joint locks**: entering RESTING saves and releases the joint locks
  exactly like zenith's `_zenith_saved_locks` pattern (a locked shoulder or
  elbow would silently refuse the crouch move). The saved locks are restored
  when RESTING completes (`DONE`), not at `AUTO_HOME` start, so homing never
  has to fight a locked joint.

### Planner integration points

RESTING must be excluded everywhere the planner would otherwise take over or
drop targets. The existing `self.zenith` guards in `arm_controller.py` are
the template; add a parallel `self.resting` guard in the same places:

- `arm_route_planner_node`: the `teleop_active` expression and the idle
  `self.__arm.hold()` branch both need `not self.__arm.resting`, or the
  update loop drops the resting targets every cycle.
- `arm_controller.py`: teleop updates, flip start, homing start, mode
  changes, and joint-lock toggles check `self.zenith` today; they must also
  check `self.resting`.
- The master's handover `hold` and E-stop `hold` must not disturb resting
  (see Abort / E-stop).

### Startup with a resting-region pose

After a power cycle the drives default to disabled and the planner infers a
mode from the measured pose. On startup the planner first publishes
`resting=False` (clearing any resting latch a crashed planner left behind),
then runs the inference. Add a resting-region check *before* the existing
zenith/flipped/planar inference: if the measured shoulder is below the normal
lower limit or the elbow is above the normal upper limit by more than the
inference tolerance, infer `RESTING`: publish mode `RESTING`, re-assert
`resting=True`, hold the measured pose, and publish `torque_enabled=False`
immediately so the hardware latch is set and the continuously republished
`/ARM/STATE/ENABLED` cannot re-enable the drives. Torque stays off until the
restore path publishes `torque_enabled=True`. A mid-collapse planner crash
with the pose still inside normal limits infers a normal mode and publishes
`torque_enabled=True` — safe, because torque had not been removed yet.

## 8. Hardware changes

### `arm_hardware_node.py`

- Subscribe to `/ARM/HARDWARE/RESTING` and `/ARM/HARDWARE/TORQUE_ENABLED`.
- Track `__resting` and `__torque_off` latches.
- Target validation in `__send_motor_command`:
  - while `__resting` and motor is shoulder or elbow, validate against
    `RESTING_LIMITS`;
  - otherwise keep the current `angle_limits()` check.
- While `__resting`, request `0x606C` for shoulder and elbow in
  `request_position`, staggered so the velocity reads never collide with the
  `0x6064` position reads to the same motor in one cycle (alternate cycles or
  offset per motor; 10 Hz velocity feedback is enough for a 0.5 s stall
  window), and publish `CURRENT_VELOCITY_*`.
- Decode incoming CAN feedback by object index:
  - `0x6064` → position (existing behavior);
  - `0x606C` → velocity (signed);
  - other → ignore.
- Torque latch:
  - `False`: `set_enabled(False)` for the three OpenCAN motors, set
    `__torque_off`, drop the target cache. `__on_arm_enable_received` must not
    re-enable while latched.
  - `True`: write each motor's measured position as its target, call
    `set_enabled(True)`, clear the target cache, clear `__torque_off`.
    Mirror `__on_arm_enable_received`: do not re-enable while `/ESTOP` is
    latched.
  - E-stop forces `__torque_off` and keeps it through the clear.
- Resting latch lifecycle: `/ARM/HARDWARE/RESTING` is refreshed at 1 Hz by
  the planner. If no message arrives for ~1.5 s, clear `__resting` so target
  validation reverts to the normal limits. `__torque_off` is sticky: it
  clears only on an explicit `torque_enabled=True`.
- Publish `/ARM/STATE/TORQUE_ENABLED` for diagnostics.

### `open_can_handler.py`

No velocity mode or profile-velocity override is needed. Existing helpers are
enough:

- `request_velocity()` already exists.
- `decode_velocity()` already exists.
- No `0x6060` / `0x60FF` writes for this feature.

## 9. Simulation

### `arm_sim_node.py`

The sim validates targets in **encoder-tick space** against
`ARM_MOTOR_PARAMS` tick limits (`TICK_LIMITS`, converted through
`ticks_per_rad`/`tick_offset`), not `MECHANICAL_LIMITS`. The provisional tick
limits are shoulder `~0..180.1°` and elbow `~-150.9..150.8°`, which would
reject the resting push targets outright, so:

- Subscribe to `/ARM/HARDWARE/RESTING` and `/ARM/HARDWARE/TORQUE_ENABLED`.
- While resting, **replace** the shoulder/elbow tick limits with
  `RESTING_LIMITS` converted through the same calibration; base/wrists keep
  the normal tick validation. Do not widen or pad the tick limits — override
  them, so the resting acceptance never depends on the provisional
  `ARM_MOTOR_PARAMS` values.
- Publish `CURRENT_VELOCITY_*` for shoulder and elbow from Gazebo
  `JointState.velocity` (or from position deltas if velocity is noisy).
- Torque off: stop advancing the Gazebo position command for base/shoulder/
  elbow and hold the current pose. Note the sim cannot model gravity sag: the
  Gazebo position controller holds the last command, so "torque off" is
  approximated as a pose hold.
- Torque on: resume from the measured pose.
- Keep the normal tick validation for non-resting operation.

### `arm.xacro`

Widen only the physical joint stops so Gazebo can reach the expected stall
points:

- `arm_shoulder_joint` lower: `-10°`.
- `arm_elbow_joint` upper: `160°`.

Do **not** change `MECHANICAL_LIMITS` in `arm_constants.py`.

## 10. Mode, flags, telemetry

- `ZENITH` = 4, `RESTING` = 5.
- Add a `RESTING` bool topic/flag alongside `ZENITH`.
- Update `telemetry_uplink_node.py` `ARM_FLAG_BITS`:
  - `ZENITH: 1 << 7`
  - `RESTING: 1 << 8`
- `arm_flags` is `UINT_16`, so no protocol size change is required.
- The sniffer subscribes to `ARM_FLAG_NAMES`, so RESTING appears automatically.
- Publish `RESTING` mode while collapsing or held; after `MOVE_OUT`, publish
  `HOMING` during `AUTO_HOME`, then the normal PNP mode. Startup inference
  publishing mode `RESTING` for a resting-region pose (§7) uses the same
  path.

## 11. Errors and safety

- Add a resting-specific error message (reuse `ARM_ERROR_REJECTED` or add a
  shared code if the basestation banner should distinguish it):
  - motion command attempted while resting;
  - mode command attempted during ZENITH or vice versa;
  - R attempted while the arm is disabled;
  - no motion at all during the start grace (arm disabled or feedback dead);
  - position feedback lost or the arm disabled mid-push (abort, never a
    stall);
  - push reached its target without stalling (physical stop missing);
  - push reached the padded limit without stalling;
  - push timeout;
  - joint moved the wrong direction during the grace period;
  - torque enable did not produce feedback.
- Watchdogs:
  - push timeout `60 s` (1°/s from 0° to -15° is 15 s; 135° to 165° is 30 s);
  - start grace `1 s`;
  - stall window `0.5 s`;
  - push-target arrival tolerance `0.1°`.
- On any fault during a push, stop stepping and hold the measured pose with
  torque on. Only the hardware E-stop forces torque off.
- The planner loop is non-blocking, so the master's 5 s planner watchdog is
  unaffected.

## 12. Tests

- Protocol:
  - `rest` and `zenith` occupy the last two bytes on both sides;
  - `lock_all_joints` is never on the wire;
  - round-trip encode/decode in `lib/rovers-protocol/tests`.
- Basestation:
  - R → `rest`, Z → `zenith`;
  - L still expands locally into per-joint lock pulses;
  - expected byte layout updated.
- Resting action:
  - 1°/s target stepping;
  - stall detection with synthetic velocity/position series (stall declared
    even when the velocity signal is noisy or absent);
  - 0.5° backoff values;
  - timeout, padded-limit abort, reached-push-target-without-stall abort,
    and no-motion-during-grace abort;
  - abort-and-hold;
  - E-stop torque latch.
- Gating and startup:
  - R rejected during ZENITH; Z and mode commands rejected during RESTING;
  - R rejected while the arm is disabled;
  - the master forwards `toggle_resting` while AUTONOMOUS owns the arm;
  - entering RESTING releases joint locks and `DONE` restores them;
  - a resting-region startup pose infers mode RESTING and publishes
    `torque_enabled=False`;
  - a crashed-planner startup sequence resets `resting` before inference.
- Planner integration:
  - the idle `hold()` branch and the `teleop_active` path leave resting
    targets alone;
  - the crouch move follows a planned path and the elbow deadband crossing
    works from a normal-side start;
  - mid-push feedback loss or disable aborts with a distinct error, never a
    stall;
  - the resting latch clears after ~1.5 s of planner silence while the
    torque latch stays set until an explicit re-enable.
- Controller:
  - `RESTING_LIMITS` only pads shoulder/elbow by 30°;
  - `MECHANICAL_LIMITS` unchanged;
  - restore exits RESTING at the crouch and starts homing.
- Hardware:
  - velocity request/decode by object index;
  - resting target acceptance;
  - torque-off latch survives enable re-asserts;
  - E-stop keeps torque off after clear.
- Simulation:
  - resting acceptance overrides the provisional tick limits (a +165° elbow
    target is accepted while resting and rejected otherwise).
- Manual:
  - full Gazebo collapse and restore;
  - confirm the sim stalls near shoulder `-10°`, elbow `+160°`.

## 13. Implementation order

1. Finish the protocol sync: the rover copy already ends `rest, zenith`.
   Update the external basestation app copy (drop `lock_all_joints`, append
   `zenith`), add `"rest": "toggle_resting"` to `ARM_STATE_COMMANDS`, and
   rebuild the basestation firmware.
2. Add constants, `RESTING_LIMITS`, mode, and flags.
3. Add hardware velocity feedback, resting target acceptance, torque latch.
4. Special-case `toggle_resting` (and `toggle_zenith`) in the master's
   command mux, then add `kinematics/actions/resting.py` and wire
   `toggle_resting` into the planner, including the `self.resting` planner
   guards, joint-lock release/restore, the planned crouch path, and the
   resting-region startup inference.
5. Add planner auto-home and mode/flag reporting.
6. Update `arm_sim_node` (tick-space resting limits override) and
   `arm.xacro`.
7. Update telemetry, sniffer/README docs, and tests.

## 14. Hardware verification checklist

- Confirm the shoulder/elbow physical stops are near `-10°`/`+160°`, and
  that no other obstruction exists within the padded band past them.
- Confirm the direction the shoulder and elbow move when the target is
  stepped past the stop.
- Confirm `0x606C` decodes with the expected sign and scale.
- Confirm controlword 0 truly removes torque and that the drives re-enable
  cleanly at the measured position (the enable word is the non-standard
  `0x2F`; the disable word must be verified against the same drives).
- Verify the `ARM_MOTOR_PARAMS` tick limits/offsets, which are currently
  marked provisional; the resting acceptance must not depend on them.
- Confirm the 0.5° backoff unloads each stop without visible sag.
- Tune `RESTING_STALL_VELOCITY_EPS`, the window, and the start grace on the
  real arm (velocity is advisory, so the position window is what must be
  tuned).
