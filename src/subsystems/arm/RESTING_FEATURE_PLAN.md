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

E-stop immediately removes OpenCAN torque and keeps it off after the E-stop
clears. Only the explicit restore re-enables torque.

## 2. Decisions (locked)

| Topic | Decision |
| --- | --- |
| Zenith | Kept. Both ZENITH (Z) and RESTING (R) exist. |
| Normal limits | `MECHANICAL_LIMITS` are **not** changed. |
| Resting limits | Shoulder/elbow may exceed normal limits by up to **30°**, hard-clamped. Base/wrists unchanged. |
| Push mechanism | Position mode only. Step the position target; do not use OpenCAN velocity mode. |
| Push speed | **1°/s** for both shoulder and elbow (0.05° per 50 ms control cycle). |
| Push target | 5° past the expected stop: shoulder `-15°`, elbow `+164°`, clamped by the 30° padding. |
| Stall detection | Windowed velocity/position check (stepping makes instantaneous velocity zero between steps). |
| Backoff | 0.5° after the stall; wait for the backoff move to finish before the next joint. |
| Torque off | CiA 402 controlword 0 on base/shoulder/elbow; latched. |
| E-stop | Forces torque off and keeps it off until restore. |
| Restore end | Leave RESTING at `[0,0,135,0,0]`, then auto-home. |
| R mid-sequence | Abort and hold, torque stays on, remain RESTING. |
| Protocol | `lock_all_joints` is basestation-only. Wire order ends `... solenoid_strike, rest, zenith`. |

## 3. Existing-state conflicts to resolve first

1. **Protocol mismatch.** The rover's `lib/rovers-protocol/protocol.yaml`
   currently ends `arm_state` with `zenith`; the basestation copy ends with
   `lock_all_joints, rest`. The rover decoder reads by position, so today the
   basestation's `lock_all_joints` byte is decoded as `zenith` and `rest` is
   ignored.
2. **Zenith is partially wired.** The controller, planner command dispatch,
   mode 4, and flag exist. It must stay, and RESTING must be added alongside it.
3. **Limits.** Normal mechanical limits are shoulder `2..178°`, elbow
   `-135..135°`. The requested stalls (`-10°`, `+160°`) are outside them by
   12° and 25° respectively, both within the agreed 30° allowance.
4. **Hardware validation is placeholder-driven.** `angle_limits()` is derived
   from `ARM_MOTOR_PARAMS`, which the constants file marks as incorrect. It
   must not gate resting pushes; use `RESTING_LIMITS` while resting.
5. **Enable re-assert.** `/ARM/STATE/ENABLED` is republished continuously, and
   `arm_hardware_node` re-sends enable frames on every message. A torque-off
   latch is required or the drives will be re-enabled immediately.
6. **Submodule state.** Both `lib/rovers-protocol` copies have uncommitted
   changes. Sync them in one upstream commit and update both submodule
   pointers.

## 4. Architecture

- `arm_route_planner_node` / `RoverArmController` owns the RESTING state
  machine, mode reporting, slow target stepping, and stall decisions.
- `arm_hardware_node` owns CAN primitives: velocity feedback, resting target
  acceptance, and the torque latch.
- `arm_sim_node` implements the same primitives against Gazebo.
- No new motion path: the planner keeps publishing the normal
  `/ARM/JOINT/TARGET_*` topics. RESTING only changes which limits are used and
  which hardware latches are active.

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

Remove the `lock_all_joints` signal. End `arm_state` with:

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

- `src/constants/constants/arm_commands.py`:
  - add `ArmCommandKey("rest", "toggle_resting")`
  - add `ArmCommandKey("zenith", "zenith")`
  - do **not** add `lock_all_joints`.
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
RESTING_HOME_TARGET = HOMING_TARGET

# Slow moves
RESTING_MOVE_ANG_VEL = math.radians(2.0)   # crouch / restore
RESTING_PUSH_RATE = math.radians(1.0)      # shoulder and elbow push [rad/s]

# Push targets (hard-clamped by RESTING_LIMITS)
RESTING_PUSH_MARGIN = math.radians(5.0)
RESTING_SHOULDER_PUSH = math.radians(-15.0)
RESTING_ELBOW_PUSH = math.radians(164.0)
RESTING_EXPECTED_SHOULDER_STALL = math.radians(-10.0)
RESTING_EXPECTED_ELBOW_STALL = math.radians(160.0)

# Stall detection
RESTING_START_GRACE_SEC = 1.0
RESTING_STALL_WINDOW_SEC = 0.5
RESTING_STALL_MOVE_EPS = math.radians(0.05)   # progress over the window
RESTING_STALL_VELOCITY_EPS = 3.0              # drive velocity units
RESTING_PUSH_TIMEOUT_SEC = 60.0

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
table; the push targets and backoff must stay inside it.

## 7. Planner / controller state machine

Add `kinematics/actions/resting.py` with a pure `RestingController`. The
controller exposes phases and returns intents (`resting`, `torque_enabled`,
`hold`) that the planner publishes.

### Collapse phases

| Phase | Planner | Hardware |
| --- | --- | --- |
| `MOVE_TO_CROUCH` | Slow-step all five joints toward `RESTING_CROUCH_TARGET` at `RESTING_MOVE_ANG_VEL` under `RESTING_LIMITS`. | Normal position mode. Resting latch on. |
| `PUSH_SHOULDER` | Step the shoulder target toward `-15°` at 1°/s. Elbow/base/wrists hold. | Poll `0x606C` and publish velocity. |
| `WAIT_SHOULDER_STALL` | After the start grace, declare a stall when the measured angle has not advanced by `RESTING_STALL_MOVE_EPS` over `RESTING_STALL_WINDOW_SEC` **and** the rolling velocity average is below `RESTING_STALL_VELOCITY_EPS`. | Continue polling. |
| `BACKOFF_SHOULDER` | Set the target to `measured_shoulder + RADIANS(0.5)` and slow-step there. Wait for arrival. | Position mode. |
| `PUSH_ELBOW` | Step the elbow target toward `+164°` at 1°/s. | Poll velocity. |
| `WAIT_ELBOW_STALL` | Same windowed check. | Continue polling. |
| `BACKOFF_ELBOW` | Set the target to `measured_elbow - RADIANS(0.5)` and slow-step there. Wait for arrival. | Position mode. |
| `TORQUE_OFF` | Publish `torque_enabled=False`, stop publishing OpenCAN targets logically, mark `COLLAPSED`. | Controlword 0 on base/shoulder/elbow, latch, clear target cache. |

If the measured angle reaches the padded `RESTING_LIMITS` boundary before the
stall is detected, abort: hold the measured pose, keep torque on, report an
error, and enter `HOLD`.

### Restore phases

| Phase | Planner | Hardware |
| --- | --- | --- |
| `TORQUE_ON` | Publish `torque_enabled=True`, wait for feedback. | Write measured positions as targets, enable, clear cache. |
| `MOVE_OUT` | Slow-step to `RESTING_CROUCH_TARGET` at `RESTING_MOVE_ANG_VEL`. | Position mode. |
| `AUTO_HOME` | Exit RESTING (`resting=False`), start the normal homing action, and refresh the homing hold timestamp every cycle until `is_homing` becomes false. | Normal operation. |
| `DONE` | Normal PNP mode at the home pose. | Torque on. |

`AUTO_HOME` uses the existing `HomingController` and `_update_homing`, so the
elbow deadband crossing is handled by the current code. Known minor issue: the
crouch shoulder is 0° while the homing mechanical lower bound is 2°; the first
homing step clamps to 2°. Accept for v1 or add a `start_homing(skip_path=True)`
variant later.

### Abort / E-stop

- **R during a sequence**: stop stepping, set the target to the measured pose,
  keep torque on, set phase `HOLD`, keep `resting=True`. The next R starts the
  restore sequence.
- **E-stop**: immediately publish `torque_enabled=False`; the hardware also
  sets its own latch on `/ESTOP`. On E-stop clear, torque stays off until the
  restore path publishes `torque_enabled=True`.
- Do not let the master's handover `hold` or E-stop `hold` disturb an active
  resting phase; the resting action owns the target while it is active.

## 8. Hardware changes

### `arm_hardware_node.py`

- Subscribe to `/ARM/HARDWARE/RESTING` and `/ARM/HARDWARE/TORQUE_ENABLED`.
- Track `__resting` and `__torque_off` latches.
- Target validation in `__send_motor_command`:
  - while `__resting` and motor is shoulder or elbow, validate against
    `RESTING_LIMITS`;
  - otherwise keep the current `angle_limits()` check.
- While `__resting`, request `0x606C` for shoulder and elbow at `DT` in
  `request_position`, and publish `CURRENT_VELOCITY_*`.
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
  - E-stop forces `__torque_off` and keeps it through the clear.
- Publish `/ARM/STATE/TORQUE_ENABLED` for diagnostics.

### `open_can_handler.py`

No velocity mode or profile-velocity override is needed. Existing helpers are
enough:

- `request_velocity()` already exists.
- `decode_velocity()` already exists.
- No `0x6060` / `0x60FF` writes for this feature.

## 9. Simulation

### `arm_sim_node.py`

- Subscribe to `/ARM/HARDWARE/RESTING` and `/ARM/HARDWARE/TORQUE_ENABLED`.
- While resting, accept shoulder/elbow targets up to `RESTING_LIMITS`.
- Publish `CURRENT_VELOCITY_*` for shoulder and elbow from Gazebo
  `JointState.velocity` (or from position deltas if velocity is noisy).
- Torque off: stop advancing the Gazebo position command for base/shoulder/
  elbow and hold the current pose.
- Torque on: resume from the measured pose.
- Keep normal target validation on `MECHANICAL_LIMITS`; only the resting
  latch enables the wider range.

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
  `HOMING` during `AUTO_HOME`, then the normal PNP mode.

## 11. Errors and safety

- Add a resting-specific error message (reuse `ARM_ERROR_REJECTED` or add a
  shared code if the basestation banner should distinguish it):
  - motion command attempted while resting;
  - push timeout;
  - push reached the padded limit without stalling;
  - joint moved the wrong direction during the grace period;
  - torque enable did not produce feedback.
- Watchdogs:
  - push timeout `60 s` (1°/s from 0° to -15° is 15 s; 135° to 164° is 29 s);
  - start grace `1 s`;
  - stall window `0.5 s`.
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
  - stall detection with synthetic velocity/position series;
  - 0.5° backoff values;
  - timeout and padded-limit abort;
  - abort-and-hold;
  - E-stop torque latch.
- Controller:
  - `RESTING_LIMITS` only pads shoulder/elbow by 30°;
  - `MECHANICAL_LIMITS` unchanged;
  - restore exits RESTING at the crouch and starts homing.
- Hardware:
  - velocity request/decode by object index;
  - resting target acceptance;
  - torque-off latch survives enable re-asserts;
  - E-stop keeps torque off after clear.
- Manual:
  - full Gazebo collapse and restore;
  - confirm the sim stalls near shoulder `-10°`, elbow `+160°`.

## 13. Implementation order

1. Sync `lib/rovers-protocol` in both repos; update R/Z wiring; remove
   `lock_all_joints` from the wire.
2. Add constants, `RESTING_LIMITS`, mode, and flags.
3. Add hardware velocity feedback, resting target acceptance, torque latch.
4. Add `kinematics/actions/resting.py` and wire `toggle_resting` into the
   planner.
5. Add planner auto-home and mode/flag reporting.
6. Update `arm_sim_node` and `arm.xacro`.
7. Update telemetry, sniffer/README docs, and tests.

## 14. Hardware verification checklist

- Confirm the shoulder/elbow physical stops are near `-10°`/`+160°`.
- Confirm the direction the shoulder and elbow move when the target is
  stepped past the stop.
- Confirm `0x606C` decodes with the expected sign and scale.
- Confirm controlword 0 truly removes torque and that the drives re-enable
  cleanly at the measured position.
- Confirm the 0.5° backoff unloads each stop without visible sag.
- Tune `RESTING_STALL_VELOCITY_EPS`, the window, and the start grace on the
  real arm.
