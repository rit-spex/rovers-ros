# Arm Teleop Smoothing — Implementation Plan

Status: implemented.

## 1. Goal

Make SpaceMouse teleop of the 5-DOF arm smooth and responsive. The arm should
move in a straight line when the operator pushes straight, stop cleanly when
the stick is released, and not stall on sensor dropouts.

Constraints:

- Stay in OpenCAN **position** mode. No drive mode change (0x6060) and no
  velocity setpoints (0x60FF).
- Keep the predefined per-joint speed caps (`ARM_MOTOR_MAX_ANG_VEL`). Do not
  raise them.
- Keep `MECHANICAL_LIMITS`, the octant/planar/typing limiters, and the IK
  solver's geometry limits. Teleop still passes through them.
- All arm-side changes stay in `src/subsystems/arm`. The basestation is not
  modified by this plan.
- Wrist lock state is a basestation-side concept. The arm code must produce
  correct translation whether the wrists are locked or free (see §4.5).

## 2. Decisions (locked)

| Topic | Decision |
| --- | --- |
| Drive mode | Position mode only. |
| Speed caps | Unchanged. Forward reach stays near 3 in/s (shoulder cap 11.25 °/s). |
| Path | Joint speeds are scaled by one common factor, so the Cartesian direction is preserved. |
| Ramping | Ramp Cartesian velocity (in/s²), not each joint independently. |
| Zero velocity | A zero stick sample ramps the arm down. It does not call `hold()`. |
| Dropout tolerance | Arm side, in `arm_teleop_node.py`. Hold the last nonzero value per axis while a zero has persisted for less than 0.1 s (§4.1). |
| Stop target | About 0.5 s from full speed to a stop. |
| Raw range | `RAW_MAX = 350` in teleop. The SpaceMouse reports up to about ±350–450, so 350 is a clamp that keeps full throw at full scale. |
| Wrists | Not a planner or controller concern. The code works with any `locked_joints` state. |

## 3. Findings

All numbers are from `RoverArmController` in a simulated plant (first-order lag,
τ = 0.12 s, assumed). They have not been confirmed on hardware.

### 3.1 Stick dropouts stop the arm

`basestation/spacemouse.py::_filtered` returns 0 unless the last 5 samples share
a sign. On a steady stick with 2% noisy samples, 9.4% of outputs are 0. The
basestation is not changed by this plan, so the arm must tolerate these
dropouts.

The arm sees them in `arm_teleop_node.py`. The planner's
`arm_route_planner_node.py::__next_deltas` treats an all-zero velocity as "no
input". `__update` then calls `RoverArmController.hold()`, which snaps the
target to the measured pose and resets `mechanical.joint_speeds`. Each dropout
therefore costs a stop and a fresh 1 s ramp.

| Zero-velocity cycles | Steady speed (sim) |
| --- | --- |
| 0% | 2.37 in/s |
| 2% | 1.82 in/s (−23%) |
| 9.4% | 0.51 in/s (−78%) |

### 3.2 Independent per-joint slewing bends the path

`arm_controller.py::_command_mechanical` calls `SimulatedMechanicalArm.step`,
which caps each joint separately. The shoulder (11.25 °/s) lags the elbow
(37.5 °/s), so the tool drifts.

- Forward (−y), full stick, 3 s: tool Z error −10.1 in.
- The IK target runs up to 13.2 in ahead of the arm, because the target is
  integrated from the previous target and never anchored to the arm.

### 3.3 The command is built from measured feedback

`arm_route_planner_node.py::__update` calls `sync_angles(measured)` every cycle.
The new command is then "measured + one step". The feedback arrives
asynchronously, so sample age (0–50 ms) directly modulates the command.

- Command step roughness: 0.015° with exact timing, 0.21° with sample-age
  jitter (about 14×).

### 3.4 Ramp and release behaviour

- Every stick press starts from zero and takes the full 1 s
  (`ANGULAR_SPEED_RAMP_TIME`).
- Release stops the arm instantly, with no deceleration.
- Reversal keeps the old speed magnitude, so the joint jumps straight to full
  speed in the new direction.

### 3.5 Rotation

With the wrists locked (`locked_joints = [F, F, F, T, T]`), rotation commands
produce no motion. With the wrists free, rx and ry produce small motion.

Yaw (rz) produces about 1e-5 rad per 0.015 rad command, whatever the wrist
state. Position residuals (inches, lever about 44 in) dominate the orientation
residual (radians) in the damped least-squares solve, so yaw is effectively
suppressed. Yaw about the tool's own vertical axis needs sideways tool motion,
which the position-first solve refuses. This is a separate issue from the
translation work (see §8).

### 3.6 Not the cause

- The drives' speed limits. Commit `6b82025` raised the drive profile speeds
  5× (base 100→500 RPM, shoulder 30→150, elbow 100→500). The controller is the
  bottleneck, not the drives.
- The per-joint caps are intentional (`2ffd5b4` lowered the sim cap 36→12 °/s).
  The shoulder cap is the real forward-speed limit.

## 4. Target design

### 4.1 Input stage (arm side)

In `arm_teleop_node.py`, before the shaper:

- Track, per axis, the last nonzero raw value and when the value changed.
- While a zero has persisted for less than `SPACEMOUSE_DROPOUT_HOLD_SEC`
  (0.1 s), keep publishing the last nonzero value.
- Once the zero has persisted past the hold, pass zero through.
- Any nonzero sample updates the held value and resets the timer.
- Apply this to the raw axis, before `AxisShaper`, so the shaper's hysteresis
  state does not reset on a dropout.

The limit is `SPACEMOUSE_DROPOUT_HOLD_SEC = 0.1` in `arm_constants.py`.

Why time-based: the basestation sends SpaceMouse state on its own 25 Hz cycle,
and the teleop node must not depend on that cadence. A dropout shows up as one
or two zero cycles (about 40–80 ms), which the 0.1 s hold absorbs. A real
release is a run of zeros: the basestation's release burst sends five zero
frames over 200 ms, so the zero outlasts the hold and is passed through. The
hold does not count samples, so a bursty stream cannot turn a dropout into a
false release, and three consecutive zeros from a slow stream behave the same
as five from a fast one.

Cost: up to 0.1 s of extra latency on a real stop, inside the 0.5 s stop
target. After the hold passes a zero through, the existing
`TELEOP_INPUT_TIMEOUT_SEC` (0.5 s) staleness zeroing still applies if the
stream goes quiet.

Range: `RAW_MAX` changes from 127 to 350. `normalize()` already clamps to
[−1, 1], so values above 350 saturate at full scale. See §8 for the sensitivity
impact.

### 4.2 Cartesian velocity ramp (planner)

- Maintain a Cartesian velocity vector `v` (in/s) in
  `RoverArmController.update_teleop`, fed by the planner's converted
  `/ARM/CMD_VELOCITY` sample.
- Ramp `v` toward the stick's desired velocity with a vector accel limit:
  - accel: 9 in/s² (about 0.5 s to full speed at 4.5 in/s)
  - decel: 18 in/s² (about 0.25 s from full speed, plus the joint ramp and
    plant lag, gives a 0.55–0.6 s measured stop)
- Zero velocity ramps down. It does not call `hold()`.
- The 0.5 s stop target is the requirement; the measured stop in the sim is
  about 0.6 s. Both values are tunable in `arm_constants.py`.

### 4.3 Internal joint reference (controller)

New teleop path in `RoverArmController`, alongside `_update_position_drive`:

1. Do not overwrite the internal reference with measured angles every cycle.
2. Anchor the desired pose to the reference each cycle:
   `desired = FK(reference) + v·DT`.
3. Solve IK from that anchor. The existing limiters and geometry constraints
   stay in the solve.
4. Coordinated speed cap: scale the joint displacement vector by the largest
   single-joint overshoot of `ARM_MOTOR_MAX_ANG_VEL`. Direction is preserved.
5. Coordinated accel limit: compute the common ratio so that no joint's speed
   change exceeds `cap / TELEOP_JOINT_ACCEL_TIME_S`. Scale the whole change
   vector, not each joint.
6. Advance the reference by the limited joint velocity × DT.
7. Bound the lead: clamp the reference to within `TELEOP_LEAD_LIMIT_DEG` (5°)
   of measured angles. This is a safety limit, not a control input.
8. If the lead bound stays active for `TELEOP_LEAD_STALL_CYCLES` (10, about
   0.5 s), stop advancing the reference, hold the arm, and report the stall
   through the existing `_fail`/`classify_failure` path (the unrecognised
   message falls back to `ARM_ERROR_REJECTED`). Without this, a continuously
   binding lead silently degrades to the old measured-following behaviour.
9. Do not add a hard step clamp (`step = d` on overshoot). It creates velocity
   discontinuities; without it the accel limit holds exactly.

The max-ratio and accel ratio are computed over the joints that are allowed to
move. Locked joints have zero displacement, so they add no speed and no accel.

### 4.4 Resync rules

Resync the reference to measured angles on:

- first sync and re-enable
- control handover (manual ↔ autonomous)
- E-stop latch
- any mode change that leaves teleop (homing, flip, zenith, pose, typing)

Otherwise the reference is internal.

### 4.5 Wrist lock independence

Wrist lock state is set by the basestation. The arm does not need a teleop
wrist-unlock feature. The controller simply uses the `locked_joints` it is
given:

- With wrists locked, the solve is 3-DOF (base, shoulder, elbow). Translation
  must be straight and within caps.
- With wrists free, the solve is 5-DOF. Translation must still be straight and
  within caps. Rotation changes, but translation does not regress.

Tests cover both states (§6).

### 4.6 Measured pose telemetry

`/ARM/STATE/POSE` and the planner's FK-derived status must keep reporting the
**measured** pose, not the internal reference. Today `__publish_pose` reads the
controller's angles, which the new design repurposes as the reference.

- Keep the measured angles in the planner and run FK on them for
  `__publish_pose` and the status log.
- The arm typing node's arrival checks consume `/ARM/STATE/POSE`, so this also
  keeps autonomous arrivals honest.
- The controller-side FK of the reference continues to serve the target marker
  and the limit checks.

### 4.7 Out of scope for this change

- Homing, flips, absolute pose (`/ARM/CMD_POSE`), typing, and zenith keep the
  existing slew path. They can move to the new output stage in a follow-up.
- Yaw weighting in the IK solver (`linkage_models.py`).
- Any change to drive mode, speed caps, CAN frame layout, or the basestation.

## 5. Files

| File | Change |
| --- | --- |
| `src/subsystems/arm/src/arm_teleop/arm_teleop_node.py` | Per-axis dropout hold before the shaper. Uses `RAW_MAX = 350`. |
| `src/subsystems/arm/src/arm_route_planner/arm_route_planner_node.py` | Cartesian velocity ramp; zero velocity ramps down instead of `hold()`; resync rules; publish `/ARM/STATE/POSE` from measured angles. |
| `src/subsystems/arm/src/arm_route_planner/kinematics/arm_controller.py` | Teleop output stage (anchored IK, coordinated caps and accel over allowed joints, lead bound with stall reporting). |
| `src/subsystems/arm/src/arm_route_planner/kinematics/slew.py` | Unchanged for non-teleop callers. |
| `src/subsystems/arm/arm_constants.py` | New: `SPACEMOUSE_DROPOUT_HOLD_SEC = 0.1`, `TELEOP_ACCEL_IN_S2 = 9`, `TELEOP_DECEL_IN_S2 = 18`, `TELEOP_ANG_ACCEL_RAD_S2 = 3`, `TELEOP_JOINT_ACCEL_TIME_S = 0.25`, `TELEOP_LEAD_LIMIT_DEG = 5`, `TELEOP_LEAD_STALL_CYCLES = 10`, `TELEOP_STOP_EPSILON = 1e-3`. `RAW_MAX` 127 → 350. Sensitivity unchanged. |
| `src/subsystems/arm/test/test_teleop_smoothing.py` | New tests (§6). |
| `src/subsystems/arm/test/test_teleop_inputs.py` | Add dropout-hold tests next to the existing shaping tests. |
| `src/subsystems/arm/README.md` | Document the teleop input hold, the output stage, and the new constants. |

No changes to `basestation/`.

## 6. Tests

Unit tests:

- **Dropout hold (`test_teleop_inputs.py`):** a zero sample keeps the last
  nonzero value for 0.1 s. A zero that persists past 0.1 s passes through. Any
  nonzero sample updates the held value and resets the timer.
- **Dropout tolerance (planner):** a held stick with 10% zero samples keeps
  ≥ 90% of its steady speed. `hold()` is not called on a zero sample.
- **Straight line:** full forward stick for 3 s keeps |Z drift| < 0.5 in, with
  wrists locked and with wrists free.
- **Diagonal:** (−y, +z) stays on its 45° line within 0.5 in.
- **Caps:** joint speed never exceeds `ARM_MOTOR_MAX_ANG_VEL`.
- **Accel:** joint accel never exceeds `cap / TELEOP_JOINT_ACCEL_TIME_S`.
- **Reversal:** the tool's Y velocity changes sign exactly once.
- **Release:** full speed to stop in ≈ 0.5 s (≤ 0.6 s), coast ≤ 1.2 in.
- **Lead bound:** reference stays within 5° of measured angles, including with
  sample-age jitter.
- **Lead-bound stall:** with the lead bound active for 10 cycles, the reference
  stops advancing and a failure is reported through the error path.
- **Resync:** handover, E-stop, and re-enable resync the reference; teleop
  without those events does not.
- **Raw range:** a raw value of 350 gives full scale; 400 clamps to full scale.

Live check (no hardware):

- Run `arm_sim_node` with the basestation and planner.
- Log stick, velocity, joint command, and measured angles for a forward push,
  a diagonal, a reversal, and a dropout-injected run.
- Verify `/ARM/STATE/POSE` follows the measured angles while the reference is
  ahead of the arm.
- Compare against the thresholds above.

Hardware (later, with a logged session):

- Confirm the sim's τ, CAN latency, and shoulder-load assumptions.
- Confirm that the drives' default accel (0x6083/0x6084, never written) is
  acceptable.
- Log raw SpaceMouse values at full deflection to confirm the 350 range.

## 7. Phases

1. **Characterisation tests.** Add tests that reproduce §3.1, §3.2, and §3.3
   against the current code. They should fail now and pass after the fix.
2. **Arm-side input.** Time-based dropout hold in `arm_teleop_node.py`.
   `RAW_MAX` → 350.
3. **Zero-velocity path.** Ramp to zero instead of `hold()`. Keep hold for
   E-stop, handover, and timeouts.
4. **Output stage.** Anchored IK, coordinated caps and accel over the allowed
   joints, lead bound with stall reporting, measured-pose telemetry, and
   resync rules.
5. **Cartesian ramp.** Vector accel and decel in the planner, tuned to the 0.5 s
   stop.
6. **Live check and docs.** Sim run, README update.

Each phase should keep the test suite green (`src/subsystems/arm/test`).

## 8. Open questions

1. **Sensitivity at RAW_MAX 350 — resolved.** The full physical throw now maps
   to the full command; `TRANSLATION_SENS` and `ROTATION_SENS` are unchanged.
   Fine control is finer than before because the stick no longer saturates at
   36% of its travel.
2. **Yaw.** Accept that yaw is not supported in teleop, or change the IK
   weighting so orientation matters more? This affects the solver for all
   modes, so it is a separate task.
3. **Scope.** Should homing, flips, and absolute pose move to the new output
   stage in this change, or in a follow-up?

## 9. Known gaps

- Drive response (τ), CAN round-trip latency, shoulder load, and friction are
  assumed, not measured.
- The enable controlword (`ENABLE_CONTROLWORD = 0x2F`) and default drive
  accel/decel have not been checked on hardware.
- The 0.5 s stop target is met in the sim at about 0.6 s. The 9 in/s² accel,
  18 in/s² decel, and 0.25 s joint ramp are starting points until the sim
  confirms them.
- The 0.1 s dropout hold assumes dropouts rarely persist past 100 ms. Verify
  against a recorded basestation SpaceMouse stream before tuning the constant.
- `SimulatedMechanicalArm` still exists and still serves non-teleop callers.
  Its per-joint behaviour is unchanged by this plan.

## 10. Verification

- `src/subsystems/arm/test/test_teleop_smoothing.py` (10 tests) and the
  `AxisHold`/raw-range tests in `test_teleop_inputs.py` pass. Coverage
  includes straight translation with locked and free wrists, a single zero
  tick decelerating instead of resetting, noise immunity, caps, accel,
  reversal, the release stop, the lead bound and stall, and re-arming.
- The full arm suite passes: 125 passed, 1 skipped, excluding the package-wide
  flake8/pep257 tests. Those still report the existing style backlog; flake8
  is clean on the changed non-excluded files.
- A live planner check with a lagging fake arm (20 Hz planner, 4.5 in/s
  forward) measured:
  - shoulder target speed 11.25 °/s (exactly its cap), elbow 15.7 °/s
  - zero target drift after the release ramp
  - `/ARM/STATE/POSE` 0.14–0.24 in from FK(measured) against 0.46–0.59 in from
    FK(target), confirming the pose topic follows the measured arm
- Hardware response, CAN latency, and the real SpaceMouse range still need a
  logged run on the rover.
