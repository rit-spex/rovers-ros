# Arm Action Driver — Implementation Plan

Status: **implemented** (2026-11-20). The behaviour decisions in §4 are in
the tree; §12 lists the remaining tuning/namespace items. See
"Implementation notes" at the end for what landed.

## Checklist

- [ ] Split the actions (flip, home, zenith, resting) into a separate
      `arm_action_driver_node`; they barely use inverse kinematics.
- [ ] With the merged action driver node, re-evaluate the stepping logic,
      including adding velocity ramping.
- [ ] Make resting only possible in planar.
- [ ] Make zenith hold and lock base and twist rather than driving them to 0°.
- [ ] Make flipping stop when the flip button is lifted.
- [ ] While in zenith, the base is ALWAYS locked out regardless of command
      settings. Only homing is exempt; teleop, IK, and autonomous input can
      never move the base in zenith.
- [ ] Reframe flipped/zenith as tracked regions rather than commanded modes.
- [ ] Flip tracks its goal: releasing and re-pressing the flip button
      resumes toward the original goal; moving the SpaceMouse drops the
      remembered goal.
- [ ] In planar, shoulder and elbow limits become purely mechanical limits.
      Whether the pose is flipped or zenith is coincidence: it is tracked
      and reported. Exiting planar while the pose fits no allowed region
      produces a new error.

## 1. Goal

Move the arm's discrete, mostly kinematic-free actions — `flip`, `home` /
`homing`, `zenith`, and `resting` — out of `arm_route_planner_node` into a
dedicated **`arm_action_driver_node`**, give all of them one shared,
ramp-limited stepping engine, and tighten the mode semantics so that
flipped/zenith become observed regions of the pose rather than commanded
states.

## 2. Why a separate node

Today the actions live inside `RoverArmController`
(`src/arm_route_planner/kinematics/arm_controller.py`) and
`kinematics/actions/{home,flip,resting}.py`, which is the same class that owns
IK, teleop, limiters, and joint locks. The actions barely use that machinery:

- `homing` steps each joint toward a fixed angle list (no IK).
- `flip` interpolates between two angle lists over a fixed frame count (no IK).
- `zenith` is a fixed joint target plus lock bookkeeping (no IK).
- `resting` is a fixed crouch target, slow pushes, and a torque latch (no IK;
  its crouch move uses the *joint-space* path planner, not Cartesian IK).

What the actions do share is: per-cycle target stepping, limit clamps,
E-stop/ownership interaction, and command gating. That shared surface is
exactly what a dedicated node can own once, with a single stepping engine
(§6). `arm_route_planner_node` keeps teleop, IK, limit modes for
pick-and-place/typing, joint locks, and `/ARM/STATE/POSE` publishing.

Note: the split is not literally "no kinematics at all". The geometric flip
needs shoulder/elbow/wrist *positions* (`l0`, `l1`, `l2`), which is forward
geometry, not inverse kinematics. The action driver will need a lightweight
FK that returns joint positions from angles — `linkage_models.py` already
provides it — but no IK solver.

## 3. Current state (what exists today)

| Concern | Today |
| --- | --- |
| Action code | `RoverArmController` owns `toggle_flipped`, `toggle_planar`, `enter_zenith`/`exit_zenith`, `_update_homing`, resting integration; `FlipController`, `HomingController`, `RestingController` are standalone classes in `kinematics/actions/`. |
| Command surface | `arm_master_node` forwards `flip`, `homing`/`home`, `home_joint:<n>`, `zenith`/`toggle_zenith`, `resting`/`toggle_resting` to the planner on `/ARM/PLANNER/COMMAND`. |
| Target ownership | `arm_route_planner_node` is the sole writer of the five `/ARM/JOINT/TARGET_*` topics today; actions run inside its 20 Hz cycle. After the split the invariant becomes "at most one writer": the planner or the action driver, never both (§5.1). |
| Stepping | Homing: fixed `HOMING_SPEED` per-cycle step (plus waypointed `_homing_path`). Flip: smoothstep over 30 frames. Resting: 1°/s pushes and 2°/s crouch moves, stall-detected. Teleop has its own Cartesian ramp. |
| Zenith | Target `[any base, 90°, 0°, 0°, 0°]`; releases joint locks and restores them on exit; remembers the previous PNP side; blocks other mode commands while active; teleop ignored. Base is *not* currently forced locked. |
| Planar | `PLANAR_LIMITS`: base ±2°, shoulder 0–180°, elbow mechanical, wrists mechanical. Exit is blocked in octants 3/5/7 and when the pose fits neither PNP side. |
| Resting | Enterable from any mode; resets mode flags on entry like zenith. |
| Regions | `flipped` is a commanded flag; zenith is a commanded mode; both are also reported on `/ARM/STATE/*` flag topics and mirrored in telemetry. |

## 4. Target behaviour

### 4.1 Action split

New node `arm_action_driver_node` (package `arm`, source
`src/arm_action_driver/arm_action_driver_node.py`), started by
`arm_launch.xml`, `main_launch.xml`, and `simulation_launch.xml`.

Moves into it:

- Flip handling: the side-flip target, maintained pause/resume, goal memory,
  safe-path re-planning, and manual-input cancel.
- Homing: full and single-joint moves with the safe joint path and the elbow
  deadband pin-and-cross.
- Zenith enter/hold/exit and its remembered side.
- `RestingController` (now `arm_action_driver/actions/resting.py`).

The action driver consumes measured angles, publishes the five joint
targets while an action is active, and publishes action state. It needs the
same E-stop freeze behaviour as the planner (see §5.3).

**Target ownership (decided):** the action driver becomes the target
publisher only while an action is active. When `arm_master_node` starts or
ends an action it publishes a `hold` to the planner (as it already does on
handovers), and the planner suppresses its own target publishing while
`/ARM/ACTION/ACTIVE` is true. This keeps one writer at a time without
changing the hardware interface. Alternatives (a separate
`/ARM/ACTION/TARGET_*` topic set, or a target mux node) are rejected: they
either duplicate the hardware interface or add a third arbiter.

### 4.2 Zenith

- Goal becomes "drive shoulder to 90°, elbow to 0°, wrist bend to 0°".
  Base and wrist twist are **not** part of the goal: they are commanded to
  hold their measured values (the target simply never steps them). This is
  a behaviour change: `ZENITH_JOINT_TARGET` drives wrist twist to 0° today.
  The constant is split into goal joints (shoulder/elbow/wrist bend) and
  held joints (base/twist), and `_is_zenith_pose()` drops its twist≈0 term,
  otherwise startup inference would stop recognising a held-twist zenith
  pose.
- While zenith is active, the base and wrist twist joints are always locked
  (`locked_joints[0] = locked_joints[4] = True`), regardless of the
  operator's lock settings:
  - `toggle_joint_lock:0` and `toggle_joint_lock:4` are rejected while
    zenith is active.
  - Teleop velocity, `/ARM/CMD_POSE`, and autonomous targets are already
    ignored; add an explicit base/twist motion guard so no path can move
    them.
  - Exception: homing (`home`, `home_joint:0`, `home_joint:4`) may move the
    locked joints; homing exits zenith first, exactly like the current
    "zenith blocks other mode commands" rule.
- The lock behaviour follows the existing zenith saved-locks pattern: the
  operator's base and twist lock settings are saved on entry and restored
  on exit.
- Keep: zenith blocks flip/planar/PNP/typing/locks while active; teleop and
  autonomous target inputs are ignored; exit returns to the remembered PNP
  side.
- The action has two substates, both reporting `ACTIVE = ZENITH` so the
  base/twist stay locked through both:
  - `ZENITH_HOLDING`: drive to the goal and hold it indefinitely, blocking
    other mode commands.
  - `ZENITH_EXITING`: a transient move to the remembered PNP-side pose
    (homing-profile speed, §6), then `ACTIVE = IDLE` and the saved locks
    restore.
- Exit triggers: `toggle_zenith` starts `ZENITH_EXITING`. `home` /
  `home_joint:*` skip the exit move entirely and transition straight into
  `HOMING` — the arm must not detour through the PNP pose on its way home,
  and this is the only path that unlocks base/twist before leaving zenith.
  PNP mode selection (`set_pnp_normal`, `toggle_flipped`, `toggle_planar`)
  stays rejected with "exit zenith mode first", exactly as today.

### 4.3 Flipping

- Flip becomes a **maintained** action like homing: the key is held.
  - Press starts the flip toward its computed goal.
  - Release ("lifted") decelerates to zero velocity and pauses (the action
    does not finish); see "paused = idle" below.
  - Press again resumes toward the **original** goal, even if the arm moved
    between presses.
- The command surface gains a release event: the basestation sends `flip`
  on button-down and `flip_release` on button-up; the master routes both to
  `/ARM/ACTION/COMMAND`. Today `flip` is one-shot and nothing reports a
  release, so maintained semantics need the new command.
- Resuming re-plans the safe path from the measured pose to the original
  goal (`plan_safe_path` + hold-joints logic, exactly as on first press),
  instead of resuming stale waypoints — the arm may have moved while
  paused. The original goal angles are kept; only the path is recomputed.
- **Paused = `ACTIVE = IDLE`** (decided 2026-11-20): after `flip_release`
  the driver decelerates to zero, publishes one final hold frame, then
  drops `ACTIVE` to IDLE and stops publishing — the planner takes target
  ownership back, holds the measured pose, and teleop is **not** gated
  while paused. The driver keeps the remembered flip goal in an internal
  idle state: `flip` pressed → re-take ownership and re-plan the safe path
  from the measured pose; SpaceMouse moves → wipe the goal.
- Moving the SpaceMouse (any nonzero manual velocity) clears the remembered
  goal whether the flip is running or paused (the driver subscribes to
  `/ARM/MANUAL/CMD_VELOCITY` in both states, §5.1). While running, the
  driver decelerates, drops `ACTIVE` to IDLE, and teleop takes over; while
  paused, teleop already owns the arm and the goal memory is simply
  dropped.
- The flip action therefore gains a goal-memory field in the idle state in
  addition to active/inactive.
- Flip start gating is unchanged: direct flips stay allowed in
  `OCTANT_DIRECT_FLIP` (even octants) and require planar elsewhere.

### 4.4 Resting

- `toggle_resting` is only accepted while **planar** is active. Planar is
  forced to stay true for the duration: the planner rejects `toggle_planar`
  while resting is active (via the action enum, §5.1), mirroring the zenith
  mode-block pattern.
- Startup inference: a resting-region pose (shoulder below the planar bound
  or elbow past its normal upper limit) currently infers RESTING directly.
  Decided: inference sets **both** the resting and planar flags (resting
  implies planar) and reports the base excursion on telemetry /
  `/ARM/STATE/ERROR` if the base is outside the planar band, so the
  operator sees it and resolves it.
- Boot inference must also *start* the resting action, not just set the
  flag: the planner (or master) commands `resting` to the action driver on
  init, because the crouch/de-torque state machine and the
  `RESTING_LATCH_REFRESH_SEC` torque-latch heartbeat now live in the driver
  (§5.1). Without this, an inferred RESTING pose would de-torque nothing.
- All other resting behaviour (crouch, stall pushes, torque latch on normal
  entry, restore-to-home) is unchanged — except the E-stop de-torque
  scoping, which is removed (§5.3): E-stop now position-locks everywhere.

### 4.5 Planar limits and region tracking

- `PLANAR_LIMITS` shoulder/elbow become purely mechanical limits
  (`MECHANICAL_LIMITS[1]` and `MECHANICAL_LIMITS[2]`), so planar covers the
  full mechanical shoulder/elbow travel with the base locked to ±2°.
- Flipped and zenith stop being modes that planar excludes. While planar is
  active, the pose may happen to be on the flipped side or straight up; that
  coincidence is **tracked from the measured pose** and reported on the
  existing `/ARM/STATE/FLIPPED` and `/ARM/STATE/ZENITH` flag topics (and
  mirrored in telemetry), but it does not change the active limits.
- Exiting planar requires the pose to fit either `PICK_AND_PLACE_LIMITS` or
  `PICK_AND_PLACE_FLIPPED_LIMITS` (and the existing octant 3/5/7 exit lock
  still applies). If neither fits, the exit is rejected with a **new error
  code** (e.g. `ARM_ERROR_PLANAR_EXIT_BLOCKED`) instead of the generic
  rejection text, so the basestation can display a distinct message.
- **Exiting planar is instantaneous and never moves the arm** (decided
  2026-11-20): the fit check passes → the mode switches; it fails → the new
  error is raised and nothing moves. The elbow singularity deadband stays a
  *homing* concern (the pin-and-cross logic lives in the homing action);
  the wider mechanical planar limits mean the operator may bend the elbow
  to a legal pose manually while planar, but the system never drives that
  bend as part of the exit.
- Region detection continues to feed mode inference at startup
  (`infer_initial_mode`): zenith pose → zenith; resting region → resting;
  restricted octants → planar; otherwise the PNP side that fits.

### 4.6 Regions instead of commanded states (summary)

| Today | Proposed |
| --- | --- |
| `flipped` is a commanded limit modifier; zenith is a commanded mode that resets planar/flipped. | Flipped/zenith are **reported regions** of the pose, tracked and published; they still select limits for teleop/IK, but no mode transition depends on them being "commanded". |
| Planar blocks flipped/zenith toggles while active. | Planar allows the pose to be in any region; the region flags just report it. |

## 5. Architecture

### 5.1 Node responsibilities

`arm_action_driver_node`:

- Owns the action state machine (idle / homing / flipping / zenith-holding
  / zenith-exiting / resting) and is the single place that steps targets
  for those actions. A paused flip is *idle with a remembered goal*, not a
  separate ACTIVE value (§4.3).
- Consumes `/ARM/JOINT/CURRENT_*` (measured angles), `/ARM/STATE/ENABLED`,
  `/ESTOP`, `/ARM/MANUAL/CMD_VELOCITY` (SpaceMouse cancel detection — the
  pre-gate teleop topic, so motion is visible even while the master gates
  teleop during an action; staleness handled like the planner's
  `VELOCITY_TIMEOUT_SEC`), and the existing resting/torque hardware topics
  it needs.
- Publishes `/ARM/JOINT/TARGET_*` while an action is active, plus:
  - `/ARM/ACTION/ACTIVE` (`std_msgs/Int8` enum: IDLE/HOMING/FLIPPING/
    ZENITH/RESTING) — the planner suppresses its targets while nonzero and
    gates mode/lock commands on the specific action (§5.2). A Bool cannot
    tell the planner that zenith is active (it must reject base/twist
    unlocks) or that resting is active (it must block `toggle_planar`), so
    the enum replaces the originally planned Bool.
  - `/ARM/ACTION/STATUS` (`custom_interfaces/ArmActionStatus`, new) —
    action id, phase, progress/error for the master and diagnostics.
  - resting hardware topics (`/ARM/HARDWARE/RESTING`,
    `/ARM/HARDWARE/TORQUE_ENABLED`) — these move from the planner to the
    action driver.
- Rejects commands that conflict with the active action (same gating rules
  the planner applies today: no flip during homing/zenith/resting, no zenith
  during flip/homing, etc.).

`arm_route_planner_node`:

- Keeps teleop, IK, pick-and-place/typing limit modes, joint locks,
  `/ARM/STATE/POSE`, planner status.
- Subscribes to `/ARM/ACTION/ACTIVE` (enum): while nonzero it does not
  publish joint targets (mirrors the existing resting/zenith "leave the
  action's targets alone" guards, now generalized). On the falling edge —
  and on a stale ACTIVE (timeout, driver dead) — it re-syncs its targets to
  the measured angles before resuming, so the arm never jumps back to
  pre-action targets.
- Keeps all mode/lock gating and reacts to the action enum: applies the
  forced base/twist zenith locks (visible in `/ARM/STATE/LOCKED_JOINTS`),
  rejects `toggle_joint_lock:0/4` in zenith and `toggle_planar` in resting,
  and blocks typing/flip/mode toggles while any action is active — the same
  checks `RoverArmController` performs today.
- Keeps publishing status so the master's 5 s planner watchdog stays fed
  (the action driver is not the planner-status writer).

`arm_master_node`:

- Routes `flip`, `flip_release` (new: button-up event), `homing`/`home`,
  `home_joint:<n>`, `zenith`/`toggle_zenith`, `resting`/`toggle_resting` to
  `/ARM/ACTION/COMMAND` (`custom_interfaces/ArmCommand`, reuses the existing
  message) instead of `/ARM/PLANNER/COMMAND`. Availability is checked like
  `__planner_available` today, so commands reject cleanly while the action
  driver is down.
- Sends the internal `hold` to the planner on action start/end (existing
  handover pattern).
- Continues to be the sole `/ARM/STATE/ERROR` writer; it merges action
  rejections from `/ARM/ACTION/STATUS` the same way it merges planner
  status today.
- The resting/zenith master-side bypass (toggles reach the planner under any
  control source) moves to the action-driver route.

### 5.2 Mode/limit coordination

Region flags (flipped, zenith, planar) are still consumed by the planner to
select limiters for teleop/IK. Two options existed; **A is decided**:

- **A — planner keeps region inference.** The planner already computes FK of
  measured angles; it keeps deriving flipped/zenith/planar regions and the
  action driver only drives joints. Least change; region rules live with the
  limiters they feed.
- (Rejected) **B — action driver owns regions.** The action driver publishes
  region flags; the planner consumes them. Cleaner "driver owns actions +
  regions" story, but duplicates FK and adds a topic dependency for teleop
  limits.

Region flags update **continuously from the measured pose in all modes**
(decided): they always reflect the observed pose and only select limits
where relevant. The region *rules* are extracted into a shared pure module
(`kinematics/regions.py`) so both nodes and the tests use one definition.

**Hysteresis (decided):** the region *flags* update every cycle, but the
planner latches the active *limit side*: PNP vs PNP-flipped limits only
re-evaluate on a flip action, planar enter/exit, zenith enter/exit, or
startup inference (the same latching `settle_side()` performs today).
Continuous teleop near a region boundary must never chatter the active
limits. `kinematics/regions.py` also applies a small angular deadband to
the reported flag transitions so telemetry does not flicker.

**Mode ownership (decided):** the planner keeps `limit_mode`, the
zenith/resting/planar/typing mode flags, joint locks, and the zenith lock
save/restore; the action driver never writes planner state. It only
publishes `/ARM/ACTION/ACTIVE` (enum) and `/ARM/ACTION/STATUS`, and the
planner reacts to them (switches to `ZENITH_MODE`/`RESTING_MODE` limits,
applies the forced locks, blocks conflicting commands). The driver owns
motion and its own action state machine; the planner owns modes and
gating.

### 5.3 Safety behaviour (unchanged)

- The action driver freezes its targets while `/ESTOP` is latched, like the
  planner and end effector do today.
- E-stop now **position-locks everywhere** (decided 2026-11-20): the
  hardware holds the measured pose with torque on, including during
  resting. The resting E-stop de-torque scoping (`TORQUE_ENABLED=false` on
  E-stop) is removed; the resting torque latch applies only to normal
  resting entry.
- Targets are clamped to the active action's limits every cycle.
- Stale `/ARM/ACTION/ACTIVE`: the planner treats an ACTIVE value that has
  not refreshed within a timeout (new `ACTION_ACTIVE_TIMEOUT_SEC`, same
  pattern as `VELOCITY_TIMEOUT_SEC`) as IDLE — a dead action driver must
  not leave target publishing suppressed or latched.
- **Handover races (three-node pub/sub is not atomic):** on action start
  the driver publishes `ACTIVE != IDLE` *before* its first target frame,
  and the master's `hold` to the planner sets an internal action-pending
  flag with a short timeout so the planner suppresses targets even if
  ACTIVE arrives late. On action end the driver publishes the final hold
  frame, then sets `ACTIVE = IDLE`; the planner re-syncs to measured angles
  on the edge (§5.1). The worst case is a sub-cycle gap (≤ one 20 Hz tick)
  with no writer — the hardware already holds the last setpoint through
  target gaps, which is the accepted behaviour.
- Known caveat (unchanged from the hardware E-stop work): the CAN
  transports drop frames while `/ESTOP` is latched, so the driver's frozen
  targets may never reach the bus; the hardware node's own position-lock
  sequence is the effective E-stop behaviour. Revisit the transports
  separately if hold frames should reach the bus.
- The master's planner watchdog, the 5 s `PLANNER_TIMEOUT_SEC`, and the
  resting/torque latches keep their current timeouts; the action driver adds
  its own status heartbeat so a dead action driver is reported.

## 6. Stepping engine (velocity ramping)

Replace the three bespoke steppers (homing fixed step, flip smoothstep,
resting slow steps) with one shared profile in the action driver.

The action driver runs its own timer at 20 Hz (the planner's rate), so
per-step constants convert cleanly (`HOMING_SPEED` × 20 Hz ≈ the current
homing rate).

Proposed design:

- **Trapezoidal velocity profile per action move**, re-planned every cycle
  from the remaining distance (a positional profile, not a precomputed time
  trajectory) so pause, resume, and E-stop freeze all follow the same
  path: accelerate at
  `ACTION_ACCEL_RAD_S2`, cruise at the min of `ARM_MOTOR_MAX_ANG_VEL[joint]`
  and the action's own cap, decelerate at `ACTION_DECEL_RAD_S2` to arrive at
  the goal. Optionally smooth the accel/decel corners (S-curve) with a jerk
  limit if the drives audibly clunk.
- **Synchronization**: scale the whole joint vector by one factor (like the
  teleop output stage) so multi-joint moves keep their direction; per-joint
  caps still apply.
- Per-action cruise speeds (proposed defaults, all tunable in
  `arm_constants.py`):
  - homing: keep today's effective speed as the cruise cap
    (`HOMING_SPEED` × 20 Hz ≈ current rate).
  - flip: current 30-frame smoothstep replaced by the profile; expected to
    finish in a similar time but with a clean stop.
  - resting crouch: 2°/s cruise (unchanged), pushes: 1°/s cruise with the
    existing stall/backoff logic unchanged (the profile only shapes the
    cruise, stall detection still overrides).
  - zenith exit: same profile as homing.
- **External overrides**: the profile is suspendable — homing's elbow
  deadband pin-and-cross and resting's stall/backoff freeze the profile and
  take over the commanded joint until the condition clears, then the
  profile is re-planned from the current pose.
- **Discrete-time braking:** at 20 Hz the braking velocity is clamped to
  `sqrt(2·a·d + (a·Δt/2)²) − a·Δt/2` (with `a = ACTION_DECEL_RAD_S2`,
  `d = remaining distance`, `Δt = 0.05 s`), and the step snaps exactly to
  the goal once `d ≤ v·Δt` with `v ≤ a·Δt` — otherwise the discrete
  integrator overshoots and oscillates around the goal.
- **Pause/resume**: a paused action (flip button released) decelerates to
  zero instead of snapping to a hold; resume re-ramps toward the same goal.
- The planner's teleop ramp is not touched; it stays in
  `arm_controller.py`. Homing/flip/zenith/resting move to the new engine;
  `SimulatedMechanicalArm` stays for any remaining planner-side callers
  (joint drive, absolute pose).

New constants: `ACTION_ACCEL_RAD_S2`, `ACTION_DECEL_RAD_S2` (or a jerk
limit if S-curve), and per-action cruise caps.

## 7. Errors and telemetry

- New error code(s) in `rover_protocol.arm_errors` (shared submodule):
  - planar exit blocked by pose (distinct from the octant lock);
  - zenith base/twist lock override rejected (raised by the planner, which
    owns the lock gating).
- Flip cancellation by manual input is **not** an error: it is reported
  through `ArmActionStatus.phase = cancelled`. Error codes stay reserved
  for operator-actionable rejections.
- Basestation app must add display strings for the new codes (external
  `rovers-basestation` change).
- No protocol size changes: flags and operation mode already carry the
  region information; `arm_flags` stays `UINT_16`.
- The sniffer subscribes to `ARM_FLAG_NAMES`, so region changes appear
  automatically once the flags are published.

## 8. Files affected

| File | Change |
| --- | --- |
| `src/arm_action_driver/arm_action_driver_node.py` | New node: action state machine, stepping engine, target publishing. |
| `src/arm_action_driver/stepping.py` | Shared positional trapezoidal profile with the discrete braking clamp. |
| `src/arm_action_driver/actions/resting.py` | Moved `RestingController`; flip/homing are driven by the node's ramp. |
| `src/arm_route_planner/kinematics/arm_controller.py` | Removed action ownership; keeps modes, limits, locks, teleop, IK. |
| `src/arm_route_planner/kinematics/regions.py` | New shared region-rule module (flipped/zenith/resting coincidence). |
| `src/arm_route_planner/arm_route_planner_node.py` | Subscribe `/ARM/ACTION/ACTIVE`; stop publishing targets while active. |
| `src/arm_master/arm_master_node.py` | Route action commands to `/ARM/ACTION/COMMAND`; hold-on-action-start; merge `/ARM/ACTION/STATUS` errors. |
| `arm_constants.py` | New topics, `/ARM/ACTION/ACTIVE` enum, `ACTION_ACTIVE_TIMEOUT_SEC`, stepping constants, planar mechanical limits. |
| `src/custom_interfaces/msg/ArmActionStatus.msg` | New message (action, phase, code). |
| `lib/rovers-protocol` | New `arm_errors` codes (submodule). |
| `rovers-basestation` (external) | Send `flip_release` on flip button-up; display strings for the new error codes. |
| `planz/RESTING_FEATURE_PLAN.md` | Update to match: resting E-stop de-torque scoping removed. |
| Launch files (`arm_launch.xml`, `main_launch.xml`, `simulation_launch.xml`) | Start `arm_action_driver_node`. |
| `README.md`, sniffer (optional) | Document the new node and topics. |

## 9. Tests

- **Stepping engine:** ramp-up/cruise/ramp-down shape, per-joint caps
  respected, synchronized direction preserved, pause decelerates to zero,
  resume reaches the original goal, no overshoot/oscillation around the
  goal at 20 Hz (discrete braking clamp, §6).
- **Flip:** goal memory survives release/re-press; a paused flip reports
  `ACTIVE = IDLE` and allows teleop; resume re-plans the safe path from the
  measured pose; SpaceMouse input clears the goal (running and paused);
  limits clamp the goal; `flip_release` routes through the master.
- **Zenith:** base and twist forced locked (`toggle_joint_lock:0/4`
  rejected, locks visible in `/ARM/STATE/LOCKED_JOINTS`); goal never steps
  base or twist; `ZENITH_HOLDING`/`ZENITH_EXITING` both report
  `ACTIVE = ZENITH`; homing skips the exit move straight into `HOMING`;
  teleop/IK/autonomous base motion blocked.
- **Resting:** `toggle_resting` rejected outside planar; resting forces
  planar; startup resting inference reports planar and starts the resting
  action in the driver (torque latch heartbeat observed).
- **Planar/regions:** `PLANAR_LIMITS` shoulder/elbow equal mechanical
  limits; region flags track pose coincidence while planar; exiting planar
  with a pose that fits neither side raises the new error; octant 3/5/7 exit
  lock still applies; teleop across a region boundary does not chatter the
  active limits (latched side + deadband).
- **Master routing:** action commands reach the action driver; planner gets
  `hold` on action start; action rejections surface on `/ARM/STATE/ERROR`.
- **Integration:** planner stops publishing targets while
  `/ARM/ACTION/ACTIVE` is nonzero and resumes cleanly after; a stale ACTIVE
  (driver killed mid-action) makes the planner resume holding the measured
  pose; both writers never publish in the same cycle (log/check in sim).
- **E-stop:** each action freezes on E-stop and resumes after; E-stop
  position-locks everywhere including resting (no `TORQUE_ENABLED=false`
  publish on E-stop anymore).

## 10. Implementation phases

1. Record golden sim logs of every action (baseline for regression), then
   extract region rules into `kinematics/regions.py`; add tests (no
   behaviour change).
2. Create `arm_action_driver_node` skeleton: subscriptions, status/active
   enum topics, launch entries; master routing switch with the planner hold.
3. Build the shared stepping engine and move homing onto it first (the
   simplest action), then flip, then zenith, then resting; keep planner
   guards in place until each move is verified.
4. Apply the behaviour changes: resting-planar gate, zenith base/twist
   lockout, flip goal memory/pause + `flip_release`, planar mechanical
   limits + exit error, resting E-stop de-torque removal.
5. Telemetry, basestation display strings, sniffer/README updates.
6. Sim verification (Gazebo `arm_drivebase`), then hardware checklist.

## 11. Verification

- Gazebo: run each action alone and in sequence; verify targets always come
  from exactly one node (`ros2 topic hz` on `/ARM/JOINT/TARGET_*` during
  actions vs teleop).
- Confirm a flip pause holds pose, teleop works while paused, and resume
  completes; a SpaceMouse nudge cancels the goal.
- Confirm zenith leaves base/twist untouched and rejects base unlock and
  base teleop; homing from zenith skips the exit move.
- Confirm resting is rejected outside planar and planar reports a flipped
  region when the pose is on that side.
- Confirm the new planar-exit error appears on the basestation and that
  exiting planar never moves the arm.
- Kill the action driver mid-flip and confirm the planner resumes holding
  the measured pose after the ACTIVE timeout (no jump to pre-action
  targets).
- Confirm E-stop during resting position-locks instead of de-torquing.

## 12. Open questions

1. **Error code namespace.** Confirm the new planar-exit error is a new
   `arm_errors` code rather than reusing `ARM_ERROR_REJECTED` with an
   argument, and that the basestation app will ship a display string for it.
2. **`/ARM/ACTION/ACTIVE` enum details.** Exact enum values and the
   `ACTION_ACTIVE_TIMEOUT_SEC` default (a few planner cycles) need picking
   during implementation; the planner and driver must share the constants.

Resolved during review (2026-11-20): target ownership via same topics +
active enum; zenith locks both base and wrist twist with a homing exemption
(locks visible in `/ARM/STATE/LOCKED_JOINTS`); resting startup inference
sets RESTING + planar, reports base excursion, and starts the resting
action in the driver; SpaceMouse motion cancels the flip goal and aborts to
teleop; "lifted" means button released (new `flip_release` command); a
paused flip is `ACTIVE = IDLE` with a remembered goal (teleop stays
available while paused); region flags track the pose continuously in all
modes but the active limit side latches (re-evaluated only on actions /
mode changes / startup) with a deadband on the reported flags; exiting
planar is instantaneous — fits or raises the new error, never moves the arm
(the elbow deadband pin-and-cross stays a homing concern); E-stop
position-locks everywhere, resting de-torque scoping removed; flip resume
re-plans the safe path from the measured pose; the planner owns
modes/gating/locks and re-syncs targets to measured angles on action end
or stale ACTIVE; zenith models `ZENITH_HOLDING`/`ZENITH_EXITING` and
homing skips the exit move; action handover orders ACTIVE before targets
with a sub-cycle no-writer gap tolerated by the hardware hold; the 20 Hz
profile uses the discrete braking clamp to avoid goal overshoot.

## Implementation notes (2026-11-20)

What landed:

- `arm_action_driver_node` owns homing, side flips, zenith, and resting.
  It reuses one `JointRamp` for homing/flip/zenith/resting targets, plans
  safe joint paths where the old controller did, and holds the elbow
  pin-and-cross at the singularity deadband.
- Flip is maintained: `flip` starts, `flip_release` decelerates to a pause
  (`ACTIVE = IDLE`, teleop available), re-pressing re-plans from the
  measured pose to the remembered goal, and manual velocity wipes the goal.
- Zenith latches base/twist at entry and reports `ZENITH_HOLDING` /
  `ZENITH_EXITING`; homing from zenith switches straight to `HOMING`.
- Resting requires planar (or an inferred resting startup pose), publishes
  the hardware latches itself, and runs the restore handshake through
  `/ARM/STATE/TORQUE_ENABLED`.
- The planner strips all action orchestration from `RoverArmController`
  (~500 lines removed), keeps modes/limits/locks/teleop/IK, mirrors the
  driver's mode flags and forced locks, and suppresses its targets while
  `/ARM/ACTION/ACTIVE` is nonzero. It re-syncs to measured angles on the
  falling edge and after a stale ACTIVE.
- The master routes the action command set to `/ARM/ACTION/COMMAND`, sends
  the planner `hold` before an action, refuses action commands while the
  driver is silent, and merges `/ARM/ACTION/STATUS` errors into
  `/ARM/STATE/ERROR`.
- `kinematics/regions.py` holds the shared onset/hysteresis rules; the
  planner now publishes `FLIPPED`/`ZENITH` from the measured region.
- `ArmActionStatus` carries `ready`, `active`, `phase`, `progress`,
  `homing_mask`, and the error event.
- The obsolete in-controller action tests were removed; the pure
  `RestingController` tests and new ramp/region tests remain, and a manual
  probe (`/tmp/opencode/action_driver_probe.py`, not committed) exercises
  zenith, flip pause/resume, homing release, the resting gate, and E-stop.

Not yet done: the §12 namespace/timing picks, Gazebo verification, and the
hardware bench checklist. The transport caveat in §5.3 still applies.
