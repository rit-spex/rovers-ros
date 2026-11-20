# Arm Action Driver — Implementation Plan

Status: **proposal** (not implemented). The behaviour decisions in §4 were
confirmed with the operator on 2026-11-20; remaining open items are listed
in §12.

## Checklist

- [ ] Split the actions (flip, home, zenith, resting) into a separate
      `arm_action_driver_node`; they barely use inverse kinematics.
- [ ] With the merged action driver node, re-evaluate the stepping logic,
      including adding velocity ramping.
- [ ] Make resting only possible in planar.
- [ ] Make zenith not care about base or twist.
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
| Target ownership | `arm_route_planner_node` is the sole writer of the five `/ARM/JOINT/TARGET_*` topics; actions run inside its 20 Hz cycle. |
| Stepping | Homing: fixed `HOMING_SPEED` per-cycle step (plus waypointed `_homing_path`). Flip: smoothstep over 30 frames. Resting: 1°/s pushes and 2°/s crouch moves, stall-detected. Teleop has its own Cartesian ramp. |
| Zenith | Target `[any base, 90°, 0°, 0°, 0°]`; releases joint locks and restores them on exit; remembers the previous PNP side; blocks other mode commands while active; teleop ignored. Base is *not* currently forced locked. |
| Planar | `PLANAR_LIMITS`: base ±5°, shoulder 0–180°, elbow mechanical, wrists mechanical. Exit is blocked in octants 3/5/7 and when the pose fits neither PNP side. |
| Resting | Enterable from any mode; resets mode flags on entry like zenith. |
| Regions | `flipped` is a commanded flag; zenith is a commanded mode; both are also reported on `/ARM/STATE/*` flag topics and mirrored in telemetry. |

## 4. Target behaviour

### 4.1 Action split

New node `arm_action_driver_node` (package `arm`, source
`src/arm_action_driver/arm_action_driver_node.py`), started by
`arm_launch.xml`, `main_launch.xml`, and `simulation_launch.xml`.

Moves into it:

- `FlipController` / `FlipSideController` (from `kinematics/actions/flip.py`)
- `HomingController` and the waypointed homing path logic
- Zenith enter/exit and its exit-target stepping
- `RestingController` (from `kinematics/actions/resting.py`)

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
  hold their measured values (the target simply never steps them).
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

### 4.3 Flipping

- Flip becomes a **maintained** action like homing: the key is held.
  - Press starts the flip toward its computed goal.
  - Release ("lifted") stops stepping and holds the measured pose (the
    action does not finish).
  - Press again resumes toward the **original** goal, even if the arm moved
    between presses (within limits).
- Moving the SpaceMouse (any nonzero manual velocity) clears the remembered
  goal and ends the action, handing control back to teleop.
- The flip action therefore gains a goal-memory field and a paused state in
  addition to active/inactive.

### 4.4 Resting

- `toggle_resting` is only accepted while **planar** is active. Entering
  resting sets planar = true (if not already) and forces it to stay true for
  the duration, mirroring the zenith flag-reset pattern.
- Startup inference: a resting-region pose (shoulder below the planar bound
  or elbow past its normal upper limit) currently infers RESTING directly.
  Decided: inference sets **both** the resting and planar flags (resting
  implies planar) and reports the base excursion on telemetry /
  `/ARM/STATE/ERROR` if the base is outside the planar band, so the
  operator sees it and resolves it.
- All other resting behaviour (crouch, stall pushes, torque latch, E-stop
  scoping, restore-to-home) is unchanged.

### 4.5 Planar limits and region tracking

- `PLANAR_LIMITS` shoulder/elbow become purely mechanical limits
  (`MECHANICAL_LIMITS[1]` and `MECHANICAL_LIMITS[2]`), so planar covers the
  full mechanical shoulder/elbow travel with the base locked to ±5°.
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
- **Elbow bending on planar exit:** with the wider mechanical shoulder/elbow
  travel, the elbow at exit time may sit on the opposite side of the
  singularity deadband from the side being entered. The exit transition
  reuses the homing pin-and-cross deadband logic: the elbow target is held
  on the correct side of the band until the measured elbow crosses, instead
  of stalling at zero. No exit motion happens before the fit check passes;
  if the fit check fails, only the error is raised.
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

- Owns the action state machine (idle / homing / flipping / zenith /
  resting) and is the single place that steps targets for those actions.
- Consumes `/ARM/JOINT/CURRENT_*` (measured angles), `/ARM/STATE/ENABLED`,
  `/ESTOP`, and the existing resting/torque hardware topics it needs.
- Publishes `/ARM/JOINT/TARGET_*` while an action is active, plus:
  - `/ARM/ACTION/ACTIVE` (`std_msgs/Bool`) — planner suppresses targets
    while true.
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
- Subscribes to `/ARM/ACTION/ACTIVE`: while true it does not publish joint
  targets (mirrors the existing resting/zenith "leave the action's targets
  alone" guards, now generalized).
- Keeps publishing status so the master's 5 s planner watchdog stays fed
  (the action driver is not the planner-status writer).

`arm_master_node`:

- Routes `flip`, `homing`/`home`, `home_joint:<n>`, `zenith`/`toggle_zenith`,
  `resting`/`toggle_resting` to `/ARM/ACTION/COMMAND`
  (`custom_interfaces/ArmCommand`, reuses the existing message) instead of
  `/ARM/PLANNER/COMMAND`.
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

### 5.3 Safety behaviour (unchanged)

- The action driver freezes its targets while `/ESTOP` is latched, like the
  planner and end effector do today.
- Targets are clamped to the active action's limits every cycle.
- The master's planner watchdog, the 5 s `PLANNER_TIMEOUT_SEC`, and the
  resting/torque latches keep their current timeouts; the action driver adds
  its own status heartbeat so a dead action driver is reported.

## 6. Stepping engine (velocity ramping)

Replace the three bespoke steppers (homing fixed step, flip smoothstep,
resting slow steps) with one shared profile in the action driver.

Proposed design:

- **Trapezoidal velocity profile per action move**: accelerate at
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
  - flip goal cancelled by manual input (informational, or reuse an existing
    code — decide);
  - zenith base-lock override rejected.
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
| `src/arm_action_driver/actions/` (or reuse `kinematics/actions/`) | Moved/adapted `flip.py`, `home.py`, `resting.py`, plus zenith logic from the controller. |
| `src/arm_route_planner/kinematics/arm_controller.py` | Remove action ownership; keep region/limit inference and the teleop stage; add the `/ARM/ACTION/ACTIVE` target-suppression guard. |
| `src/arm_route_planner/kinematics/regions.py` | New shared region-rule module (flipped/zenith/planar coincidence tracking). |
| `src/arm_route_planner/arm_route_planner_node.py` | Subscribe `/ARM/ACTION/ACTIVE`; stop publishing targets while active. |
| `src/arm_master/arm_master_node.py` | Route action commands to `/ARM/ACTION/COMMAND`; hold-on-action-start; merge `/ARM/ACTION/STATUS` errors. |
| `arm_constants.py` | New topics, stepping constants, planar mechanical limits. |
| `src/custom_interfaces/msg/ArmActionStatus.msg` | New message (action, phase, code). |
| `lib/rovers-protocol` | New `arm_errors` codes (submodule). |
| Launch files (`arm_launch.xml`, `main_launch.xml`, `simulation_launch.xml`) | Start `arm_action_driver_node`. |
| `README.md`, sniffer (optional) | Document the new node and topics. |

## 9. Tests

- **Stepping engine:** ramp-up/cruise/ramp-down shape, per-joint caps
  respected, synchronized direction preserved, pause decelerates to zero,
  resume reaches the original goal.
- **Flip:** goal memory survives release/re-press; SpaceMouse input clears
  the goal; limits clamp the goal.
- **Zenith:** base forced locked (toggle_joint_lock:0 rejected); goal never
  steps base or twist; homing exemption moves the base and exits zenith;
  teleop/IK/autonomous base motion blocked.
- **Resting:** `toggle_resting` rejected outside planar; resting forces
  planar; startup resting inference reports planar.
- **Planar/regions:** `PLANAR_LIMITS` shoulder/elbow equal mechanical
  limits; region flags track pose coincidence while planar; exiting planar
  with a pose that fits neither side raises the new error; octant 3/5/7 exit
  lock still applies.
- **Master routing:** action commands reach the action driver; planner gets
  `hold` on action start; action rejections surface on `/ARM/STATE/ERROR`.
- **Integration:** planner stops publishing targets while
  `/ARM/ACTION/ACTIVE` is true and resumes cleanly after; both writers never
  publish in the same cycle (log/check in sim).

## 10. Implementation phases

1. Extract region rules into `kinematics/regions.py`; add tests (no
   behaviour change).
2. Create `arm_action_driver_node` skeleton: subscriptions, status/active
   topics, launch entries; master routing switch with the planner hold.
3. Move homing first (simplest action), then flip, then zenith, then
   resting; keep planner guards in place until each move is verified.
4. Build the shared stepping engine and switch all four actions to it.
5. Apply the behaviour changes: resting-planar gate, zenith base lockout,
   flip goal memory/pause, planar mechanical limits + exit error.
6. Telemetry, basestation display strings, sniffer/README updates.
7. Sim verification (Gazebo `arm_drivebase`), then hardware checklist.

## 11. Verification

- Gazebo: run each action alone and in sequence; verify targets always come
  from exactly one node (`ros2 topic hz` on `/ARM/JOINT/TARGET_*` during
  actions vs teleop).
- Confirm a flip pause holds pose and resume completes; a SpaceMouse nudge
  cancels the goal.
- Confirm zenith leaves base/twist untouched and rejects base unlock and
  base teleop.
- Confirm resting is rejected outside planar and planar reports a flipped
  region when the pose is on that side.
- Confirm the new planar-exit error appears on the basestation.

## 12. Open questions

1. **Error code namespace.** Confirm the new planar-exit error is a new
   `arm_errors` code rather than reusing `ARM_ERROR_REJECTED` with an
   argument, and that the basestation app will ship a display string for it.
2. **Zenith base/twist lock reporting.** Should the forced base/twist locks
   be visible in `/ARM/STATE/LOCKED_JOINTS` while zenith is active (i.e. the
   mask shows bits 0 and 4 set), or only enforced internally and reported
   through a dedicated flag?
3. **Flip resume range.** If the arm is moved (by E-stop release, sag, or
   teleop nudge) between a flip pause and resume, how far may it be from the
   paused pose before resume re-plans instead of continuing to the original
   goal? Propose a tolerance constant; needs hardware feel testing.

Resolved during review (2026-11-20): target ownership via same topics +
active flag; zenith locks both base and wrist twist with a homing exemption;
resting startup inference sets RESTING + planar and reports base excursion;
SpaceMouse motion cancels the flip goal and aborts to teleop; "lifted" means
button released; region flags track the pose continuously in all modes;
planar-exit elbow bending reuses the homing deadband pin-and-cross logic.
