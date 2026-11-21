# Arm

ROS 2 package for the SPEX rover's 5-DOF arm plus gripper and solenoid. It
contains the arm subsystem master, the route planner, and the CAN hardware
bridge, plus a (placeholder) camera node and an opt-in debug sniffer.

The arm model is a five-joint chain — base yaw, shoulder pitch, elbow
pitch, wrist bend, wrist twist — with a separate gripper and solenoid. The
kinematics, limiters, and drive modes come from the `armSimulator4`
controller and live in `arm_route_planner/kinematics/`.

## Package layout

```
src/subsystems/arm/
├── arm_constants.py               # all arm constants: topics, motors, limits,
│                                  # CAN mappings, and controller tuning
├── launch/arm_launch.xml          # standalone launch for the main arm nodes
├── resource/arm                   # ament index marker
├── setup.py / setup.cfg           # entry points, src-layout packaging
├── src/
│   ├── arm_camera/                # placeholder node
│   │   └── arm_camera_node.py
│   ├── arm_teleop/                # teleop input normalization
│   │   └── arm_teleop_node.py
│   ├── arm_typing/                # autonomous keyboard typing
│   │   ├── arm_typing_node.py
│   │   └── typing_plan.py
│   ├── arm_master/
│   │   └── arm_master_node.py     # command router, readiness, errors
│   ├── arm_end_effector/
│   │   └── arm_end_effector_node.py # gripper target and solenoid
│   ├── arm_route_planner/
│   │   ├── arm_route_planner_node.py # IK, teleop, limits, target writer
│   │   └── kinematics/            # shared kinematic core (pure library)
│   │       ├── linkage_models.py  # symbolic FK, IK, frames, RPY helpers
│   │       ├── limiters.py        # workspace limiters and octant gating
│   │       ├── path_planner.py    # safe joint-space path search
│   │       ├── slew.py            # joint slew / velocity-limit model
│   │       ├── regions.py         # observed flipped/zenith/resting regions
│   │       └── arm_controller.py  # RoverArmController: teleop, IK, modes
│   ├── arm_action_driver/
│   │   ├── arm_action_driver_node.py # homing, flip, zenith, resting motion
│   │   ├── stepping.py            # shared trapezoidal step profile
│   │   └── actions/
│   │       └── resting.py         # RESTING collapse/restore state machine
│   ├── arm_hardware/
│   │   ├── arm_hardware_node.py   # ROS ↔ CAN boundary, transport, standard CAN
│   │   ├── arm_simulated_teensy_node.py # wrist/gripper Teensy stand-in (no wrist HW)
│   │   └── open_can_handler.py    # OpenCAN: base, shoulder, elbow
│   └── arm_sniffer/               # opt-in debug node, not launched by default
│       └── arm_sniffer_node.py
├── planz/                          # design plans:
│   ├── ACTION_DRIVER_PLAN.md       # action driver design (implemented)
│   ├── RESTING_FEATURE_PLAN.md     # resting collapse/de-torque plan
│   └── TELEOP_SMOOTHING_PLAN.md    # teleop smoothing plan
└── test/                           # unit, limiter, and config tests
```

## Nodes

The package provides eight runtime nodes plus an opt-in debug tool.
`arm_launch.xml` starts the first eight and renames them to `arm_camera`,
`arm_teleop`, `arm_typing`, `arm_route_planner`, `arm_action_driver`,
`arm_master`, `arm_end_effector`, and `arm_hardware`.

| Executable | Short alias | Standalone node name | Launched by default | Responsibility |
| --- | --- | --- | --- | --- |
| `arm_teleop_node` | `teleop` | `arm_teleop_node` | yes | Owns SpaceMouse semantics; publishes normalized velocity and gripper commands. |
| `arm_typing_node` | `typing` | `arm_typing_node` | yes | Autonomous keyboard typing: plane/key inputs, drive-edge alignment, Cartesian key targets, and solenoid strikes. |
| `arm_route_planner_node` | `route_planner` | `arm_route_planner_node` | yes | Owns 5-DOF target generation: IK, teleop, limit modes, locks, pose, and the arm target topics. |
| `arm_action_driver_node` | `action_driver` | `arm_action_driver_node` | yes | Owns homing, side flips, zenith, and resting motion plus the resting/torque hardware latches. |
| `arm_master_node` | `master` | `arm_master_node` | yes | Arm subsystem master: arbitrates control ownership, routes commands, watches planner and driver health, and is the sole `/ARM/STATE/ERROR` writer. |
| `arm_end_effector_node` | `end_effector` | `arm_end_effector_node` | yes | Owns the gripper target, gripper drive, and solenoid strike. |
| `arm_hardware_node` | `hardware` | `arm_hardware_node` | yes | Owns the CAN link: sends targets, polls measured angles, enforces enable/E-stop. Replaced by `rover_simulation/arm_sim_node` in simulation. |
| `arm_camera_node` | `camera` | `arm_camera_node` | yes | Placeholder; no logic yet. |
| `arm_sniffer_node` | `sniffer`, `ASN` | `arm_sniffer_node` | no | Read-only diagnostic printer. |
| `arm_simulated_teensy_node` | `sim_teensy`, `SATN` | `arm_simulated_teensy` | no | Simulated wrist/gripper Teensy; not used by the real rover launch. |

Every executable keeps its canonical `arm_*_node` name, so existing launch
files and scripts are unchanged. The short aliases are only conveniences:

```bash
ros2 run arm sniffer
ros2 run arm ASN
ros2 run arm sim_teensy
ros2 run arm SATN
```

The alias runs the same node with the same runtime node name as the canonical
executable (`arm_sniffer_node` and `arm_simulated_teensy` respectively).

---

### `arm_teleop_node`

The only node that knows the SpaceMouse exists. It subscribes to the single
`/BASESTATION/spacemouse/axes` topic (`custom_interfaces/SpaceMouseAxes`),
applies the sign convention, deadzone with entry hysteresis, power curve,
vector-magnitude limit, and sensitivity, then republishes the result as
manual velocity on `/ARM/MANUAL/CMD_VELOCITY` at `TELEOP_PUBLISH_RATE_HZ`
(default 50 Hz). It republishes continuously so the master always has a fresh
sample.

The basestation packs all six axes into one message so a sample is atomic and
the teleop node has one liveness timestamp instead of six.

- Axis processing happens exactly once, here; downstream nodes consume
  normalized values only.
- Translation is converted from inches-per-cycle to m/s at the ROS boundary.
- The three linear axes and the three rotation axes are each scaled so a
  diagonal deflection is no faster than a single-axis one.
- Raw HID values are normalized against `RAW_MAX = 350`, which is the
  SpaceMouse's useful full throw; values above it clamp to full scale.
- A zero sample only counts as a release once it has persisted for
  `SPACEMOUSE_DROPOUT_HOLD_SEC` (0.1 s). The basestation's short sign filter
  can emit isolated zeros while the stick is held, and treating those as a
  release would stop the arm and cost a fresh ramp.
- An axis whose raw value has not refreshed for `TELEOP_INPUT_TIMEOUT_SEC` is
  commanded as zero. The basestation republishes teleop axes every cycle, so a
  dead link cannot be mistaken for a held stick.
- The gripper and solenoid are deliberately not handled here; they belong to
  `arm_end_effector_node` and the basestation's up/down arrows.
- It emits manual intent only, on its own topic. Keyboard actions arrive
  separately on `/ARM/COMMAND` from `basestation_node`. `arm_master_node` is
  the mux that arbitrates both, so this node never reads the control source.

Tuning lives in `arm_constants.py` (`TELEOP_PUBLISH_RATE_HZ`,
`TRANSLATION_SENS`, `ROTATION_SENS`, `DEADZONE`, `ZERO_HYSTERESIS`,
`INPUT_EXPONENT`, `SPACEMOUSE_DROPOUT_HOLD_SEC`, `RAW_MAX`,
`TELEOP_INPUT_TIMEOUT_SEC`). The node does not declare ROS parameters because
nothing overrides them; changing the constants requires a rebuild, which
matches the rest of the arm tuning.

---

### `arm_typing_node`

Autonomous keyboard typing. It is the first user of the autonomous command
surface: it requests `AUTONOMOUS` control, publishes absolute arm poses on
`/ARM/AUTONOMOUS/CMD_POSE`, and publishes solenoid commands on
`/ARM/AUTONOMOUS/COMMAND`. `arm_master_node` is still the only mux and
forwards those topics only while autonomy owns the arm.

Inputs:

| Topic | Type | Notes |
| --- | --- | --- |
| `/ARM/TYPING/PLANE` | `custom_interfaces/ArmTypingPlane` | Keyboard face in the rover base frame. The origin is the keyboard centre; `+X` runs along the keyboard, `+Y` up the face, and `+Z` out toward the rover. `side` may be `+1` (left), `-1` (right), or `0` to infer from `x`. |
| `/ARM/TYPING/KEYS` | `std_msgs/String` | Literal text to type. Spaces, tabs, newlines, and braced names such as `{enter}` or `{space}` are supported. |
| `/ARM/STATE/CONTROL_SOURCE` | `std_msgs/UInt8` | Confirms that autonomy actually owns the arm. |
| `/ARM/PLANNER/STATUS` | `custom_interfaces/ArmPlannerStatus` | Planner readiness. |
| `/ARM/STATE/POSE` | `custom_interfaces/Pose` | Measured end-effector pose used for the arrival check. |
| `/ESTOP` | `std_msgs/Bool` | Aborts the sequence and stops the wheels. |

Outputs:

| Topic | Type | Notes |
| --- | --- | --- |
| `/ARM/CONTROL/REQUEST` | `std_msgs/UInt8` | Requests `AUTONOMOUS` while typing and hands back `MANUAL` when done. |
| `/ARM/AUTONOMOUS/CMD_POSE` | `custom_interfaces/Pose` | Absolute key pose in the rover base frame. |
| `/ARM/AUTONOMOUS/COMMAND` | `custom_interfaces/ArmCommand` | `fire_solenoid` at each key. |
| `/BASESTATION/xbox/AXIS_LY` | `std_msgs/Float32` | Left-wheel forward/back creep, capped at `TYPING_DRIVE_MAX_POWER` (10%). |
| `/BASESTATION/xbox/AXIS_RY` | `std_msgs/Float32` | Right-wheel forward/back creep, capped at `TYPING_DRIVE_MAX_POWER` (10%). |
| `/ARM/TYPING/STATUS` | `custom_interfaces/ArmTypingStatus` | State, current key, progress, and detail for debugging. |

How a sequence runs:

1. Parse the string into keys from `TYPING_KEYBOARD` in
   `arm_constants.py`. The table is a reference layout, not a measurement;
   calibrate the per-key positions and sizes in key units against the real
   keyboard, then adjust the pitch if needed.
2. Request autonomous ownership and wait for the planner to report ready.
3. Send `typing_mode` once so the planner selects the typing limits: base
   restricted to +/-90 deg and the wrist locks released for a level tool.
4. For each key, compute its position in the plane. If the key is not on the
   arm's lateral reach line, drive forward/back at up to 10% until it is.
   The rover must already be parallel to the keyboard and at the right
   standoff; the differential drive cannot correct those without turning.
   The drive command is published on the normal Xbox axes
   (`/BASESTATION/xbox/AXIS_LY` and `AXIS_RY`), so `chassis_node` handles it
   exactly like an operator holding the sticks and needs no autonomous-drive
   code path. Do not hold the sticks while typing: the basestation publishes
   on the same topics, so a live stick takes over until it returns to
   neutral.
5. Publish the key pose with the tool at `TYPING_STANDOFF_M` from the face.
6. Wait until `/ARM/STATE/POSE` is within the position and orientation
   tolerances, then fire the solenoid and advance.

This works because the keyboard sits on the rover's left or right: the arm
base can stay near `+90` or `-90` deg while the drivebase moves the next key
onto the reach line. It deliberately does not ask the wrist to sweep across
the keyboard.

Example for a vertical keyboard on the rover's right (plane normal points at
the rover, `-X`; adjust `x`/`z` to a reachable standoff):

```bash
ros2 topic pub --once /ARM/TYPING/PLANE \
  custom_interfaces/msg/ArmTypingPlane \
  "{x: 0.5, y: 0.0, z: 0.45, roll: 1.5708, pitch: 0.0, yaw: -1.5708, side: -1}"

ros2 topic pub --once /ARM/TYPING/KEYS \
  std_msgs/msg/String "{data: 'hello'}"
```

The node publishes progress on `/ARM/TYPING/STATUS` and waits for a fresh
plane stream while it drives; publish the plane repeatedly from the camera or
localization source while a sequence is running.

The node rejects uppercase and shifted symbols: the solenoid is a momentary
striker and cannot hold Shift while another key is pressed. It also refuses
to drive without a fresh plane pose, releases ownership when finished, and
stops the wheels on every state transition. It leaves the planner in typing
limits when the sequence ends so the arm is not pulled out of a side pose;
select homing or another limit mode to restore the normal P&P setup.

---


### `arm_master_node`

The arm subsystem master and command mux. It is the only node that reads the
control source, and every manual input reaches an actuator through it:

- **Teleop:** `/ARM/MANUAL/CMD_VELOCITY` is forwarded to `/ARM/CMD_VELOCITY`
  while manual owns the arm.
- **Keyboard:** `/ARM/COMMAND` is split by owner. 5-DOF motion commands go to
  `arm_route_planner_node` on `/ARM/PLANNER/COMMAND`, which validates them and
  owns their readiness, E-stop, and unknown-command errors. The gripper and
  solenoid commands (`gripper_up`, `gripper_down`, `fire_solenoid`,
  `toggle_solenoid`) go only to `arm_end_effector_node` on
  `/ARM/END_EFFECTOR/COMMAND`; the planner never sees them, so the master
  validates their preconditions.
- **Actions:** `flip`, `flip_release`, `home`/`homing`, `home_release`,
  `home_joint:<n>`, `zenith`/`toggle_zenith`/`zenith_release`, and
  `resting`/`toggle_resting` go to `arm_action_driver_node` on
  `/ARM/ACTION/COMMAND`, after an internal `hold` to the planner. Resting
  and zenith bypass the MANUAL-only gate; `flip_release`, `zenith_release`,
  and `home_release` are always forwarded so a hold-to-run action can stop.

Because the master emits the arbitrated stream, the planner and end effector
do not subscribe to `/ARM/STATE/CONTROL_SOURCE` and never need to know who owns
the arm.

It owns `/ARM/STATE/ERROR` and consumes `/ARM/PLANNER/STATUS` and
`/ARM/ACTION/STATUS` for readiness and the latest error events. Commands are
ignored while E-stopped; solenoid strikes are allowed before the arm
initializes, while the gripper jog waits for planner readiness. Rejections of
a held gripper arrow are throttled to one report per second so the error
banner stays readable. If planner status stops arriving for
`PLANNER_TIMEOUT_SEC` (5 s), the master reports `ARM_ERROR_REJECTED` so the
loss stays visible; action commands are likewise refused when the action
driver has not reported for `ACTION_ACTIVE_TIMEOUT_SEC`.

It is also the control arbiter and the single writer of
`/ARM/STATE/CONTROL_SOURCE`. Sources request ownership on
`/ARM/CONTROL/REQUEST`; the master validates the request (manual always wins,
autonomous needs a live and ready planner outside an E-stop) and publishes
the result with transient-local QoS. `/ARM/COMMAND` and
`/ARM/MANUAL/CMD_VELOCITY` are the manual surfaces, so they are ignored while
an autonomous source owns the arm. `/ARM/AUTONOMOUS/CMD_POSE` and
`/ARM/AUTONOMOUS/COMMAND` are the autonomous surfaces and are ignored while
manual owns the arm.

A handover publishes a zero velocity, an internal `hold` to the planner, and
an internal `stop_jog` command to the end effector, so the previous owner's
last sample cannot leak into the new owner's window. Manual velocity also
requires a neutral stick after every handover or E-stop before motion resumes.
If an autonomous pose stream stops for `AUTONOMOUS_POSE_TIMEOUT_SEC`, the
master holds the measured pose instead of chasing a dead target.

---


### `arm_end_effector_node`

The gripper and solenoid. It has no kinematic model and no error logic: the
master validates the command preconditions and reports errors, and the
planner is not involved. It tracks the measured gripper angle unless an arrow
jog is active, moves the target at `GRIPPER_JOG_RATE` while `gripper_up` /
`gripper_down` commands keep arriving, clamps it to `GRIPPER_MIN_ANGLE` ..
`GRIPPER_MAX_ANGLE`, and stops if the command stream goes quiet for
`GRIPPER_JOG_TIMEOUT_SEC`. Up opens (toward 0) and down closes (toward 90
degrees).

A solenoid strike is a single `true` pulse on `/ARM/JOINT/TARGET_SOLENOID`;
the Teensy fires and retracts the solenoid itself. The node freezes its
outputs while E-stopped, matching the other arm nodes. It publishes the
gripper target as soon as a measured angle is available; `arm_hardware_node`
and the Gazebo bridge drop targets while the arm is disabled.

---


### `arm_route_planner_node`

The 5-DOF teleop/IK core and the main consumer of joint feedback. It owns a
`RoverArmController` for its whole lifetime and stays uninitialized until all
five measured joint angles have arrived; it then syncs the controller to the
measured pose and adopts the matching normal/flipped side. It then runs one
cycle every `DT` (0.05 s, 20 Hz):

1. Sync the controller to the measured angles.
2. Read `/ARM/ACTION/ACTIVE`; while the action driver owns an action, stop
   publishing targets and skip the controller step.
3. Read the latest `/ARM/CMD_VELOCITY` and convert it to per-cycle deltas,
   unless gripper drive is active or the teleop source went quiet.
4. Advance the mode state machine via `RoverArmController.update()`.
5. Publish the rate-limited 5-DOF joint targets, planner state, and pose.

It owns the 5-DOF operator behavior that needs IK and limits: position vs
joint drive, normal / flipped / planar limit modes, joint locks, position /
orientation locks, orientation guide, and target publishing. Homing, flips,
zenith, and resting motion live in `arm_action_driver_node`; the planner
mirrors the active action's mode flags and locks (forced base/twist locks in
zenith, planar during resting) so gating and telemetry stay coherent. When
`/ARM/ACTION/ACTIVE` falls back to idle, the planner re-syncs its targets to
the measured pose before publishing again, so the arm never jumps back to a
pre-action target. It is the only writer of the five arm targets outside an
action; the end effector node owns the gripper and solenoid. Unchanged
targets are not republished every cycle, but are refreshed every 20 cycles so
late consumers still receive them; a locked joint republishes its last
commanded target instead of echoing encoder noise. Command rejections and
runtime failures are reported in `/ARM/PLANNER/STATUS`, and the master
republishes them on `/ARM/STATE/ERROR` so that topic has one writer.

---

### `arm_action_driver_node`

Owns the discrete, mostly Cartesian-free actions that used to run inside the
planner: homing (full and single-joint), side flips, the zenith test pose, and
the resting collapse/de-torque/restore sequence. It consumes the measured
joint angles and publishes the five joint targets **only while an action is
active**, plus:

- `/ARM/ACTION/ACTIVE` (`std_msgs/Int8`: IDLE/HOMING/FLIPPING/ZENITH/RESTING)
  so the planner knows when to stop publishing.
- `/ARM/ACTION/STATUS` (`custom_interfaces/ArmActionStatus`: ready, active,
  phase, progress, homing mask, error event).
- `/ARM/HARDWARE/RESTING` and `/ARM/HARDWARE/TORQUE_ENABLED`, the resting
  latches the hardware consumes.

All moves use one shared positional trapezoidal profile
(`ACTION_ACCEL_RAD_S2` / `ACTION_DECEL_RAD_S2`) and one cruise speed owned
by the action driver node (`MOVE_SPEED_RAD_S`, 25 deg/s for every joint). Flip
is a maintained action: `flip` starts it,
`flip_release` pauses it (`ACTIVE` returns to IDLE, so teleop is available),
re-pressing resumes toward the remembered goal with a freshly planned safe
path, and any manual velocity clears the goal. Homing is also maintained:
`home_release` (or the `HOMING_HOLD_TIMEOUT_SEC` deadman after the last
`home`) stops it immediately. Zenith is hold-to-run while it drives: its
key-up event `zenith_release` stops the move and holds the achieved pose.
Resting stays a toggle, so releasing its key does not undo the mode it
selected. Zenith holds base and wrist twist, and the planner force-locks
both while it runs; homing from zenith skips the exit move. Resting requires
planar mode (or a resting-region startup pose). E-stop freezes target
publishing; the hardware position-locks the measured pose.

The node itself is a thin shell: each action is an `ActionConfiguration`
subclass (`arm_action_driver/actions/{homing,flip,zenith,resting}.py`)
that declares its trigger/release commands, active enum, motion
limits, and hold/toggle flags.  The node owns the move and resting
engines (gating, release/adopt handling, ramp stepping, progress); a
parameter block never implements lifecycle behaviour.  The node routes
commands, enforces ownership gating, and publishes status from the
registry (`arm_action_driver/actions/__init__.py`) alone. Adding an
action is creating a class and appending it to the registry - see
`arm_action_driver/actions/base.py`.

---


### `arm_hardware_node`

The real-hardware CAN boundary. It receives joint targets and routes each one
to the backend that owns that motor:

- **OpenCAN** (base, shoulder, elbow) via `open_can_handler.py`, which handles
  tick conversion, enable/speed frames, and position reads.
- **Standard CAN** (wrist bend, wrist twist, gripper, solenoid) directly in the
  node, using the `MOVE_WRIST`, `MOVE_GRIPPER`, and `MOVE_SOLENOID` frames.

Targets are validated against `angle_limits()` and dropped when the arm is
disabled. `/ESTOP` latches the arm off and blocks enable requests until it
clears, but it does not remove OpenCAN torque: the handler sends every measured
position as the new target so the drives hold the pose. The enable level is
re-asserted on every tick so a dropped frame cannot leave the motors latched
off. OpenCAN positions are
requested at `DT`, and feedback is published even while the arm is disabled so
the planner can initialize. A solenoid strike sends one
`MOVE_SOLENOID.Enabled=1` frame; the Teensy retracts it, so there is no disable
command. `destroy_node()` makes a best-effort attempt to disable the motors.

---


### `arm_camera_node`

Placeholder only: it creates a node and spins, with no publishers or
subscribers. It is included in `arm_launch.xml` so the launch composition does
not change when the camera is implemented.

---


### `arm_sniffer_node`

Read-only diagnostic tool, deliberately excluded from every launch file. It
subscribes to the current/target joint topics plus the arm state and
E-stop topics and prints one snapshot per second. See
[Debugging](#debugging).

---


### Composition roots

`arm_launch.xml` is the standalone entry point. On the real rover the
composition root is `src/main/launch/main_launch.xml`, which starts the
teleop, route planner, action driver, master, end effector, and hardware
nodes. In simulation,
`src/simulation/rover_simulation/launch/simulation_launch.xml` starts the
Gazebo bridge, teleop node, route planner, action driver, master, and end
effector; the bridge replaces `arm_hardware_node` and speaks the same
target/current topics.

## Data flow

```
  spacemouse ──► arm_teleop ──/ARM/MANUAL/CMD_VELOCITY──┐
                                                        │
  keyboard ──► arm_master ──────────────────────────────┤
 (basestation)     ▲  │                                 │
                   │  ├──/ARM/CONTROL/REQUEST            │
  plane + keys ────┘  │                                 │
  arm_typing ─────────┘                                 │
      │  /ARM/AUTONOMOUS/CMD_POSE                       │
      │  /ARM/AUTONOMOUS/COMMAND                        │
      │                                                 ▼
      │            /ARM/CMD_VELOCITY ────────► arm_route_planner
      │            /ARM/CMD_POSE ────────────►        │ /ARM/JOINT/TARGET_{5}
      │            /ARM/PLANNER/COMMAND ─────►        │ /ARM/STATE/*
      │            /ARM/ACTION/COMMAND ──────► arm_action_driver
      │                                               │ /ARM/JOINT/TARGET_{5}
      │                                               │ /ARM/ACTION/{ACTIVE,STATUS}
      │                                               │
      │            /ARM/STATE/CONTROL_SOURCE
      │                  (diagnostic, transient-local)
      │            /ARM/END_EFFECTOR/COMMAND
      ▼                                               ▼
  chassis ◄──/BASESTATION/xbox/AXIS_LY + AXIS_RY   arm_end_effector
                                                    │ /ARM/JOINT/TARGET_GRIPPER
                                                    │ /ARM/JOINT/TARGET_SOLENOID
                                                    └──/ARM/STATE/ERROR (master)

                         arm_hardware_node ──CAN──► Teensy
                                             ▲        motors
                                             └─/ARM/JOINT/CURRENT_*
```

`arm_master_node` is the only external command entry point, the only mux, and
the only writer of `/ARM/STATE/ERROR`. `arm_teleop_node` publishes manual
intent to the master rather than straight to the planner, so the master can
arbitrate it against the keyboard stream and the control source. The planner
and end effector consume only the master's arbitrated topics. The planner's
five joint targets and the end effector's gripper target both feed
`arm_hardware_node`.

In simulation `arm_hardware_node` is swapped for
`rover_simulation/arm_sim_node`. It exposes the same target, enable, E-stop,
solenoid, and current-angle topics, but sends joint positions to the Gazebo
`arm_cont` controller instead of the CAN motors.

## Control ownership

`/ARM/STATE/CONTROL_SOURCE` (`std_msgs/UInt8`) says who currently drives the
arm. The arm master is its single writer and the only reader that gates on
it. It publishes with transient-local QoS so a node that starts late still
sees the current source.

| Value | Source | Meaning |
| --- | --- | --- |
| `0` | `NONE` | No source may move the arm. |
| `1` | `MANUAL` | Teleop, keyboard, and gripper jog. Boot default. |
| `2` | `AUTONOMOUS` | An autonomous controller owns the arm. |

Any source requests a change by publishing the value on
`/ARM/CONTROL/REQUEST` (`std_msgs/UInt8`). The master applies this policy:

- `MANUAL` and `NONE` are always accepted, so the operator can always
  reclaim or release the arm.
- `AUTONOMOUS` is accepted only when the planner is alive, ready, and the
  E-stop is clear. Refusals are logged and reported as
  `ARM_ERROR_REJECTED`.
- Unknown values are ignored.

Ownership is enforced in one place: the master drops manual velocity and
manual commands while another source owns the arm, and publishes a zero
velocity, planner `hold`, and `stop_jog` on every handover. The planner and
end effector consume only the master's arbitrated topics, so they stay
source-agnostic. `arm_typing_node` is the first autonomous source; it uses
`/ARM/AUTONOMOUS/CMD_POSE` and `/ARM/AUTONOMOUS/COMMAND` rather than the
manual surfaces.

## Debugging

`arm_sniffer_node` is a read-only diagnostic printer. It subscribes to the
current/target joint topics plus the arm state topics and prints one compact
snapshot per second. It is deliberately excluded from every
launch file, so it only runs when started by hand:

```sh
ros2 run arm arm_sniffer_node   # canonical
ros2 run arm sniffer            # short alias
ros2 run arm ASN                # mnemonic alias
```

Output is an aligned status/flags block and a joint table, for example:

```text
ARM   enabled : True                flags  : PLANAR
      e-stop  : False               locked : none
      control : MANUAL              homing : none
      mode    : PICK_AND_PLACE (0)  error  : none
      joint        cur(deg) trt(deg) dlt(deg)
      base            12.34    12.50     0.16
      shoulder       -45.00   -45.00     0.00
      elbow           90.00    90.00     0.00
      wrist_bend       0.00     0.00     0.00
      wrist_twist      0.00     0.00     0.00
      gripper          0.10     0.10     0.00
```

## Topics

`*` below means one topic per joint: `BASE`, `SHOULDER`, `ELBOW`,
`WRIST_BEND`, `WRIST_TWIST`, `GRIPPER`.

### Ownership at a glance

| Node | Publishes | Subscribes |
| --- | --- | --- |
| `arm_teleop_node` | `/ARM/MANUAL/CMD_VELOCITY` | `/BASESTATION/spacemouse/axes` |
| `arm_typing_node` | `/ARM/CONTROL/REQUEST`, `/ARM/AUTONOMOUS/CMD_POSE`, `/ARM/AUTONOMOUS/COMMAND`, `/BASESTATION/xbox/AXIS_LY`, `/BASESTATION/xbox/AXIS_RY`, `/ARM/TYPING/STATUS` | `/ARM/TYPING/PLANE`, `/ARM/TYPING/KEYS`, `/ARM/STATE/CONTROL_SOURCE`, `/ARM/PLANNER/STATUS`, `/ARM/STATE/POSE`, `/ESTOP` |
| `arm_master_node` | `/ARM/CMD_VELOCITY`, `/ARM/CMD_POSE`, `/ARM/PLANNER/COMMAND`, `/ARM/ACTION/COMMAND`, `/ARM/END_EFFECTOR/COMMAND`, `/ARM/STATE/CONTROL_SOURCE`, `/ARM/STATE/ERROR` | `/ESTOP`, `/ARM/COMMAND`, `/ARM/MANUAL/CMD_VELOCITY`, `/ARM/AUTONOMOUS/CMD_POSE`, `/ARM/AUTONOMOUS/COMMAND`, `/ARM/CONTROL/REQUEST`, `/ARM/PLANNER/STATUS`, `/ARM/ACTION/STATUS` |
| `arm_end_effector_node` | `/ARM/JOINT/TARGET_GRIPPER`, `/ARM/JOINT/TARGET_SOLENOID` | `/ESTOP`, `/ARM/END_EFFECTOR/COMMAND`, `/ARM/JOINT/CURRENT_GRIPPER` |
| `arm_route_planner_node` | `/ARM/JOINT/TARGET_*` (5-DOF), planner `/ARM/STATE/*` topics, `/ARM/STATE/POSE`, `/ARM/PLANNER/STATUS` | `/ESTOP`, `/ARM/PLANNER/COMMAND`, `/ARM/ACTION/ACTIVE`, `/ARM/ACTION/STATUS`, `/ARM/CMD_VELOCITY`, `/ARM/JOINT/CURRENT_*` (5-DOF) |
| `arm_action_driver_node` | `/ARM/JOINT/TARGET_*` (5-DOF while an action is active), `/ARM/ACTION/ACTIVE`, `/ARM/ACTION/STATUS`, `/ARM/HARDWARE/RESTING`, `/ARM/HARDWARE/TORQUE_ENABLED` | `/ESTOP`, `/ARM/ACTION/COMMAND`, `/ARM/MANUAL/CMD_VELOCITY`, `/ARM/JOINT/CURRENT_*` (5-DOF), `/ARM/STATE/{ENABLED,PLANAR,FLIPPED,LOCKED_JOINTS,TORQUE_ENABLED}` |
| `arm_hardware_node` | `/ARM/JOINT/CURRENT_*`, CAN frames to the Teensy | `/ARM/JOINT/TARGET_*`, `/ARM/JOINT/TARGET_SOLENOID`, `/ARM/STATE/ENABLED`, `/ESTOP`, CAN frames from the Teensy |
| `arm_sniffer_node` | — | currents, targets, state flags, `/ARM/STATE/ENABLED`, `/ARM/STATE/CONTROL_SOURCE`, `/ESTOP` |
| `arm_camera_node` | — | — |

### `arm_teleop_node`

Inputs:

| Topic | Type | Notes |
| --- | --- | --- |
| `/BASESTATION/spacemouse/axes` | `custom_interfaces/SpaceMouseAxes` | Raw HID axes (`BEST_EFFORT`). One atomic sample of all six axes, republished every basestation cycle so the teleop node can detect a dead link. |

Outputs:

| Topic | Type | Notes |
| --- | --- | --- |
| `/ARM/MANUAL/CMD_VELOCITY` | `custom_interfaces/Velocity` | Manual teleop intent. The master arbitrates it onto `/ARM/CMD_VELOCITY`. |

The node applies axis signs, deadzone with hysteresis, the power curve, a
vector-magnitude limit, and sensitivity once, then republishes the latched
command at a fixed rate (`TELEOP_PUBLISH_RATE_HZ`, default 50 Hz), including
all-zero commands. This keeps the master's input fresh while the stick is
idle. An axis whose raw value has not refreshed for
`TELEOP_INPUT_TIMEOUT_SEC` is
commanded as zero, so a dead link cannot leave the arm moving. It handles
teleop only; the master muxes this stream with the keyboard and any future
autonomous source.
`custom_interfaces/Pose` is the absolute target type. The master forwards
it on `/ARM/CMD_POSE` only while an autonomous source owns the arm.

### `arm_master_node`

Inputs:

| Topic | Type | Notes |
| --- | --- | --- |
| `/ESTOP` | `std_msgs/Bool` | While asserted, commands are ignored and the planner health check pauses. |
| `/ARM/COMMAND` | `std_msgs/String` | One high-level action per message. Manual command surface. |
| `/ARM/MANUAL/CMD_VELOCITY` | `custom_interfaces/Velocity` | Manual teleop intent. Forwarded only while `MANUAL` owns the arm. |
| `/ARM/AUTONOMOUS/CMD_POSE` | `custom_interfaces/Pose` | Autonomous absolute target. Forwarded only while `AUTONOMOUS` owns the arm; silence for `AUTONOMOUS_POSE_TIMEOUT_SEC` sends a planner hold. |
| `/ARM/AUTONOMOUS/COMMAND` | `custom_interfaces/ArmCommand` | Autonomous gripper/solenoid or planner commands. Forwarded only while `AUTONOMOUS` owns the arm. |
| `/ARM/CONTROL/REQUEST` | `std_msgs/UInt8` | Request control ownership: `0` none, `1` manual, `2` autonomous. |
| `/ARM/PLANNER/STATUS` | `custom_interfaces/ArmPlannerStatus` | Planner readiness and the latest command/runtime rejection. |

`/ARM/COMMAND` accepts:
`flip`, `zenith` / `toggle_zenith`, `resting` / `toggle_resting`,
`planar` / `toggle_planar`,
`pnp_normal`, `toggle_flipped`,
`homing` / `home`, `joint_drive`, `position_drive`, `sync` / `sync_target`,
`toggle_position_lock`, `toggle_orientation_lock`, `guide_orientation`,
`toggle_joint_lock:<0-4>`, `home_joint:<0-4>`, `gripper_up`,
`gripper_down`, and `fire_solenoid` (`toggle_solenoid` is a legacy alias).
The resting/zenith toggles bypass the master's manual-only gate (they are
global safety commands, not motion intent), and `flip_release`,
`zenith_release`, and `home_release` are always forwarded so an interrupted
hold-to-run action can stop. The other motion
commands are forwarded to the route planner for validation; `flip`, `home`/
`homing`, `home_joint:<n>`, zenith, and resting go to the action driver;
`gripper_up`, `gripper_down`, `fire_solenoid`, and `toggle_solenoid` go only
to the end effector. See the module docstring in `arm_master_node.py` for the
routing rules.

`/ARM/COMMAND` also carries keyboard actions. `basestation_node` decodes the
basestation's per-action press states (0 idle, 1 just pressed, 2 held, 3
just released). Edge-triggered actions publish on just-pressed; the held
gripper arrows and homing keys (`home`, `home_joint:<0-4>`) publish on press
and on every held state so the end effector can jog and the route planner
keeps homing until the stream stops. On just-released, hold-to-run actions
publish their key-up event (`flip_release`, `zenith_release`,
`home_release`) so motion stops as soon as the operator lets go. Resting
remains a toggle with no release command.
The basestation's `ARM_STATE_COMMANDS` dict is the single source for the
signal-to-command mapping, with `ARM_STATE_HELD_COMMANDS` marking the
actions that repeat while held and `ARM_STATE_RELEASE_COMMANDS` marking the
actions that stop on key-up.
The physical key map (including the `Z` key for zenith) lives in the
basestation firmware. Zenith arrives on the appended `arm_state.zenith`
signal, after `solenoid_strike`, so existing signal offsets do not move; the
firmware must map `Z` to that signal.

Outputs:

| Topic | Type | Notes |
| --- | --- | --- |
| `/ARM/CMD_VELOCITY` | `custom_interfaces/Velocity` | Arbitrated teleop velocity. Zeroed on every handover and E-stop; requires a neutral stick before manual motion resumes. |
| `/ARM/CMD_POSE` | `custom_interfaces/Pose` | Arbitrated absolute target from an autonomous source. |
| `/ARM/PLANNER/COMMAND` | `custom_interfaces/ArmCommand` | Motion commands, for planner validation. Includes the internal `hold` sent on handover/E-stop and before an action starts. |
| `/ARM/ACTION/COMMAND` | `custom_interfaces/ArmCommand` | `flip`, `flip_release`, `home`/`homing`, `home_release`, zenith/`zenith_release`, and resting, for the action driver. |
| `/ARM/END_EFFECTOR/COMMAND` | `custom_interfaces/ArmCommand` | Only the gripper/solenoid commands, gated on E-stop and readiness. Includes the internal `stop_jog` sent on a handover. |
| `/ARM/STATE/CONTROL_SOURCE` | `std_msgs/UInt8` | Authoritative control owner (`0` none, `1` manual, `2` autonomous). Diagnostic; published transient-local. |
| `/ARM/STATE/ERROR` | `std_msgs/UInt32` | Last rejection, packed as `(seq << 16) \| (code << 8) \| arg`; codes are in `rover_protocol.arm_errors`. |

The master runs its health cycle every `DT` (0.05 s, 20 Hz). It ignores
commands while E-stopped; solenoid strikes are forwarded even before the arm
initializes, while the gripper jog requires fresh planner readiness. The
planner and action driver report command and runtime rejections in
`/ARM/PLANNER/STATUS` and `/ARM/ACTION/STATUS`; the master republishes them
on `/ARM/STATE/ERROR`; identical runtime codes are suppressed until the
condition changes, while each rejected command forces a new sequence number.
Rejections of a held gripper arrow are throttled to one report per second.

### `arm_end_effector_node`

Inputs:

| Topic | Type | Notes |
| --- | --- | --- |
| `/ESTOP` | `std_msgs/Bool` | Freezes the gripper target while asserted. |
| `/ARM/END_EFFECTOR/COMMAND` | `custom_interfaces/ArmCommand` | `gripper_up`, `gripper_down`, `fire_solenoid`, `toggle_solenoid`, or the internal `stop_jog`. |
| `/ARM/JOINT/CURRENT_GRIPPER` | `std_msgs/Float32` | Measured gripper angle [rad]. |

Outputs:

| Topic | Type | Notes |
| --- | --- | --- |
| `/ARM/JOINT/TARGET_GRIPPER` | `std_msgs/Float32` | Gripper target [rad]. Tracks the measured angle unless a jog is active. |
| `/ARM/JOINT/TARGET_SOLENOID` | `std_msgs/Bool` | Momentary solenoid strike; one `true` pulse per command. The Teensy fires and retracts the solenoid itself, so the rover never sends a disable. |

Each `gripper_up` / `gripper_down` command extends the jog deadline; the node
moves the target at `GRIPPER_JOG_RATE` per cycle and stops after
`GRIPPER_JOG_TIMEOUT_SEC` (0.25 s) without a new command. Up opens (toward
0 rad), down closes (toward 90 degrees).

### `arm_route_planner_node`

Inputs:

| Topic | Type | Notes |
| --- | --- | --- |
| `/ESTOP` | `std_msgs/Bool` | Pauses the planner while asserted. |
| `/ARM/PLANNER/COMMAND` | `custom_interfaces/ArmCommand` | Forwarded high-level commands. |
| `/ARM/ACTION/ACTIVE` | `std_msgs/Int8` | Nonzero while the action driver owns the arm; the planner stops publishing targets then. |
| `/ARM/ACTION/STATUS` | `custom_interfaces/ArmActionStatus` | Supplies the homing-joint mask while the driver homes. |
| `/ARM/CMD_VELOCITY` | `custom_interfaces/Velocity` | Latest teleop velocity; converted to per-cycle deltas at `DT`. |
| `/ARM/CMD_POSE` | `custom_interfaces/Pose` | Absolute Cartesian target; takes priority over teleop while fresh. |
| `/ARM/JOINT/CURRENT_*` | `std_msgs/Float32` | Measured 5-DOF joint angles [rad]; the planner initializes once all five arrive. |

Outputs:

| Topic | Type | Notes |
| --- | --- | --- |
| `/ARM/JOINT/TARGET_*` | `std_msgs/Float32` | Rate-limited 5-DOF joint targets [rad] while no action owns the arm. Unchanged targets are skipped, with a forced refresh every 20 cycles. |
| `/ARM/STATE/HOMING` … `/ARM/STATE/RESTING` | `std_msgs/Bool` | Planner flags: `HOMING`, `FLIPPING`, `FLIPPED`, `PLANAR`, `JOINT_DRIVE`, `POSITION_LOCK`, `ORIENTATION_LOCK`, `ZENITH`, `RESTING`. `FLIPPED` and `ZENITH` track the measured pose region; `HOMING`/`FLIPPING` follow the action driver. |
| `/ARM/STATE/LOCKED_JOINTS` | `std_msgs/UInt8` | Bitmask: bit `i` set means joint `i` is motor-locked (base/twist are forced on during zenith). |
| `/ARM/STATE/HOMING_JOINTS` | `std_msgs/UInt8` | Bitmask: bit `i` set means joint `i` is being homed (from the driver status). |
| `/ARM/STATE/OPERATION_MODE` | `std_msgs/UInt8` | Wire code: `0` PNP, `1` PNP flipped, `2` planar, `3` homing, `4` zenith, `5` resting. |
| `/ARM/STATE/POSE` | `custom_interfaces/Pose` | Measured end-effector pose (metres/radians). |
| `/ARM/PLANNER/STATUS` | `custom_interfaces/ArmPlannerStatus` | Readiness plus the latest command/runtime rejection code. |

The planner waits until all five measured angles arrive before it starts; it
then runs one control cycle every `DT` (0.05 s, 20 Hz). Runtime rejections
(e.g. jogging into a restricted octant) repeat every cycle, so identical
codes are suppressed until the condition changes; each rejected command
still forces a new sequence number. Path searches run inside the command
callback, so a long homing or flip plan can delay planner status briefly;
the master only declares the planner dead after `PLANNER_TIMEOUT_SEC`
(5 s).

### `arm_action_driver_node`

Inputs:

| Topic | Type | Notes |
| --- | --- | --- |
| `/ESTOP` | `std_msgs/Bool` | Freezes target publishing while asserted. |
| `/ARM/ACTION/COMMAND` | `custom_interfaces/ArmCommand` | `flip`, `flip_release`, `home`/`homing`, `home_release`, `home_joint:<n>`, zenith/`zenith_release`, resting, and the internal `adopt`. |
| `/ARM/MANUAL/CMD_VELOCITY` | `custom_interfaces/Velocity` | Any nonzero sample clears a remembered flip goal and pauses a running flip. |
| `/ARM/JOINT/CURRENT_*` | `std_msgs/Float32` | Measured 5-DOF joint angles [rad]. |
| `/ARM/STATE/ENABLED` | `std_msgs/Bool` | Resting push precondition. |
| `/ARM/STATE/PLANAR` | `std_msgs/Bool` | Resting is accepted only in planar (or an inferred resting pose). |
| `/ARM/STATE/FLIPPED` | `std_msgs/Bool` | Selects the remembered side for homing and the zenith exit. |
| `/ARM/STATE/LOCKED_JOINTS` | `std_msgs/UInt8` | Joints held during homing and flips. |
| `/ARM/STATE/TORQUE_ENABLED` | `std_msgs/Bool` | Hardware torque latch, for the resting restore handshake. |

Outputs:

| Topic | Type | Notes |
| --- | --- | --- |
| `/ARM/JOINT/TARGET_*` | `std_msgs/Float32` | 5-DOF targets while an action is active; nothing is published while idle. |
| `/ARM/ACTION/ACTIVE` | `std_msgs/Int8` | `0` idle, `1` homing, `2` flipping, `3` zenith, `4` resting. |
| `/ARM/ACTION/STATUS` | `custom_interfaces/ArmActionStatus` | Readiness, active enum, phase, progress, homing mask, and the latest rejection. |
| `/ARM/HARDWARE/RESTING` | `std_msgs/Bool` | Widen shoulder/elbow targets to `RESTING_LIMITS`; refreshed at 1 Hz. |
| `/ARM/HARDWARE/TORQUE_ENABLED` | `std_msgs/Bool` | OpenCAN torque intent; sticky off until an explicit `True`. |

### `arm_hardware_node`

Inputs:

| Topic / frame | Type | Notes |
| --- | --- | --- |
| `/ARM/JOINT/TARGET_*` | `std_msgs/Float32` | Joint targets [rad]; ignored unless the arm is enabled. |
| `/ARM/JOINT/TARGET_SOLENOID` | `std_msgs/Bool` | `true` fires one strike; ignored unless enabled. |
| `/ARM/STATE/ENABLED` | `std_msgs/Bool` | Arm enable level from `master_node`; re-asserts OpenCAN enable frames every tick. |
| `/ESTOP` | `std_msgs/Bool` | Level-triggered stop; latches the arm off until it clears. |
| Arm CAN frames addressed to the Jetson | `custom_interfaces/Can` | Position feedback (`READ_*`) from the motors. |

Outputs:

| Topic / frame | Type | Notes |
| --- | --- | --- |
| `/ARM/JOINT/CURRENT_*` | `std_msgs/Float32` | Measured joint angles [rad]; published even while the arm is disabled. |
| Arm CAN frames addressed to the Teensy | `custom_interfaces/Can` | `SEND_BASE`, `SEND_SHOULDER`, `SEND_ELBOW`, `MOVE_WRIST`, `MOVE_GRIPPER`, `MOVE_SOLENOID`, plus OpenCAN enable/speed/read frames. |

Targets are checked against `angle_limits()` and duplicate targets are
dropped before reaching the backend. OpenCAN motors are polled for position
at `DT`; standard-CAN feedback is decoded from the incoming frames. CAN topic
names and payloads are defined in `src/constants/constants/CAN_constants.py`
and `lib/rovers-protocol/protocol.yaml`.

### `arm_sniffer_node`

Subscribes to `/ARM/JOINT/CURRENT_*`, `/ARM/JOINT/TARGET_*`, every
`/ARM/STATE/*` topic, and `/ESTOP`, and prints one snapshot per second. It
publishes nothing and is never launched automatically.

### `arm_camera_node`

No topics yet; it only constructs and spins.

### Simulation

`rover_simulation/arm_sim_node` replaces `arm_hardware_node`: it subscribes
to the same `/ARM/JOINT/TARGET_*`, `/ARM/JOINT/TARGET_SOLENOID`,
`/ARM/STATE/ENABLED`, and `/ESTOP` topics and publishes
`/ARM/JOINT/CURRENT_*`. Internally it works in encoder ticks using the same
`ARM_MOTOR_PARAMS` calibration, tick limits, and per-motor speed limits as
`arm_hardware_node`, converts to radians only at the Gazebo/ROS boundary,
and sends the six-joint position command to the Gazebo `arm_cont`
controller. It republishes the Gazebo joint states as arm feedback. The
master, planner, teleop, and basestation topics are identical between
simulation and hardware.

## Controller behaviour

- **Position drive (default):** stick translation/rotation moves the
  target frame; the frame is clamped by the active limiter, solved with
  damped-least-squares IK. Teleop does not rebuild the command from the
  measured angles: the planner ramps the Cartesian velocity
  (`TELEOP_ACCEL_IN_S2` / `TELEOP_DECEL_IN_S2`), the controller integrates an
  internal joint reference, and the per-joint speed cap and accel limit are
  applied to the whole joint vector so the path keeps its direction. The
  reference may lead the measured arm by at most `TELEOP_LEAD_LIMIT_DEG`;
  a longer lag stops the stage and reports a tracking stall. Releasing the
  stick ramps to a stop in about half a second, then holds.
- **Joint drive:** `rx → elbow`, `ry → shoulder`, `rz → base`; no IK. The
  planner applies the joint limits and keeps the older per-joint slew.
- **Absolute pose:** `/ARM/CMD_POSE` replaces the target frame and runs the
  same IK/limit path. It takes priority over teleop while the target stream
  is fresh; the master's stale-target hold stops the chase.
- **Limit modes:** Pick-and-Place normal and flipped, Planar (base lock),
  Typing (base limited to +/-90 deg, wrist locks released), Zenith
  (straight-up test pose), Resting (collapse/de-torque with padded
  shoulder/elbow travel), and Homing. Restricted octants (3, 5, 7) require
  Planar.
- **Zenith mode:** the `Z` key commands `[any base, 90, 0, 0, 0]` degrees, a
  straight-up pose suitable for depowering the motors. It releases joint
  locks while active, ignores teleop, and on the next `Z` press moves to the
  nearest valid pose on the last Pick-and-Place side before restoring that
  side and its locks. Other mode commands, homing, and flips are refused
  until zenith is exited.
- **Resting:** the `R` key collapses the arm for transport: move to the
  crouch `[0, 0, 135, 0, 0]` degrees along a planned path at the shared
  action speed, push the shoulder toward its lower stop and the elbow toward
  its upper stop at 1 deg/s until each stalls, back each off 0.5 degrees,
  then remove torque from the three OpenCAN motors. The next `R` restores
  torque at the measured pose, returns to the crouch, hands off to normal
  homing, and ends in Pick-and-Place with the operator's saved joint locks
  restored. R mid-sequence aborts and holds
  with torque on; an E-stop during resting latches torque off until the
  restore. While resting, other mode commands, homing, flips, joint locks,
  and teleop are rejected, and the shoulder/elbow targets are validated
  against the padded `RESTING_LIMITS` instead of the normal limits. The
  planner refreshes the `/ARM/HARDWARE/RESTING` and
  `/ARM/HARDWARE/TORQUE_ENABLED` levels at 1 Hz; the hardware times the
  resting latch out after 1.5 s of silence while the torque latch stays off
  until an explicit re-enable. Details: `planz/RESTING_FEATURE_PLAN.md`.
- **Startup mode inference:** on the first measured sync the planner selects
  the mode that matches the pose instead of assuming normal Pick-and-Place.
  The straight-up pose selects Zenith; a resting-region pose (shoulder below
  the planar bound or elbow past its upper limit) selects Resting and keeps
  torque off until the restore; restricted octants (notably top-back)
  select Planar when the base is inside the planar band; otherwise the
  Pick-and-Place side whose limits fit is adopted, falling back to the side
  with the smaller excursion. The inferred mode is published on
  `/ARM/STATE/OPERATION_MODE` and its flag topic so the basestation
  display matches the arm; `telemetry_uplink_node` mirrors the mode, locks,
  and flags into `arm_status` as well as `arm_encoders`, so the basestation
  stored state updates even before every joint angle has arrived.
- **Actions:** `flip`, `zenith`, `planar`, `pnp_normal`, `typing_mode` /
  `typing_off`, `homing`, `joint_drive`, `guide_orientation`, and joint
  locks run in the route planner. `homing` and `home_joint:<n>` are
  maintained actions: keep the key held, and the planner stops the move when
  the command stream goes quiet. `typing_mode` is normally sent by
  `arm_typing_node` on the autonomous command surface, not by the operator.
  `gripper_up`, `gripper_down`, and `fire_solenoid` stay in the end effector
  node. The full command list and syntax are documented in the module
  docstrings of `arm_master_node.py` and `arm_route_planner_node.py`.
- **Slew model:** `SimulatedMechanicalArm` is still used by homing, flips,
  joint drive, and absolute pose, so those targets never jump by more than
  each joint's hardware speed times `DT`. Teleop uses the coordinated output
  stage above instead. The per-joint speeds come from `max_speed`
  (motor-shaft RPM) divided by the 80:1 gearbox, then converted to rad/s and
  ticks/s with the joint calibration. The planner owns the model because it is
  the sole writer of the five arm targets.
- **Watchdogs:** teleop motion stops after `VELOCITY_TIMEOUT_SEC` (0.5 s)
  applied to `/ARM/CMD_VELOCITY` in the planner, and the teleop node zeroes
  an axis whose raw input stopped refreshing for `TELEOP_INPUT_TIMEOUT_SEC`
  (0.5 s). A gripper jog stops after `GRIPPER_JOG_TIMEOUT_SEC` (0.25 s)
  without a new arrow command, and homing stops after
  `HOMING_HOLD_TIMEOUT_SEC` (0.25 s) without a fresh home command. The master
  treats the planner as unavailable after `PLANNER_TIMEOUT_SEC` (5 s) without
  status and reports `ARM_ERROR_REJECTED`. The planner runs path searches inside its command
  callback, so a long homing or flip plan can delay status; the 5 s watchdog
  covers that gap. An autonomous pose target stops after
  `AUTONOMOUS_POSE_TIMEOUT_SEC` (0.5 s) without a refresh, and
  `arm_typing_node` stops the drivebase when its plane pose goes stale or it
  loses ownership.

## Arm constants

`arm_constants.py` is the single source of truth for all arm constants:
joint order and ROS topic names, `ARM_MOTOR_IDX` and the per-motor
calibration table (`ARM_MOTOR_PARAMS`), mechanical limits, home pose, arm
geometry, CAN mappings, and controller tuning (IK, octants, path planning,
waypoint following, SpaceMouse axis map and sensitivity).  It also holds the
node-level defaults that must stay consistent across processes: teleop
publish rate, teleop/velocity/planner timeouts, the gripper-jog rejection
throttle, the manual re-arm neutral epsilon, and the debug sniffer cadence.
The typing reference layout (`TYPING_KEYBOARD`, key units, pitch) and
the typing motion/safety tuning also live here, so the node and its geometry
tests share one definition.

Only node infrastructure stays local: ROS `QoSProfile` objects, the
gripper jog direction table tied to that node's command handling, and small
derived lookups such as the joint-name index.

It lives at the package root rather than under `src/` because colcon's
`--symlink-install` resolves `py_modules` from the package root only.  It is
deliberately named `arm_constants` so the workspace `constants` ament package
resolves normally alongside it; `arm_constants.py` imports
`constants.CAN_constants` and `constants.CommandCodes` directly.  The latter
is the existing bridge that puts the workspace `lib/rovers-protocol`
submodule on `sys.path` for `rover_protocol.arm_errors`.

## Hardware layer

`arm_constants.py` holds `ARM_MOTOR_IDX` (base, shoulder, elbow, bend
wrist, twist wrist, gripper) and `ARM_MOTOR_PARAMS`, the per-motor
`ticks_per_rad`, `tick_offset`, tick limits, and `max_speed`.  `max_speed`
is the drive profile velocity on the motor shaft in RPM; `max_output_rad_s`
and `max_ticks_per_s` divide by the 80:1 gearbox and convert through the
joint calibration.  `ARM_MOTOR_MAX_ANG_VEL` is the resulting per-joint speed
list used by the planner and simulation.

The base and elbow joints use Tinsmith M4230-series integrated robotic joint
actuators: 32768 encoder counts per motor revolution through an 80:1 gearbox.
`TINSMITH_M4230_TICKS_PER_RAD` derives the resulting scale of about 417215
ticks per output radian, and both joints use it with the sign of their tick
direction. The shoulder uses a different actuator and keeps its own measured
calibration. Offsets and tick limits stay per-joint measurements and still
need hardware verification.

`open_can_handler.py` is joint agnostic and handles only the three
OpenCAN motors (base, shoulder, elbow): it packs and decodes single-frame
CANopen SDO transfers. The standard CiA 402 objects are controlword
`0x6040`, target position `0x607A`, profile velocity `0x6081`, target
velocity `0x60FF`, position `0x6064`, velocity `0x606C`, torque `0x6077`,
current `0x6078`, DC-link voltage `0x6079`, statusword `0x6041`, and error
code `0x603F`. It converts ticks using the calibration entry it is given,
can request diagnostic values (velocity, torque, voltage, status) and can
read vendor-specific objects through `request_object()` / `decode_raw()`.
Position writes reject non-finite targets and anything at or beyond one full
revolution. The named read helpers default to all three OpenCAN motors and
accept an optional `motor_idx` to poll just one. The public API is split into
named commands, `request()` / `request_all()` for reads, and matching
`decode()` / typed decoders; the low-level SDO frame packing stays private.
It has no wrist or gripper knowledge.

`arm_hardware_node.py` is the CAN transport and owns the standard-CAN
joints directly: it creates the outgoing publishers, exposes
`publish_can()`, `decode_can_packet()`, and the frame lock, and handles
the wrist bend, wrist twist, and gripper frames (`MOVE_WRIST`,
`MOVE_GRIPPER`).  The wrist axes are separate commands: one callback only
ever writes its own signal.

`open_can_handler.py` keeps every tick conversion and frame layout
internal, so `arm_hardware_node` only deals in radians. The node asks the
OpenCAN handler whether it owns a motor (`handles()` /
`resolve_message()`) and sends or decodes through it; standard-CAN joints
are dispatched from the node itself. All six joints are independent; the
two wrist angles are separate joints even though `MOVE_WRIST` carries
both signals.

Safety behaviour:

- The arm only moves while `/ARM/STATE/ENABLED` is true.
- `/ESTOP` is level-triggered: it latches off and ignores enable requests
  until it clears, but it keeps OpenCAN torque enabled by writing the
  measured positions as targets, so the arm holds its pose. `master_node`
  republishes the enabled level, so the arm re-arms once the stop clears.
- Motor feedback keeps flowing while the arm is disabled, so the
  planner can initialize and track the measured pose.
- `destroy_node()` makes a best-effort attempt to disable the motors.

## Build and run

```bash
cd <workspace>
colcon build --packages-select arm
source install/setup.bash
ros2 launch arm arm_launch.xml
```

Notes:

- The package uses a `src/` layout: `find_packages(where='src')` with an
  explicit `package_dir` mapping. `arm_constants.py` is a real top-level module
  at the package root; colcon's `--symlink-install` resolves `py_modules`
  there, not under `src/`.
- On the rover, prefer `ros2 launch main main_launch.xml` so the master
  node, CAN comms, and telemetry are included.
- For simulation, use the `rover_simulation` launch files with
  `robot_config:=arm_drivebase`.

## Tests

The kinematic/limiter tests are standalone scripts:

```bash
source install/setup.bash
cd src/subsystems/arm
PYTHONPATH=.:src:$PYTHONPATH python3 test/test_limiter_modes.py
PYTHONPATH=.:src:$PYTHONPATH python3 test/test_pick_and_place_configs.py
```

Lint checks are the standard ament ones and can be run with
`colcon test --packages-select arm`.
