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
│   ├── arm_master/
│   │   └── arm_master_node.py     # command router, readiness, errors
│   ├── arm_end_effector/
│   │   └── arm_end_effector_node.py # gripper target and solenoid
│   ├── arm_route_planner/
│   │   ├── arm_route_planner_node.py # IK, actions, 5-DOF target writer
│   │   └── kinematics/            # shared kinematic core (pure library)
│   │       ├── generic_frames.py  # symbolic FK + damped-least-squares IK
│   │       ├── limiters.py        # workspace limiters and octant gating
│   │       ├── simulated_arm.py   # RoverArmController + slew model
│   │       └── actions/           # homing and flip controllers
│   ├── arm_hardware/
│   │   ├── arm_hardware_node.py   # ROS ↔ CAN boundary, transport, standard CAN
│   │   └── open_can_handler.py    # OpenCAN: base, shoulder, elbow
│   └── arm_sniffer/               # opt-in debug node, not launched by default
│       └── arm_sniffer_node.py
└── test/                          # unit, limiter, and config tests
```

## Nodes

The package provides six runtime nodes plus an opt-in debug tool.
`arm_launch.xml` starts the first six and renames them to `arm_camera`,
`arm_teleop`, `arm_route_planner`, `arm_master`, `arm_end_effector`, and
`arm_hardware`.

| Executable | Standalone node name | Launched by default | Responsibility |
| --- | --- | --- | --- |
| `arm_teleop_node` | `arm_teleop_node` | yes | Owns SpaceMouse semantics; publishes normalized velocity and gripper commands. |
| `arm_route_planner_node` | `arm_route_planner_node` | yes | Owns 5-DOF target generation: IK, limit modes, flips, homing, locks, pose, and the arm target topics. |
| `arm_master_node` | `arm_master_node` | yes | Arm subsystem master: arbitrates control ownership, routes commands, watches planner readiness, and is the sole `/ARM/STATE/ERROR` writer. |
| `arm_end_effector_node` | `arm_end_effector_node` | yes | Owns the gripper target, gripper drive, and solenoid strike. |
| `arm_hardware_node` | `arm_hardware_node` | yes | Owns the CAN link: sends targets, polls measured angles, enforces enable/E-stop. Replaced by `gazebo_bridge` in simulation. |
| `arm_camera_node` | `arm_camera_node` | yes | Placeholder; no logic yet. |
| `arm_sniffer_node` | `arm_sniffer_node` | no | Read-only diagnostic printer. |

### `arm_teleop_node`

The only node that knows the SpaceMouse exists. It subscribes to the six raw
`/BASESTATION/spacemouse/*` axes, applies the sign convention, deadzone with
entry hysteresis, power curve, vector-magnitude limit, and sensitivity, then
republishes the result as manual velocity on `/ARM/MANUAL/CMD_VELOCITY` at
`TELEOP_PUBLISH_RATE_HZ` (default 50 Hz). It republishes continuously so the
master always has a fresh sample.

- Axis processing happens exactly once, here; downstream nodes consume
  normalized values only.
- Translation is converted from inches-per-cycle to m/s at the ROS boundary.
- The three linear axes and the three rotation axes are each scaled so a
  diagonal deflection is no faster than a single-axis one.
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
`INPUT_EXPONENT`, `TELEOP_INPUT_TIMEOUT_SEC`). The node does not declare ROS
parameters because nothing overrides them; changing the constants requires a
rebuild, which matches the rest of the arm tuning.

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

Because the master emits the arbitrated stream, the planner and end effector
do not subscribe to `/ARM/STATE/CONTROL_SOURCE` and never need to know who owns
the arm.

It owns `/ARM/STATE/ERROR` and consumes `/ARM/PLANNER/STATUS` for readiness
and the latest planner error event. Commands are ignored while E-stopped;
solenoid strikes are allowed before the arm initializes, while the gripper
jog waits for planner readiness. Rejections of a held gripper arrow are
throttled to one report per second so the error banner stays readable. If
planner status stops arriving for `PLANNER_TIMEOUT_SEC` (5 s), the master
reports `ARM_ERROR_REJECTED` so the loss stays visible.

It is also the control arbiter and the single writer of
`/ARM/STATE/CONTROL_SOURCE`. Sources request ownership on
`/ARM/CONTROL/REQUEST`; the master validates the request (manual always wins,
autonomous needs a live and ready planner outside an E-stop) and publishes
the result with transient-local QoS. `/ARM/COMMAND` and
`/ARM/MANUAL/CMD_VELOCITY` are the manual surfaces, so they are ignored while
an autonomous source owns the arm.

A handover publishes a zero velocity on `/ARM/CMD_VELOCITY` and an internal
`stop_jog` command to the end effector, so the previous owner's last sample
cannot leak into the new owner's window. Manual velocity also requires a
neutral stick after every handover or E-stop before motion resumes.

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

### `arm_route_planner_node`

The 5-DOF state machine and the main consumer of joint feedback. It starts
uninitialized and stays that way until all five measured joint angles have
arrived, then builds `RoverArmController` from the measured pose and adopts
the matching normal/flipped side. It then runs one cycle every `DT` (0.05 s,
20 Hz):

1. Sync the controller to the measured angles.
2. Read the latest `/ARM/CMD_VELOCITY` and convert it to per-cycle deltas,
   unless gripper drive is active or the teleop source went quiet.
3. Advance the mode state machine via `RoverArmController.update()`.
4. Publish the rate-limited 5-DOF joint targets, planner state, and pose.

It owns every 5-DOF operator behavior: position vs joint drive, normal /
flipped / planar limit modes, homing, side flips, single-joint homing, joint
locks, position/orientation locks, orientation guide, and target publishing.
It is the sole writer of the five arm targets; the end effector node owns the
gripper and solenoid. Unchanged targets are not republished every cycle, but
are refreshed every 20 cycles so late consumers still receive them; a locked
joint republishes its last commanded target instead of echoing encoder
noise. Command rejections and runtime failures are reported in
`/ARM/PLANNER/STATUS`, and the master republishes them on
`/ARM/STATE/ERROR` so that topic has one writer.

### `arm_hardware_node`

The real-hardware CAN boundary. It receives joint targets and routes each one
to the backend that owns that motor:

- **OpenCAN** (base, shoulder, elbow) via `open_can_handler.py`, which handles
  tick conversion, enable/speed frames, and position reads.
- **Standard CAN** (wrist bend, wrist twist, gripper, solenoid) directly in the
  node, using the `MOVE_WRIST`, `MOVE_GRIPPER`, and `MOVE_SOLENOID` frames.

Targets are validated against `angle_limits()` and dropped when the arm is
disabled. `/ESTOP` latches the arm off, disables the OpenCAN motors, and blocks
enable requests until it clears; the enable level is re-asserted on every tick
so a dropped frame cannot leave the motors latched off. OpenCAN positions are
requested at `DT`, and feedback is published even while the arm is disabled so
the planner can initialize. A solenoid strike sends one
`MOVE_SOLENOID.Enabled=1` frame; the Teensy retracts it, so there is no disable
command. `destroy_node()` makes a best-effort attempt to disable the motors.

### `arm_camera_node`

Placeholder only: it creates a node and spins, with no publishers or
subscribers. It is included in `arm_launch.xml` so the launch composition does
not change when the camera is implemented.

### `arm_sniffer_node`

Read-only diagnostic tool, deliberately excluded from every launch file. It
subscribes to the current/target joint topics plus the arm state and
E-stop topics and prints one snapshot per second. See
[Debugging](#debugging).

### Composition roots

`arm_launch.xml` is the standalone entry point. On the real rover the
composition root is `src/main/launch/main_launch.xml`, which starts the
teleop, route planner, master, end effector, and hardware nodes. In
simulation,
`src/simulation/rover_simulation/launch/simulation_launch.xml` starts the
Gazebo bridge, teleop node, route planner, master, and end effector; the
bridge replaces `arm_hardware_node` and speaks the same target/current
topics.

## Data flow

```
  spacemouse ──► arm_teleop ──/ARM/MANUAL/CMD_VELOCITY──┐
                                                        │
  keyboard ──► arm_master ──────────────────────────────┤
 (basestation)     │  ▲                                 │
                   │  └──/ARM/CONTROL/REQUEST            │
                   │                                    ▼
                   │  /ARM/CMD_VELOCITY ──────► arm_route_planner
                   │  /ARM/PLANNER/COMMAND ───►        │ /ARM/JOINT/TARGET_{5}
                   │                                   │ /ARM/STATE/*
                   │  /ARM/STATE/CONTROL_SOURCE
                   │        (diagnostic, transient-local)
                   │  /ARM/END_EFFECTOR/COMMAND
                   ▼
             arm_end_effector ──/ARM/JOINT/TARGET_GRIPPER
                   │           ──/ARM/JOINT/TARGET_SOLENOID
                   │
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
`rover_simulation/gazebo_bridge`, which maps the same target/current topics
onto the Gazebo `arm_cont` position controller.

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
velocity plus `stop_jog` on every handover. The planner and end effector
consume only the master's arbitrated topics, so they stay source-agnostic.
The autonomous setpoint/command surface is the next step and should be a
separate input topic, fed into the same mux.

## Debugging

`arm_sniffer_node` is a read-only diagnostic printer. It subscribes to the
current/target joint topics plus the arm state topics and prints one
multi-line snapshot per second. It is deliberately excluded from every
launch file, so it only runs when started by hand:

```sh
ros2 run arm arm_sniffer_node
```

Output is spread over multiple lines with blank separators, for example:

```text
=================== ARM SNIFFER ===================

  enabled   : True
  e-stop    : False
  mode      : PICK_AND_PLACE (0)

  flags     : PLANAR
  locked    : none
  homing    : none

  joint            current [deg]   target [deg]   delta [deg]
  --------------   -------------   ------------   -----------
  base                   12.34          12.50         0.16
  shoulder              -45.00         -45.00         0.00
  elbow                  90.00          90.00         0.00
  wrist_bend              0.00           0.00         0.00
  wrist_twist             0.00           0.00         0.00
  gripper                 0.10           0.10         0.00

  error     : none

===================================================
```

## Topics

`*` below means one topic per joint: `BASE`, `SHOULDER`, `ELBOW`,
`WRIST_BEND`, `WRIST_TWIST`, `GRIPPER`.

### Ownership at a glance

| Node | Publishes | Subscribes |
| --- | --- | --- |
| `arm_teleop_node` | `/ARM/MANUAL/CMD_VELOCITY` | `/BASESTATION/spacemouse/*` |
| `arm_master_node` | `/ARM/CMD_VELOCITY`, `/ARM/PLANNER/COMMAND`, `/ARM/END_EFFECTOR/COMMAND`, `/ARM/STATE/CONTROL_SOURCE`, `/ARM/STATE/ERROR` | `/ESTOP`, `/ARM/COMMAND`, `/ARM/MANUAL/CMD_VELOCITY`, `/ARM/CONTROL/REQUEST`, `/ARM/PLANNER/STATUS` |
| `arm_end_effector_node` | `/ARM/JOINT/TARGET_GRIPPER`, `/ARM/JOINT/TARGET_SOLENOID` | `/ESTOP`, `/ARM/END_EFFECTOR/COMMAND`, `/ARM/JOINT/CURRENT_GRIPPER` |
| `arm_route_planner_node` | `/ARM/JOINT/TARGET_*` (5-DOF), planner `/ARM/STATE/*` topics, `/ARM/STATE/POSE`, `/ARM/PLANNER/STATUS` | `/ESTOP`, `/ARM/PLANNER/COMMAND`, `/ARM/CMD_VELOCITY`, `/ARM/JOINT/CURRENT_*` (5-DOF) |
| `arm_hardware_node` | `/ARM/JOINT/CURRENT_*`, CAN frames to the Teensy | `/ARM/JOINT/TARGET_*`, `/ARM/JOINT/TARGET_SOLENOID`, `/ARM/STATE/ENABLED`, `/ESTOP`, CAN frames from the Teensy |
| `arm_sniffer_node` | — | currents, targets, state flags, `/ARM/STATE/ENABLED`, `/ARM/STATE/CONTROL_SOURCE`, `/ESTOP` |
| `arm_camera_node` | — | — |

### `arm_teleop_node`

Inputs:

| Topic | Type | Notes |
| --- | --- | --- |
| `/BASESTATION/spacemouse/{x,y,z,rx,ry,rz}` | `std_msgs/Int16` | Raw HID axes (`BEST_EFFORT`). Republished every basestation cycle so the teleop node can detect a dead link. |

Outputs:

| Topic | Type | Notes |
| --- | --- | --- |
| `/ARM/MANUAL/CMD_VELOCITY` | `custom_interfaces/ArmVelocity` | Manual teleop intent. The master arbitrates it onto `/ARM/CMD_VELOCITY`. |

The node applies axis signs, deadzone with hysteresis, the power curve, a
vector-magnitude limit, and sensitivity once, then republishes the latched
command at a fixed rate (`TELEOP_PUBLISH_RATE_HZ`, default 50 Hz), including
all-zero commands. This keeps the master's input fresh while the stick is
idle. An axis whose raw value has not refreshed for
`TELEOP_INPUT_TIMEOUT_SEC` is
commanded as zero, so a dead link cannot leave the arm moving. It handles
teleop only; the master muxes this stream with the keyboard and any future
autonomous source.
`custom_interfaces/ArmPose` is defined for absolute target poses
(`/ARM/CMD_POSE`), but nothing consumes it yet.

### `arm_master_node`

Inputs:

| Topic | Type | Notes |
| --- | --- | --- |
| `/ESTOP` | `std_msgs/Bool` | While asserted, commands are ignored and the planner health check pauses. |
| `/ARM/COMMAND` | `std_msgs/String` | One high-level action per message. Manual command surface. |
| `/ARM/MANUAL/CMD_VELOCITY` | `custom_interfaces/ArmVelocity` | Manual teleop intent. Forwarded only while `MANUAL` owns the arm. |
| `/ARM/CONTROL/REQUEST` | `std_msgs/UInt8` | Request control ownership: `0` none, `1` manual, `2` autonomous. |
| `/ARM/PLANNER/STATUS` | `custom_interfaces/ArmPlannerStatus` | Planner readiness and the latest command/runtime rejection. |

`/ARM/COMMAND` accepts:
`flip`, `planar` / `toggle_planar`, `pnp_normal`, `toggle_flipped`,
`homing` / `home`, `joint_drive`, `position_drive`, `sync` / `sync_target`,
`toggle_position_lock`, `toggle_orientation_lock`, `guide_orientation`,
`toggle_joint_lock:<0-4>`, `home_joint:<0-4>`, `gripper_up`,
`gripper_down`, and `fire_solenoid` (`toggle_solenoid` is a legacy alias).
The motion commands are forwarded to the route planner for validation;
`gripper_up`, `gripper_down`, `fire_solenoid`, and `toggle_solenoid` go only
to the end effector. See the module docstring in `arm_master_node.py` for the
routing rules.

`/ARM/COMMAND` also carries keyboard actions. `basestation_node` decodes the
basestation's per-action press states (0 idle, 1 just pressed, 2 held, 3
just released). Edge-triggered actions publish on the just-pressed state; the
held gripper arrows publish on press and on every held state so the end
effector can jog until the stream stops. The key map and edge detection live
on the basestation.

Outputs:

| Topic | Type | Notes |
| --- | --- | --- |
| `/ARM/CMD_VELOCITY` | `custom_interfaces/ArmVelocity` | Arbitrated teleop velocity. Zeroed on every handover and E-stop; requires a neutral stick before manual motion resumes. |
| `/ARM/PLANNER/COMMAND` | `custom_interfaces/ArmCommand` | Motion commands, for planner validation. |
| `/ARM/END_EFFECTOR/COMMAND` | `custom_interfaces/ArmCommand` | Only the gripper/solenoid commands, gated on E-stop and readiness. Includes the internal `stop_jog` sent on a handover. |
| `/ARM/STATE/CONTROL_SOURCE` | `std_msgs/UInt8` | Authoritative control owner (`0` none, `1` manual, `2` autonomous). Diagnostic; published transient-local. |
| `/ARM/STATE/ERROR` | `std_msgs/UInt32` | Last rejection, packed as `(seq << 16) \| (code << 8) \| arg`; codes are in `rover_protocol.arm_errors`. |

The master runs its health cycle every `DT` (0.05 s, 20 Hz). It ignores
commands while E-stopped; solenoid strikes are forwarded even before the arm
initializes, while the gripper jog requires fresh planner readiness. The
planner reports command and runtime rejections in `/ARM/PLANNER/STATUS`, and
the master republishes them on `/ARM/STATE/ERROR`; identical runtime codes
are suppressed until the condition changes, while each rejected command
forces a new sequence number. Rejections of a held gripper arrow are
throttled to one report per second.

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
| `/ARM/CMD_VELOCITY` | `custom_interfaces/ArmVelocity` | Latest teleop velocity; converted to per-cycle deltas at `DT`. |
| `/ARM/JOINT/CURRENT_*` | `std_msgs/Float32` | Measured 5-DOF joint angles [rad]; the planner initializes once all five arrive. |

Outputs:

| Topic | Type | Notes |
| --- | --- | --- |
| `/ARM/JOINT/TARGET_*` | `std_msgs/Float32` | Rate-limited 5-DOF joint targets [rad]. Unchanged targets are skipped, with a forced refresh every 20 cycles. |
| `/ARM/STATE/HOMING` … `/ARM/STATE/ORIENTATION_LOCK` | `std_msgs/Bool` | Planner flags: `HOMING`, `FLIPPING`, `FLIPPED`, `PLANAR`, `JOINT_DRIVE`, `POSITION_LOCK`, `ORIENTATION_LOCK`. |
| `/ARM/STATE/LOCKED_JOINTS` | `std_msgs/UInt8` | Bitmask: bit `i` set means joint `i` is motor-locked. |
| `/ARM/STATE/HOMING_JOINTS` | `std_msgs/UInt8` | Bitmask: bit `i` set means joint `i` is being homed. |
| `/ARM/STATE/OPERATION_MODE` | `std_msgs/UInt8` | Wire code: `0` PNP, `1` PNP flipped, `2` planar, `3` homing. |
| `/ARM/STATE/POSE` | `custom_interfaces/ArmPose` | Measured end-effector pose (metres/radians). |
| `/ARM/PLANNER/STATUS` | `custom_interfaces/ArmPlannerStatus` | Readiness plus the latest command/runtime rejection code. |

The planner waits until all five measured angles arrive before it starts; it
then runs one control cycle every `DT` (0.05 s, 20 Hz). Runtime rejections
(e.g. jogging into a restricted octant) repeat every cycle, so identical
codes are suppressed until the condition changes; each rejected command
still forces a new sequence number. Path searches run inside the command
callback, so a long homing or flip plan can delay planner status briefly;
the master only declares the planner dead after `PLANNER_TIMEOUT_SEC`
(5 s).

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

`rover_simulation/gazebo_bridge` replaces `arm_hardware_node`: it subscribes
to the same `/ARM/JOINT/TARGET_*` and `/ARM/JOINT/TARGET_SOLENOID` topics and
publishes `/ARM/JOINT/CURRENT_*` from the Gazebo `arm_cont` position
controller. The master, planner, teleop, and basestation topics are
identical between simulation and hardware.

## Controller behaviour

- **Position drive (default):** stick translation/rotation moves the
  target frame; the frame is clamped by the active limiter, solved with
  damped-least-squares IK, then rate-limited before it becomes a joint
  target. This runs in the route planner.
- **Joint drive:** `rx → elbow`, `ry → shoulder`, `rz → base`; no IK. The
  planner applies the joint limits.
- **Limit modes:** Pick-and-Place normal and flipped, Planar (base lock),
  and Homing. Restricted octants (3, 5, 7) require Planar.
- **Actions:** `flip`, `planar`, `pnp_normal`, `homing`, `joint_drive`,
  `guide_orientation`, and joint locks run in the route planner.
  `gripper_up`, `gripper_down`, and `fire_solenoid` stay in the end effector
  node. The full command list and syntax are documented in the module
  docstrings of `arm_master_node.py` and `arm_route_planner_node.py`.
- **Slew model:** `SimulatedMechanicalArm` is used on hardware too, so a
  target never jumps by more than `MAX_ANG_VEL * DT`. The planner owns it
  because it is the sole writer of the five arm targets.
- **Watchdogs:** teleop motion stops after `VELOCITY_TIMEOUT_SEC` (0.5 s)
  applied to `/ARM/CMD_VELOCITY` in the planner, and the teleop node zeroes
  an axis whose raw input stopped refreshing for `TELEOP_INPUT_TIMEOUT_SEC`
  (0.5 s). A gripper jog stops after `GRIPPER_JOG_TIMEOUT_SEC` (0.25 s)
  without a new arrow command. The master treats the planner as
  unavailable after `PLANNER_TIMEOUT_SEC` (5 s) without status and reports
  `ARM_ERROR_REJECTED`. The planner runs path searches inside its command
  callback, so a long homing or flip plan can delay status; the 5 s watchdog
  covers that gap.

## Arm constants

`arm_constants.py` is the single source of truth for all arm constants:
joint order and ROS topic names, `ARM_MOTOR_IDX` and the per-motor
calibration table (`ARM_MOTOR_PARAMS`), mechanical limits, home pose, arm
geometry, CAN mappings, and controller tuning (IK, octants, path planning,
waypoint following, SpaceMouse axis map and sensitivity).  It also holds the
node-level defaults that must stay consistent across processes: teleop
publish rate, teleop/velocity/planner timeouts, the gripper-jog rejection
throttle, the manual re-arm neutral epsilon, and the debug sniffer cadence.

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
`ticks_per_rad`, `tick_offset`, tick limits, and `max_speed`.

`open_can_handler.py` is joint agnostic and handles only the three
OpenCAN motors (base, shoulder, elbow): it packs and decodes
`ENABLE` / `SET_SPEED` / `SEND` / `READ` frames, converts ticks using the
calibration entry it is given, and has no wrist or gripper knowledge.

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
- `/ESTOP` is level-triggered: it latches off, disables the OpenCAN
  motors, and ignores enable requests until it clears. `master_node`
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
