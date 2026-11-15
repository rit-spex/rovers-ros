# Arm

ROS 2 package for the SPEX rover's 5-DOF arm plus gripper and solenoid. It
contains the operator-facing arm controller and the CAN hardware bridge, plus
a (placeholder) camera node and an opt-in debug sniffer.

The arm model is a five-joint chain — base yaw, shoulder pitch, elbow
pitch, wrist bend, wrist twist — with a separate gripper and solenoid. The
kinematics, limiters, and drive modes come from the `armSimulator4`
controller and live in `utils/`.

## Package layout

```
src/subsystems/arm/
├── launch/arm_launch.xml          # standalone launch for the main arm nodes
├── resource/arm                   # ament index marker
├── setup.py / setup.cfg           # entry points, src-layout packaging
├── src/
│   ├── constants.py               # all arm constants: topics, motors, limits,
│   │                              # CAN mappings, and controller tuning
│   ├── arm_camera/                # placeholder node
│   │   └── arm_camera_node.py
│   ├── arm_teleop/                # teleop input normalization
│   │   └── arm_teleop_node.py
│   ├── arm_controller/
│   │   ├── arm_controller_node.py # ROS-facing controller node
│   │   └── utils/
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

The package provides four runtime nodes plus an opt-in debug tool.
`arm_launch.xml` starts the first four and renames them to `arm_camera`,
`arm_teleop`, `arm_controller`, and `arm_hardware`.

| Executable | Standalone node name | Launched by default | Responsibility |
| --- | --- | --- | --- |
| `arm_teleop_node` | `arm_teleop_node` | yes | Owns SpaceMouse semantics; publishes normalized velocity and gripper commands. |
| `arm_controller_node` | `arm_controller_node` | yes | Owns arm state: modes, locks, homing/flip actions, IK, and rate-limited joint targets. |
| `arm_hardware_node` | `arm_hardware_node` | yes | Owns the CAN link: sends targets, polls measured angles, enforces enable/E-stop. Replaced by `gazebo_bridge` in simulation. |
| `arm_camera_node` | `arm_camera_node` | yes | Placeholder; no logic yet. |
| `arm_sniffer_node` | `arm_sniffer_node` | no | Read-only diagnostic printer. |

### `arm_teleop_node`

The only node that knows the SpaceMouse exists. It subscribes to the six raw
`/BASESTATION/spacemouse/*` axes, applies the sign convention, deadzone, power
curve, and sensitivity from `constants.py`, then republishes the result as a
velocity command at `publish_rate_hz` (default 50 Hz). It republishes
continuously because the basestation publishes raw axes only on change.

- Axis processing happens exactly once, here; the controller consumes
  normalized values.
- Translation is converted from inches-per-cycle to m/s at the ROS boundary.
- The processed `ry` axis is also published as the gripper axis.
- It emits teleop intent only. Keyboard actions bypass it and arrive on
  `/ARM/COMMAND` from `basestation_node`; autonomy or pose sources should feed
  the same command topics through a separate mux node.

### `arm_controller_node`

The arm state machine and the main consumer of teleop input. It starts
uninitialized and stays that way until all five measured joint angles have
arrived, then builds `RoverArmController` from the measured pose and adopts the
matching normal/flipped side. It then runs one cycle every `DT` (0.05 s,
20 Hz):

1. Sync the controller to the measured angles.
2. Convert the latest velocity command to per-cycle deltas; ignore it once it
   is older than `VELOCITY_TIMEOUT_SEC` (0.5 s).
3. Advance the mode state machine via `RoverArmController.update()`.
4. Publish rate-limited joint targets and the arm state flags.

It owns every operator-facing behavior: position vs joint drive, normal /
flipped / planar limit modes, homing, side flips, single-joint homing, joint
locks, position/orientation locks, orientation guide, and gripper drive. The
gripper stays outside the 5-DOF model. Commands are refused while E-stopped and
rejections are packed into `/ARM/STATE/ERROR`. Unchanged targets are not
republished every cycle, but are refreshed every 20 cycles so late consumers
still receive them; a locked joint republishes its last commanded target
instead of echoing encoder noise.

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
the controller can initialize. A solenoid strike sends one
`MOVE_SOLENOID.Enabled=1` frame; the Teensy retracts it, so there is no disable
command. `destroy_node()` makes a best-effort attempt to disable the motors.

### `arm_camera_node`

Placeholder only: it creates a node and spins, with no publishers or
subscribers. It is included in `arm_launch.xml` so the launch composition does
not change when the camera is implemented.

### `arm_sniffer_node`

Read-only diagnostic tool, deliberately excluded from every launch file. It
subscribes to the current/target joint topics plus the controller state and
E-stop topics and prints one snapshot per second. See
[Debugging](#debugging).

### Composition roots

`arm_launch.xml` is the standalone entry point. On the real rover the
composition root is `src/main/launch/main_launch.xml`, which starts the teleop,
hardware, and controller nodes. In simulation,
`src/simulation/rover_simulation/launch/simulation_launch.xml` starts the
Gazebo bridge, teleop node, and controller; the bridge replaces
`arm_hardware_node` and speaks the same target/current topics.

## Data flow

```
 spacemouse / keyboard / /ARM/COMMAND
                  │
                  ▼
          arm_teleop_node ──/ARM/CMD_VELOCITY──► arm_controller_node
                  │                         │  /ARM/JOINT/TARGET_*
                  └──/ARM/CMD_GRIPPER_AXIS──┘        │
                                                     ▼
                                        arm_hardware_node ──CAN──► Teensy
                                                     ▲             motors
                                                     └─/ARM/JOINT/CURRENT_*
```

In simulation `arm_hardware_node` is swapped for
`rover_simulation/gazebo_bridge`, which maps the same target/current topics
onto the Gazebo `arm_cont` position controller.

## Debugging

`arm_sniffer_node` is a read-only diagnostic printer. It subscribes to the
current/target joint topics plus the controller state topics and prints one
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
| `arm_teleop_node` | `/ARM/CMD_VELOCITY`, `/ARM/CMD_GRIPPER_AXIS` | `/BASESTATION/spacemouse/*` |
| `arm_controller_node` | `/ARM/JOINT/TARGET_*`, `/ARM/JOINT/TARGET_SOLENOID`, `/ARM/STATE/*` | `/ESTOP`, `/ARM/CMD_VELOCITY`, `/ARM/CMD_GRIPPER_AXIS`, `/ARM/COMMAND`, `/ARM/JOINT/CURRENT_*` |
| `arm_hardware_node` | `/ARM/JOINT/CURRENT_*`, CAN frames to the Teensy | `/ARM/JOINT/TARGET_*`, `/ARM/JOINT/TARGET_SOLENOID`, `/ARM/STATE/ENABLED`, `/ESTOP`, CAN frames from the Teensy |
| `arm_sniffer_node` | — | currents, targets, state flags, `/ARM/STATE/ENABLED`, `/ESTOP` |
| `arm_camera_node` | — | — |

### `arm_teleop_node`

Inputs:

| Topic | Type | Notes |
| --- | --- | --- |
| `/BASESTATION/spacemouse/{x,y,z,rx,ry,rz}` | `std_msgs/Int16` | Raw HID axes (`BEST_EFFORT`). The basestation publishes only on change. |

Outputs:

| Topic | Type | Notes |
| --- | --- | --- |
| `/ARM/CMD_VELOCITY` | `custom_interfaces/ArmVelocity` | Linear velocity [m/s] and angular velocity [rad/s] in the arm base frame. |
| `/ARM/CMD_GRIPPER_AXIS` | `std_msgs/Float32` | Processed `ry` axis in [-1, 1], used only in gripper-drive mode. |

The node applies axis signs, deadzone, the power curve, and sensitivity once,
then republishes the latched command at a fixed rate (`publish_rate_hz`,
default 50 Hz), including all-zero commands. This keeps the controller's
watchdog alive while the stick is idle. It handles teleop only; autonomy,
pose, and joint-target sources should feed the same command topics through a
separate input/mux node. `custom_interfaces/ArmPose` is defined for absolute
target poses (`/ARM/CMD_POSE`), but nothing consumes it yet.

### `arm_controller_node`

Inputs:

| Topic | Type | Notes |
| --- | --- | --- |
| `/ESTOP` | `std_msgs/Bool` | While asserted, the cycle pauses and all commands except clearing E-stop are refused. |
| `/ARM/CMD_VELOCITY` | `custom_interfaces/ArmVelocity` | Normalized arm velocity from `arm_teleop_node`. |
| `/ARM/CMD_GRIPPER_AXIS` | `std_msgs/Float32` | Normalized gripper axis, applied only in gripper-drive mode. |
| `/ARM/COMMAND` | `std_msgs/String` | One high-level action per message. |
| `/ARM/JOINT/CURRENT_*` | `std_msgs/Float32` | Measured joint angles [rad]; the controller initializes once all five arrive. |

`/ARM/COMMAND` accepts:
`flip`, `planar` / `toggle_planar`, `pnp_normal`, `toggle_flipped`,
`homing` / `home`, `joint_drive`, `position_drive`, `sync` / `sync_target`,
`toggle_position_lock`, `toggle_orientation_lock`, `guide_orientation`,
`toggle_joint_lock:<0-4>`, `home_joint:<0-4>`, `gripper_drive`, and
`fire_solenoid` (`toggle_solenoid` is a legacy alias). See the module
docstring in `arm_controller_node.py` for the full semantics.

`/ARM/COMMAND` also carries keyboard actions. `basestation_node` decodes the
basestation's per-action press states (0 idle, 1 just pressed, 2 held, 3
just released) and publishes only the just-pressed actions, so the key map
and edge detection live on the basestation, not in the controller.

If `/ARM/CMD_VELOCITY` goes quiet for more than `VELOCITY_TIMEOUT_SEC` (0.5 s), the
controller stops applying the last command instead of continuing to move.

Outputs:

| Topic | Type | Notes |
| --- | --- | --- |
| `/ARM/JOINT/TARGET_*` | `std_msgs/Float32` | Rate-limited joint targets [rad]. Unchanged targets are skipped, with a forced refresh every 20 cycles. |
| `/ARM/JOINT/TARGET_SOLENOID` | `std_msgs/Bool` | Momentary solenoid strike; one `true` pulse per command. The Teensy fires and retracts the solenoid itself, so the rover never sends a disable. |
| `/ARM/STATE/HOMING` … `/ARM/STATE/GRIPPER_DRIVE` | `std_msgs/Bool` | One topic per high-level flag: `HOMING`, `FLIPPING`, `FLIPPED`, `PLANAR`, `JOINT_DRIVE`, `POSITION_LOCK`, `ORIENTATION_LOCK`, `GRIPPER_DRIVE`. |
| `/ARM/STATE/LOCKED_JOINTS` | `std_msgs/UInt8` | Bitmask: bit `i` set means joint `i` is motor-locked. |
| `/ARM/STATE/HOMING_JOINTS` | `std_msgs/UInt8` | Bitmask: bit `i` set means joint `i` is being homed. |
| `/ARM/STATE/OPERATION_MODE` | `std_msgs/UInt8` | Wire code: `0` PNP, `1` PNP flipped, `2` planar, `3` homing. |
| `/ARM/STATE/ERROR` | `std_msgs/UInt32` | Last rejection, packed as `(seq << 16) \| (code << 8) \| arg`; codes are in `rover_protocol.arm_errors`. |

The node waits until all five measured angles arrive before it starts; it
then runs one control cycle every `DT` (0.05 s, 20 Hz). Runtime rejections
(e.g. jogging into a restricted octant) repeat every cycle, so identical
codes are suppressed until the condition changes; each rejected command
still forces a new sequence number.

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
controller. The controller, teleop, and basestation topics are identical
between simulation and hardware.

## Controller behaviour

- **Position drive (default):** stick translation/rotation moves the
  target frame; the frame is clamped by the active limiter, solved with
  damped-least-squares IK, then rate-limited before it becomes a joint
  target.
- **Joint drive:** `rx → elbow`, `ry → shoulder`, `rz → base`; no IK.
- **Limit modes:** Pick-and-Place normal and flipped, Planar (base lock),
  and Homing. Restricted octants (3, 5, 7) require Planar.
- **Actions:** `flip`, `planar`, `pnp_normal`, `homing`, `joint_drive`,
  `guide_orientation`, joint locks, gripper drive, and `fire_solenoid` are
  all available on `/ARM/COMMAND`. The full list and command syntax are
  documented in the module docstring of `arm_controller_node.py`.
- **Slew model:** `SimulatedMechanicalArm` is used on hardware too, so a
  target never jumps by more than `MAX_ANG_VEL * DT`.

## Shared constants

`src/constants.py` is the single source of truth for all arm constants:
joint order and ROS topic names, `ARM_MOTOR_IDX` and the per-motor
calibration table (`ARM_MOTOR_PARAMS`), mechanical limits, home pose, arm
geometry, CAN mappings, and controller tuning (IK, octants, path planning,
SpaceMouse sensitivity).

The arm source root is ahead of the workspace `constants` package on
`sys.path`, so the top-level `constants.py` shadows that package.
`src/constants.py` adds the workspace package directory to its `__path__`
so `constants.CAN_*` and `constants.CommandCodes` still resolve to the
shared package. The arm nodes use `constants.CommandCodes` as the existing
bridge for the workspace `lib/rovers-protocol` submodule.

## Hardware layer

`src/constants.py` holds `ARM_MOTOR_IDX` (base, shoulder, elbow, bend
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
  controller can initialize and track the measured pose.
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
  explicit `package_dir` mapping, and `constants` installed as a top-level
  module. The package-root `constants.py` is a symlink into `src/` so
  `--symlink-install` links it into the build space.
- On the rover, prefer `ros2 launch main main_launch.xml` so the master
  node, CAN comms, and telemetry are included.
- For simulation, use the `rover_simulation` launch files with
  `robot_config:=arm_drivebase`.

## Tests

The kinematic/limiter tests are standalone scripts:

```bash
source install/setup.bash
cd src/subsystems/arm
PYTHONPATH=src:$PYTHONPATH python3 test/test_limiter_modes.py
PYTHONPATH=src:$PYTHONPATH python3 test/test_pick_and_place_configs.py
```

Lint checks are the standard ament ones and can be run with
`colcon test --packages-select arm`.
