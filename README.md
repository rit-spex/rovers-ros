# Rovers ROS

ROS 2 workspace for the RIT SPEX rover (Scorpio). Runs on the Nvidia Jetson
Orin Nano.

## Workspace layout

```text
src/
  communications/      transports and protocol: real xbee/can_comms links,
                       simulated xbee_udp/can_udp links, basestation nodes
  constants/           shared CAN/protocol constants (cross-cutting leaf)
  custom_interfaces/   shared ROS message definitions (cross-cutting leaf)
  inverse_kinematics/  arm inverse kinematics
  main/                real-rover composition root
  simulation/          Gazebo assets, bridges, GUI plugin, and RViz view
  subsystems/          hardware and capability subsystems:
    arm/               arm control and drivers
    autonomous/        navigation, SLAM, pathfinding
                       (also hosts the vendored joy_tester and
                       pointcloud_to_laserscan tools)
    sensors/           camera and GPS drivers
    chassis/           drivebase node
    status_led/        status LED heartbeat
```

The tree deliberately mixes four organizing axes because no single one fits
every package:

- **By capability** — `subsystems/autonomous/`, `subsystems/sensors/`,
  `communications/`.
- **By hardware subsystem** — `subsystems/arm/`, `subsystems/chassis/`,
  `subsystems/status_led/`.
- **By deployment target** — `main/` (real rover) versus `simulation/`
  (Gazebo).
- **By provenance** — vendored packages (`joy_tester`,
  `pointcloud_to_laserscan`) and the `lib/rovers-protocol` submodule.

`simulation/` means *Gazebo-specific*, not *simulation-only*. For example,
`can_udp` and `xbee_udp` are simulation-only, but they substitute a
transport rather than emulating Gazebo, so they stay under
`communications/`. `main/launch/main_launch.xml` and
`simulation/rover_simulation/launch/simulation_launch.xml` are parallel
composition roots: they select real versus simulated transports while
sharing the same subsystem nodes.

See `src/simulation/README.md` for the Gazebo stack, its topic namespaces,
and its launch entry points. For a primer on the ROS 2 runtime graph, node
lifecycle, QoS, and adding a new node, see
[`ROS2_EXPLAINED.md`](ROS2_EXPLAINED.md).

## Shared Protocol Submodule

This repository depends on the shared protocol package (`rovers-protocol`)
for encoding/decoding messages from the basestation. The same protocol repo
is used by both this repo and `rovers-basestation`.

```bash
# Clone with submodule
git clone --recurse-submodules https://github.com/rit-spex/rovers-ros.git

# Or if already cloned
git submodule update --init --recursive
```

Expected submodule location: `lib/rovers-protocol/`

### How it works

The basestation sends controller data over XBee radio as compact bit-packed
bytes. The `basestation_node` decodes them using `MessageEncoder` from the
shared protocol and publishes the values as ROS topics. The
`telemetry_uplink_node` collects rover telemetry from ROS topics, encodes
them using the same protocol, and sends them back to the basestation over
UDP.

```
                        XBee Radio
  ┌───────────────┐   ───────────────> ┌─────────────────┐
  │ BASESTATION   │   controller data  │  ROVER (ROS 2)  │
  │ (Raspberry Pi)│  <───────────────  │  (this repo)    │
  │               │   telemetry (UDP)  │                 │
  └───────────────┘                    └─────────────────┘
```

### ROS nodes that use the protocol

| Node | File | Purpose |
|------|------|---------|
| `basestation_node` | `src/communications/basestation/basestation/basestation_node.py` | Decodes XBee messages → ROS topics |
| `telemetry_uplink_node` | `src/communications/basestation/basestation/telemetry_uplink_node.py` | Encodes rover telemetry and sends it to the basestation via UDP |

### Protocol trace debugging

Set `protocol_trace:=1` to print every protocol message at hex level. This
verifies that the rover encodes and decodes the same bit-packed IDs and
field values the basestation expects.

```bash
# In simulation
ros2 launch rover_simulation simulation_launch.xml protocol_trace:=1

# On hardware (SSH in and run)
ros2 launch main main_launch.xml protocol_trace:=1
```

`basestation_node` and `telemetry_uplink_node` then log one line per
message, for example:

```text
[protocol rx] id=0x02 name=xbox payload={'AXIS_LY': 100, 'AXIS_RY': 100, ...} bytes=02 00 ff ...
[protocol tx] id=0xF1 name=arm_encoders payload={'arm_base_position': 0, ...} bytes=f1 04 5a ...
```

`rx` lines are basestation messages decoded on the rover; `tx` lines are
telemetry frames encoded for the basestation.

`simulation_launch.xml` forwards the argument to `ROVER_PROTOCOL_TRACE`;
`scripts/sim.sh` does not pass it, so re-launch the rover stack with
`protocol_trace:=1` when you need a trace.

## Building

ROS 2 Humble is required. From the workspace root:

```bash
colcon build --symlink-install
source scripts/source.sh
```

Or do both with:

```bash
source scripts/source.sh --build
```

`--symlink-install` is what the helper scripts use; it keeps Python package
files live-linked so source edits do not require a rebuild. `scripts/source.sh`
reports the overlay and the last build time.

## Running

### Real rover

```bash
ros2 launch main main_launch.xml
```

Or source and launch in one step:

```bash
source scripts/source.sh --run
```

This starts the real transports (`can_comms`, `xbee`), `basestation_node`,
`telemetry_uplink_node`, `chassis`, `arm`, `inverse_kinematics`,
`master_node`, and `status_led`. It expects the CAN interface and the XBee
radio to be attached; `xbee_node` logs a warning and keeps running if the
radio is missing, but `can_comms` cannot open `can0` without it.

### Simulation

```bash
# Full simulation: Gazebo + simulated rover stack + basestation
./scripts/sim.sh start [drivebase|arm_drivebase|life_drivebase]

# Simulated rover stack only, no Gazebo or GUI (headless machines)
./scripts/sim.sh stack [config]

# RViz view of the simulated rover
./scripts/sim.sh view

# Restart / stop / inspect
./scripts/sim.sh restart [config]
./scripts/sim.sh stop
./scripts/sim.sh status
```

`sim.sh start` needs `gnome-terminal` and a graphical session; it builds the
workspace, then opens one tab each for Gazebo, the rover stack, and the
basestation (the last one only when `BASE_WS` exists). `reset` is accepted as
an alias for `restart`. The Gazebo window contains the robot-configuration
selector and the simulation restart button.

The configuration argument selects the simulated build and is remembered in
`~/.config/rovers-ros/sim_config`; the default is `arm_drivebase`. See
`src/simulation/README.md` for the simulated drive and arm bridges.

To launch the **real** rover stack automatically at boot, install the systemd
unit (it runs `main_launch.xml`, not the simulation):

```bash
./scripts/autoboot/install.sh     # enable at boot
./scripts/autoboot/uninstall.sh   # disable and remove
```

See `scripts/README.md` for the helper scripts, environment variables, and
log locations.

## Connecting to the rover

```bash
ssh rovers@129.21.91.140
# Password: rovers
```

Then:

```bash
cd ~/ros/rovers-ros
source scripts/source.sh
ros2 launch main main_launch.xml
```

`can_comms` resets the `can0` interface when it starts. Install the narrow
passwordless sudo rule once so neither manual launches nor autoboot prompt for
a password:

```bash
./scripts/can/install.sh
```

It only allows `ip link set can0 ...`, not general root access.

If autoboot is installed, the real rover stack is already running after boot;
launching it again starts duplicate nodes.
