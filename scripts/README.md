# scripts

Helper scripts for building, sourcing, and running the rover workspace.

| Script | Purpose |
|---|---|
| `source.sh` | Source ROS 2, the workspace overlay, and `.venv` if present |
| `common.sh` | Shared paths, build helpers, and process cleanup (source only) |
| `sim.sh` | Start/stop the simulation, run the rover stack headless, or open the RViz view |
| `restart_gazebo.sh` | Rebuild and restart Gazebo + rover stack (used by the GUI panel) |

## Usage

```bash
# Shell environment
source scripts/source.sh

# Full simulation: Gazebo + rover stack + basestation
./scripts/sim.sh start [drivebase|arm_drivebase|life_drivebase]

# Rover stack only, no Gazebo or GUI
./scripts/sim.sh stack [config]

# RViz view of the simulated rover
./scripts/sim.sh view

# Restart / stop / inspect
./scripts/sim.sh restart [config]
./scripts/sim.sh stop
./scripts/sim.sh status
```

`sim.sh start` requires `gnome-terminal` and a graphical session. Use
`sim.sh stack` on a headless machine.

`restart_gazebo.sh` is what the in-Gazebo Simulation Control panel calls. It
rebuilds the sim packages, stops the Gazebo GUI and rover stack, then starts
them again. The external basestation process is left running.

## Environment variables

| Variable | Default | Meaning |
|---|---|---|
| `ROS_SETUP` | `/opt/ros/humble/setup.bash` | ROS 2 underlay to source |
| `ROS_WS` | directory containing `scripts/` | Rover workspace root |
| `BASE_WS` | `$ROS_WS/../rovers-basestation` | Basestation workspace |
| `WORLD` | `empty.world` | World file in `rover_simulation/worlds/` |
| `ROBOT_CONFIG` | saved config, else `arm_drivebase` | Build selection |
| `SIM_STATE_DIR` | `${XDG_RUNTIME_DIR:-/tmp}/rovers-sim` | Tab state and logs |

## Gazebo ros2_control plugin

`gazebo_ros2_control` supplies `libgazebo_ros2_control.so`, which loads the
controller manager inside Gazebo. It is a system dependency (declared in
`rover_simulation/package.xml`), so install it system-wide:

```bash
sudo apt install ros-humble-gazebo-ros2-control
```

`setup_gazebo()` in `common.sh` exports the workspace GUI plugin path and
stops with install instructions if `gazebo_ros2_control` is missing.

## Logs

`restart_gazebo.sh` writes to `$SIM_STATE_DIR/logs/`:

```text
build.log    colcon output
gazebo.log   Gazebo launch output
stack.log    rover stack launch output
```
