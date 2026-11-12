# Simulation

All Gazebo-facing assets live under this directory.

```text
src/simulation/
  rover_simulation/
    description/   robot xacro, Gazebo plugins, ros2_control interfaces
    worlds/        empty.world, obstacles.world
    config/        Gazebo params, controller config, twist mux
    rviz/          sim.rviz (RViz view for the simulated rover)
    launch/        launch_sim.launch.py, rsp.launch.py,
                   simulation_launch.xml (simulated rover node stack)
    rover_simulation/
      gazebo_bridge.py     arm topics <-> Gazebo joint commands
      sim_config_node.py   publishes the selected build to telemetry
  gazebo_sim_control/
    Gazebo GUI plugin: robot configuration and restart panel
```

## Why the bridge lives here

`gazebo_bridge.py` and `sim_config_node.py` only exist for simulation.
They are not used by `main_launch.xml` on the real rover, so they are
installed by `rover_simulation` rather than by `arm` or `main`.

## Topic namespaces

Topics that exist only for Gazebo are namespaced under `/GAZEBO` so they
cannot be confused with the real rover's interfaces:

```text
/GAZEBO/joint_states                           shared joint state bus
/GAZEBO/ARM/arm_cont/commands                  arm forward command controller
/GAZEBO/CHASSIS/diff_cont/cmd_vel_unstamped    drive controller velocity command
/GAZEBO/CHASSIS/diff_cont/cmd_vel_joy          twist_mux joystick input
/GAZEBO/CHASSIS/diff_cont/odom                 drive odometry
```

Arm topics live under `/GAZEBO/ARM/` and drivebase topics under
`/GAZEBO/CHASSIS/`. `/GAZEBO/joint_states` is shared because it carries
both arm and wheel joints.

The controller topics come from the ros2_control controller manager, which
runs in the `GAZEBO` namespace. Topics that emulate the real rover (the
`/ARM/JOINT/*`, `/ARM/STATE/*`, `/ROVER/*`, `/ESTOP`,
`/BASESTATION/*` interfaces) deliberately keep their hardware names.

### URDF comment gotcha

`gazebo_ros2_control` passes the robot description through
`--param robot_description:=...`, and rcl parses that value as YAML. Avoid
`: ` (colon followed by a space) in xacro comments that end up in the
URDF, otherwise the controller manager fails to start and no controllers
(including the arm) load.

## Running

Use the workspace entry point:

```bash
./scripts/sim.sh start
./scripts/sim.sh reset
./scripts/sim.sh stop
./scripts/sim.sh view      # RViz view, once the stack is running
```

`launch_sim.launch.py` starts Gazebo, the robot state publisher, the
controllers, and `sim_estop_gate`. The gate holds the simulated diff drive at
zero through `twist_mux` while `/ESTOP` is active. `simulation_launch.xml`
starts the simulated rover nodes (`xbee_udp`, `can_udp`, chassis, arm bridge,
IK, etc.).

## Robot configuration

The simulated build is selected with the `robot_config` launch argument:

```text
drivebase        drivebase only
arm_drivebase    drivebase + arm
life_drivebase   drivebase + life telemetry (no life hardware model)
```

`life_drivebase` currently enables the life-detection telemetry flag; it does
not add life hardware to the Gazebo model.

Set it with the script:

```bash
./scripts/sim.sh start arm_drivebase
./scripts/sim.sh stack life_drivebase
```

or directly:

```bash
ros2 launch rover_simulation launch_sim.launch.py robot_config:=arm_drivebase
```

The default is saved in `~/.config/rovers-ros/sim_config`. The Gazebo
**Simulation Control** panel can change it and restart the simulation.

`master_node` starts with E-STOP latched until it receives a basestation
heartbeat. `sim.sh start` runs the basestation, so it clears normally;
`sim.sh stack` alone stays stopped until a heartbeat is published.

## Keyboard

Arm keys arrive from the basestation on
`/BASESTATION/keyboard/arm_keys` and are consumed directly by
`inverse_kinematics`, which converts the held-key bitmask into arm
actions. The simulator does not capture keys or drive arm joints
directly; arm motion comes from the normal `/ARM/JOINT/*` interfaces (IK,
SpaceMouse, or keyboard).

