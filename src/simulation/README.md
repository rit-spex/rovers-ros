# Simulation

All Gazebo-facing assets live under this directory.

```text
src/simulation/
  rover_simulation/
    description/   robot xacro, Gazebo plugins, ros2_control interfaces
    worlds/        empty.world, obstacles.world
    config/        Gazebo params and controller config
    rviz/          sim.rviz (RViz view for the simulated rover)
    launch/        launch_sim.launch.py, rsp.launch.py,
                   simulation_launch.xml (simulated rover node stack)
    rover_simulation/
      arm_sim_node.py        simulated arm hardware (replaces arm_hardware)
      drivebase_sim_node.py  simulated drive hardware (replaces the Teensy)
      sim_config_node.py     publishes the selected build to telemetry
  gazebo_sim_control/
    Gazebo GUI plugin: robot configuration and restart panel
```

## Why the simulation-only nodes live here

`arm_sim_node.py`, `drivebase_sim_node.py`, and `sim_config_node.py` only
exist for simulation. They are not used by `main_launch.xml` on the real
rover, so they are installed by `rover_simulation` rather than by `arm`,
`chassis`, or `main`.

They are not a second command path. They are the simulated hardware
backends: they implement the same interfaces as `arm_hardware_node` and the
chassis Teensy, and only the final link to Gazebo is different.

## Topic namespaces

Topics that exist only for Gazebo are namespaced under `/GAZEBO` so they
cannot be confused with the real rover's interfaces:

```text
/GAZEBO/joint_states                           shared joint state bus
/GAZEBO/ARM/arm_cont/commands                  arm forward command controller
/GAZEBO/CHASSIS/diff_cont/cmd_vel_unstamped    drive controller velocity command
/GAZEBO/CHASSIS/diff_cont/cmd_vel_joy          simulated joystick input
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
controllers, and `drivebase_sim_node`. The drivebase node handles `/ESTOP`,
the CAN `Drive_Power` path, navigation input, joystick input, and the motor
deadzone. `simulation_launch.xml`
starts the simulated rover nodes (`xbee_udp`, `can_udp`, chassis,
`arm_sim_node`, IK, etc.).

`drivebase_sim_node` applies a small per-wheel motor deadzone
(`motor_deadzone`, default `0.1` m/s) so tiny drive commands do not creep the
simulated rover. Override it with `-p motor_deadzone:=<m/s>`;
`wheel_separation` (default `0.58` m) must match
`config/my_controllers.yaml`.

## Drive path parity

The real rover has one way to reach its motors: `chassis_node` turns
basestation joystick axes into a `Drive_Power` CAN frame, and the chassis
Teensy turns that into motor power. The simulation keeps that path rather
than inventing a second one:

```text
chassis_node -> CAN/TX/CHASSIS/Drive_Power
             -> drivebase_sim_node (stands in for the Teensy)
             -> /GAZEBO/CHASSIS/diff_cont/cmd_vel_unstamped
             -> diff_cont -> 4 wheels
```

`drivebase_sim_node` decodes the frame with the same `constants` helpers the
CAN transports use, scales per-side power in `[-1, 1]` by
`max_wheel_angular_velocity` (default `10` rad/s, matching the joint command
limit) times the wheel radius, and writes the result directly to the
diff-drive controller. There is no separate twist mux or relay stage.

It is only authoritative while it is commanding motion: a neutral stream
goes passive, and a released command publishes zero for `0.3` s before
letting another source take over. The same node accepts `/cmd_vel`
navigation (priority `10`) and the direct `joystick.launch.py` topic
(priority `100`), while basestation input (priority `200`) wins when the
sticks move. `/ESTOP` stays highest.

Because the CAN input is latched, extended silence on `Drive_Power`
(`1.0` s) is treated as a lost command path: the node publishes a final stop
and goes passive. Link loss itself is handled upstream, where `master_node`
latches `/ESTOP` after the basestation heartbeat stops.

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
heartbeat. It tolerates up to 1 second without one before latching again,
so a single delayed UDP/radio packet does not stop the rover. `sim.sh start`
runs the basestation, so it clears normally; `sim.sh stack` alone stays
stopped until a heartbeat is published.

## Keyboard

Arm keys are mapped to actions on the basestation and arrive over the radio
as per-action press states (0 idle, 1 just pressed, 2 held, 3 just released).
`basestation_node` converts the just-pressed states into `/ARM/COMMAND`
messages, so `arm_controller_node` never holds the key map or does any edge
detection. The simulator does not capture keys or drive arm joints directly;
arm motion comes from the normal `/ARM/JOINT/*` interfaces (IK, SpaceMouse,
or keyboard).

