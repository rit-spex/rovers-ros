# autoboot

Run the rover stack automatically at boot through a systemd service.

`autoboot.sh` sources `scripts/source.sh` and runs:

```bash
ros2 launch main main_launch.xml
```

That is the real rover stack only: no Gazebo and no basestation.

## Maker Faire mode

Pass `--maker-faire` to also start `arm_simulated_teensy_node` alongside the
normal rover stack:

```bash
./scripts/autoboot/install.sh --maker-faire
```

This uses the `main` package's `maker_faire_launch.xml`
(`ros2 launch main maker_faire_launch.xml`), an autoboot-only profile
that includes `main_launch.xml` and adds the simulated wrist/gripper Teensy.
`main_launch.xml` itself is not changed, so manual launches are unaffected.
Re-run `install.sh` without the flag to go back to the normal profile.

`autoboot.sh` accepts the same flag for direct foreground runs:

```bash
./scripts/autoboot/autoboot.sh --maker-faire
```

## Install

```bash
./scripts/autoboot/install.sh [--maker-faire]
```

The installer asks for sudo, then:

- makes `autoboot.sh` executable,
- writes `/etc/systemd/system/rovers-autoboot.service` with **this**
  workspace's absolute path, the invoking user, and the selected boot profile,
- reloads systemd and enables the unit at boot.

Nothing is hardcoded to `/home/rovers`; the same script works on the rover,
a dev laptop, or a differently named checkout.

## Manage

```bash
sudo systemctl start rovers-autoboot.service     # start now
systemctl status rovers-autoboot.service         # state
journalctl -u rovers-autoboot.service -f         # follow logs
sudo systemctl stop rovers-autoboot.service      # stop the stack
sudo systemctl disable rovers-autoboot.service   # do not start at boot
```

Re-run `install.sh` after moving the workspace or changing users.

## Disable

| Goal | Command |
|---|---|
| Stop now, keep installed | `sudo systemctl stop rovers-autoboot.service` |
| Stop starting at boot, keep installed | `sudo systemctl disable rovers-autoboot.service` |
| Remove the unit completely | `./scripts/autoboot/uninstall.sh` |

`uninstall.sh` stops and disables the service, deletes
`/etc/systemd/system/rovers-autoboot.service`, and reloads systemd. It uses
sudo automatically. Re-run `install.sh` to turn autoboot back on.

## Files

| File | Purpose |
|---|---|
| `autoboot.sh` | Sources the workspace and launches the rover stack |
| `autoboot.service` | systemd unit template (`@USER@`, `@HOME@`, `@ROS_WS@`, `@AUTOBOOT_ARGS@`) |
| `install.sh` | Fills in the template and installs the unit |
| `uninstall.sh` | Stops, disables, and removes the unit |

The Maker Faire profile now ships with the `main` package
(`src/main/launch/maker_faire_launch.xml`) and is launched as
`ros2 launch main maker_faire_launch.xml`.
