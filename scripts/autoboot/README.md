# autoboot

Run the rover stack automatically at boot through a systemd service.

`autoboot.sh` sources `scripts/source.sh` and runs:

```bash
ros2 launch main main_launch.xml
```

That is the real rover stack only: no Gazebo and no basestation.

## Install

```bash
./scripts/autoboot/install.sh
```

The installer asks for sudo, then:

- makes `autoboot.sh` executable,
- writes `/etc/systemd/system/rovers-autoboot.service` with **this**
  workspace's absolute path and the invoking user,
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
| `autoboot.service` | systemd unit template (`@USER@`, `@HOME@`, `@ROS_WS@`) |
| `install.sh` | Fills in the template and installs the unit |
| `uninstall.sh` | Stops, disables, and removes the unit |
