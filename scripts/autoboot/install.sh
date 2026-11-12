#!/usr/bin/env bash
# Install the autoboot systemd service for this workspace.
#
#   ./scripts/autoboot/install.sh
#
# Writes /etc/systemd/system/rovers-autoboot.service with this workspace's
# absolute path and the invoking user, then enables it at boot.

set -euo pipefail

_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_ws="$(cd "$_here/../.." && pwd)"
_service_src="$_here/autoboot.service"
_service_dst="/etc/systemd/system/rovers-autoboot.service"

if [ "$(id -u)" -ne 0 ]; then
    echo "Root is required to install a systemd unit; re-running with sudo."
    exec sudo -- "$_here/install.sh" "$@"
fi

# SUDO_USER is set when the script re-executes itself through sudo.
_run_user="${SUDO_USER:-$USER}"
_run_home="$(getent passwd "$_run_user" | cut -d: -f6)"
_run_home="${_run_home:-$HOME}"

chmod +x "$_here/autoboot.sh"

sed -e "s|@USER@|$_run_user|g" \
    -e "s|@HOME@|$_run_home|g" \
    -e "s|@ROS_WS@|$_ws|g" \
    "$_service_src" > "$_service_dst"
chmod 644 "$_service_dst"

systemctl daemon-reload
systemctl enable rovers-autoboot.service

cat <<EOF
Installed $_service_dst

  user:      $_run_user
  workspace: $_ws

Start it now:      sudo systemctl start rovers-autoboot.service
Status:            systemctl status rovers-autoboot.service
Logs:              journalctl -u rovers-autoboot.service -f
Disable at boot:   sudo systemctl disable rovers-autoboot.service
EOF
