#!/bin/bash
# Remove the autoboot systemd service installed by install.sh.
#
#   ./scripts/autoboot/uninstall.sh
#
# Stops the service if it is running, disables it at boot, and deletes
# /etc/systemd/system/rovers-autoboot.service.

set -euo pipefail

_service="rovers-autoboot.service"
_service_dst="/etc/systemd/system/$_service"

if [ "$(id -u)" -ne 0 ]; then
    echo "Root is required to remove a systemd unit; re-running with sudo."
    _here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    exec sudo -- "$_here/uninstall.sh" "$@"
fi

# Best effort: the unit may already be stopped or disabled.
systemctl stop "$_service" 2>/dev/null || true
systemctl disable "$_service" 2>/dev/null || true

if [ -f "$_service_dst" ]; then
    rm -f "$_service_dst"
    systemctl daemon-reload
    echo "Removed $_service_dst"
else
    echo "No unit installed at $_service_dst"
fi

echo "Autoboot is disabled. Re-enable with: ./scripts/autoboot/install.sh"
