#!/usr/bin/env bash
# Install the narrow sudo rule that lets can_comms configure can0 without a
# password. Run once per rover user:
#
#   ./scripts/can/install.sh
#
# Only `ip link set can0 ...` is allowed; nothing else gains root access.

set -euo pipefail

_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "$(id -u)" -ne 0 ]; then
    echo "Root is required to install the sudoers rule; re-running with sudo."
    exec sudo -- "$_here/install.sh" "$@"
fi

# SUDO_USER is set when this script re-executes itself through sudo.
_run_user="${SUDO_USER:-${USER:-}}"
if [ -z "$_run_user" ] || [ "$_run_user" = "root" ]; then
    echo "[!] Could not determine the non-root rover user." >&2
    echo "    Run as: sudo -u <rover-user> ... or use sudo from that account." >&2
    exit 1
fi

_ip_bin="$(command -v ip || true)"
if [ -z "$_ip_bin" ]; then
    echo "[!] ip(8) not found; install iproute2 first." >&2
    exit 1
fi

_sudoers="/etc/sudoers.d/rovers-can"
_tmp="$(mktemp)"
trap 'rm -f "$_tmp"' EXIT

cat >"$_tmp" <<EOF
# Installed by rovers-ros scripts/can/install.sh.
# Allow the rover stack to reset can0 without prompting for a password.
$_run_user ALL=(root) NOPASSWD: $_ip_bin link set can0 *
EOF

chmod 0440 "$_tmp"
if ! visudo -cf "$_tmp" >/dev/null; then
    echo "[!] Generated sudoers rule failed validation; not installing it." >&2
    exit 1
fi

install -m 0440 -o root -g root "$_tmp" "$_sudoers"

echo "Installed $_sudoers"
echo "  user:    $_run_user"
echo "  allowed: $_ip_bin link set can0 *"
echo
echo "Test it with:"
echo "  sudo -n $_ip_bin link set can0 down"
echo "  sudo -n $_ip_bin link set can0 up"
