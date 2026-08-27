#!/usr/bin/env bash
# Install the pipeline as an always-on systemd USER service (no sudo needed).
# After this, the robot->RTSP pipeline auto-starts, auto-restarts on failure, and
# recovers by itself when the robot reconnects.
set -euo pipefail
cd "$(dirname "$0")"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"

DEST="$HOME/.config/systemd/user"
mkdir -p "$DEST"
cp systemd/robot-video-pipeline.service "$DEST/robot-video-pipeline.service"

systemctl --user daemon-reload
systemctl --user enable --now robot-video-pipeline.service
echo "[install] robot-video-pipeline.service enabled and started."
echo

# For the service to also start at BOOT without anyone logging in, user lingering
# must be enabled. That is the ONE step that needs sudo:
if [ "$(loginctl show-user "$USER" -p Linger --value 2>/dev/null)" != "yes" ]; then
  echo "TO SURVIVE REBOOTS (run once, needs sudo):"
  echo "    sudo loginctl enable-linger $USER"
  echo
fi
echo "Status:  systemctl --user status robot-video-pipeline.service"
echo "Logs:    journalctl --user -u robot-video-pipeline.service -f"
