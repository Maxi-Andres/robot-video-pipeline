#!/usr/bin/env bash
# Start the whole stack: the robot->RTSP pipeline (as an always-on systemd service)
# + the Frigate NVR. Safe to run repeatedly.
#   Frigate UI:  http://<this-host-ip>:5000
set -euo pipefail
cd "$(dirname "$0")"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
ROBOT_IP="${ROBOT_IP:-192.168.123.161}"

if ! ping -c1 -W2 "$ROBOT_IP" >/dev/null 2>&1; then
  echo "WARNING: robot $ROBOT_IP not reachable — the stream stays empty until it is up" >&2
  echo "         (the pipeline will recover on its own once the robot is back)." >&2
fi

# 1. Pipeline as a supervised systemd user service (auto-restart / self-heal).
if systemctl --user cat robot-nvr.service >/dev/null 2>&1; then
  systemctl --user start robot-nvr.service
else
  echo "[start] first run — installing the systemd service"
  ./install-service.sh
fi

# 2. Frigate NVR.
echo "[start] launching Frigate"
( cd frigate && docker compose up -d )

IP="$(ip -4 -o addr show scope global 2>/dev/null | awk 'NR==1{print $4}' | cut -d/ -f1)"
echo
echo "Ready:"
echo "  Frigate NVR (monitor + recordings):  http://${IP:-127.0.0.1}:5000"
echo "  Raw RTSP (VLC / other NVR):          rtsp://${IP:-127.0.0.1}:8554/robot"
echo "  Browser live (WebRTC):               http://${IP:-127.0.0.1}:8889/robot"
