#!/usr/bin/env bash
# Stop the whole stack: Frigate NVR + the robot->RTSP pipeline service.
# The service stays ENABLED (it will start again on boot / start-all.sh). To keep it
# off permanently:  systemctl --user disable robot-nvr.service
set -uo pipefail
cd "$(dirname "$0")"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"

echo "[stop] Frigate"
( cd frigate && docker compose down ) || true

echo "[stop] pipeline service"
systemctl --user stop robot-nvr.service 2>/dev/null || true
# Clean up any stray manually-launched processes too.
pkill -f "go2_jpeg_stream" 2>/dev/null || true
pkill -f "mediamtx.yml"    2>/dev/null || true
echo "[stop] done"
