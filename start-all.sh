#!/usr/bin/env bash
# Start the whole stack: the robot->RTSP pipeline (detached) + the Frigate NVR.
#   Frigate UI:  http://<this-host-ip>:5000   (no login)
set -euo pipefail
cd "$(dirname "$0")"
HERE="$(pwd)"
ROBOT_IP="${ROBOT_IP:-192.168.123.161}"

if ! ping -c1 -W2 "$ROBOT_IP" >/dev/null 2>&1; then
  echo "WARNING: robot $ROBOT_IP not reachable — the camera stream will be empty until it is up." >&2
fi

# 1. Source pipeline (mediamtx + go2_jpeg_stream + ffmpeg). Detached so it survives
#    this shell; skipped if already running.
if pgrep -f "$HERE/go2_jpeg_stream" >/dev/null 2>&1; then
  echo "[start] pipeline already running"
else
  echo "[start] launching robot -> RTSP pipeline"
  setsid bash -c "$HERE/run.sh > /tmp/robot-nvr-run.log 2>&1" </dev/null >/dev/null 2>&1 &
  disown
  sleep 6
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
