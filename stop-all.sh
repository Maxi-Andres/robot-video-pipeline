#!/usr/bin/env bash
# Stop the whole stack: Frigate NVR + the robot->RTSP pipeline.
set -uo pipefail
cd "$(dirname "$0")"

echo "[stop] Frigate"
( cd frigate && docker compose down ) || true

echo "[stop] pipeline"
pkill -f "$(pwd)/go2_jpeg_stream" 2>/dev/null || true
pkill -f "$(pwd)/bin/ffmpeg"      2>/dev/null || true
pkill -f "$(pwd)/mediamtx"        2>/dev/null || true
echo "[stop] done"
