#!/usr/bin/env bash
# Start the full robot -> NVR bridge:
#   Go2 JPEG (videohub, DDS) -> ffmpeg encode H.264 -> mediamtx RTSP.
#
# The NVR then pulls  rtsp://<this-host-ip>:8554/robot
# Browser (no NVR):   http://<this-host-ip>:8888/robot  (HLS)  or  :8889  (WebRTC)
#
# Env overrides:
#   NIC     robot-network interface   (default enp4s0)
#   MAXFPS  cap the poll rate         (default 0 = as fast as the robot answers)
#   RTSP    target path on mediamtx   (default rtsp://127.0.0.1:8554/robot)
set -euo pipefail
cd "$(dirname "$0")"

NIC="${NIC:-enp4s0}"
MAXFPS="${MAXFPS:-0}"
RTSP="${RTSP:-rtsp://127.0.0.1:8554/robot}"
FFMPEG="./bin/ffmpeg"

# CycloneDDS must bind the robot-network interface, or the SDK receives nothing.
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$NIC\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>}"

[ -x ./go2_jpeg_stream ] || { echo "build first: ./build.sh" >&2; exit 1; }
[ -x ./mediamtx ]        || { echo "mediamtx binary missing" >&2; exit 1; }
[ -x "$FFMPEG" ]         || { echo "ffmpeg missing at $FFMPEG" >&2; exit 1; }

# Start the RTSP server; stop it when this script exits.
./mediamtx ./mediamtx.yml &
MTX_PID=$!
trap 'kill $MTX_PID 2>/dev/null || true' EXIT INT TERM
sleep 1

echo "[run] streaming Go2 JPEG via $NIC -> $RTSP"
# JPEG frames in -> H.264 out. zerolatency/ultrafast + tiny GOP keep latency low and
# give the NVR frequent keyframes to start on. wallclock timestamps pace the VFR input.
./go2_jpeg_stream "$NIC" "$MAXFPS" | "$FFMPEG" -hide_banner -loglevel warning \
  -fflags nobuffer -flags low_delay \
  -f mjpeg -use_wallclock_as_timestamps 1 -i pipe:0 \
  -c:v libx264 -preset ultrafast -tune zerolatency -pix_fmt yuv420p -g 30 \
  -f rtsp -rtsp_transport tcp "$RTSP"
