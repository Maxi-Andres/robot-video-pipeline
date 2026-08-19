#!/usr/bin/env bash
# Supervised robot -> NVR pipeline. Keeps mediamtx running persistently and
# auto-restarts the capture->encode->publish chain whenever it ends (e.g. the robot
# drops off the network and comes back). Designed to run forever under systemd.
#
#   Go2 JPEG (videohub, DDS) -> ffmpeg encode H.264 -> mediamtx RTSP (:8554/robot)
#
# Env overrides:
#   NIC     robot-network interface   (default enp4s0)
#   MAXFPS  cap the poll rate         (default 0 = as fast as the robot answers)
#   RTSP    target path on mediamtx   (default rtsp://127.0.0.1:8554/robot)
#   SERVER_ONLY=1  run mediamtx ONLY, with no local capture. Use this once the capture
#                  moved onto the robot (robot/run-video.sh pushes in over SRT), which is
#                  what lets the robot live on a different network entirely.
set -uo pipefail    # NOT -e: the supervision loop must survive child failures
cd "$(dirname "$0")"

NIC="${NIC:-enp4s0}"
MAXFPS="${MAXFPS:-0}"
RTSP="${RTSP:-rtsp://127.0.0.1:8554/robot}"
FFMPEG="./bin/ffmpeg"

# CycloneDDS must bind the robot-network interface, or the SDK receives nothing.
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$NIC\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>}"

SERVER_ONLY="${SERVER_ONLY:-0}"

[ -x ./mediamtx ] || { echo "mediamtx binary missing (run ./setup.sh)" >&2; exit 1; }
if [ "$SERVER_ONLY" != 1 ]; then
  # Only the local-capture path needs these; in SERVER_ONLY mode the robot encodes.
  [ -x ./go2_jpeg_stream ] || { echo "build first: ./build.sh" >&2; exit 1; }
  [ -x "$FFMPEG" ]         || { echo "ffmpeg missing (run ./setup.sh)" >&2; exit 1; }
fi

# Start the RTSP server once; it stays up across capture restarts.
./mediamtx ./mediamtx.yml &
MTX_PID=$!

running=1
cleanup() { running=0; kill "$MTX_PID" 2>/dev/null || true; pkill -P $$ 2>/dev/null || true; }
trap cleanup EXIT INT TERM
sleep 1

if [ "$SERVER_ONLY" = 1 ]; then
  echo "[run] SERVER_ONLY: mediamtx up; waiting for the robot to publish over SRT (:8890)"
  # Nothing to supervise locally — just die if mediamtx does, so systemd restarts it.
  wait "$MTX_PID"
  echo "[run] mediamtx exited" >&2
  exit 1
fi

echo "[run] supervisor up; NIC=$NIC -> $RTSP"
while [ "$running" = 1 ]; do
  # If mediamtx died, exit so the outer supervisor (systemd) restarts the whole unit.
  if ! kill -0 "$MTX_PID" 2>/dev/null; then
    echo "[run] mediamtx exited — bailing so systemd restarts everything" >&2
    exit 1
  fi

  echo "[run] starting capture -> encode -> publish"
  # go2_jpeg_stream exits after a few seconds without frames (robot offline); ffmpeg
  # then gets EOF and exits. The loop restarts the pair, so the stream re-publishes
  # cleanly as soon as the robot is back.
  # NOTE: force constant output rate (-vsync cfr -r 15). Without it, the JPEG frames'
  # wallclock timestamps arrive irregularly (esp. the G1) and ffmpeg stalls its RTSP
  # output -> mediamtx drops the publisher with an i/o timeout and the stream dies a
  # few seconds after starting. CFR normalizes the cadence and keeps the publish alive.
  ./go2_jpeg_stream "$NIC" "$MAXFPS" | "$FFMPEG" -hide_banner -loglevel warning \
    -f mjpeg -use_wallclock_as_timestamps 1 -i pipe:0 \
    -c:v libx264 -preset ultrafast -tune zerolatency -pix_fmt yuv420p \
    -vsync cfr -r 15 -g 15 \
    -f rtsp -rtsp_transport tcp "$RTSP" || true

  [ "$running" = 1 ] && { echo "[run] capture ended; retry in 3s" >&2; sleep 3; }
done
