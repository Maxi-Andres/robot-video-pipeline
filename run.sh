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
set -uo pipefail    # NOT -e: the supervision loop must survive child failures
cd "$(dirname "$0")"

NIC="${NIC:-enp4s0}"
MAXFPS="${MAXFPS:-0}"
RTSP="${RTSP:-rtsp://127.0.0.1:8554/robot}"
FFMPEG="./bin/ffmpeg"

# CycloneDDS must bind the robot-network interface, or the SDK receives nothing.
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$NIC\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>}"

[ -x ./go2_jpeg_stream ] || { echo "build first: ./build.sh" >&2; exit 1; }
[ -x ./mediamtx ]        || { echo "mediamtx binary missing (run ./setup.sh)" >&2; exit 1; }
[ -x "$FFMPEG" ]         || { echo "ffmpeg missing (run ./setup.sh)" >&2; exit 1; }

# Start the RTSP server once; it stays up across capture restarts.
./mediamtx ./mediamtx.yml &
MTX_PID=$!

running=1
cleanup() { running=0; kill "$MTX_PID" 2>/dev/null || true; pkill -P $$ 2>/dev/null || true; }
trap cleanup EXIT INT TERM
sleep 1

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
  ./go2_jpeg_stream "$NIC" "$MAXFPS" | "$FFMPEG" -hide_banner -loglevel warning \
    -fflags nobuffer -flags low_delay \
    -f mjpeg -use_wallclock_as_timestamps 1 -i pipe:0 \
    -c:v libx264 -preset ultrafast -tune zerolatency -pix_fmt yuv420p -g 30 \
    -f rtsp -rtsp_transport tcp "$RTSP" || true

  [ "$running" = 1 ] && { echo "[run] capture ended; retry in 3s" >&2; sleep 3; }
done
