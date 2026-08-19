#!/usr/bin/env bash
# Robot-side video: capture from DDS, encode in HARDWARE, push out over SRT.
#
# Runs ON THE ROBOT (its high-level Jetson). This is the half of the pipeline that must be
# L2-adjacent to the robot's DDS — proven, not assumed: from another subnet the robot pings
# fine (1.3 ms) but only 2 of 122 DDS topics are visible. See SplunkCode/RED-Y-DDS.md.
#
#   go2_jpeg_stream (DDS) -> nvjpegdec -> nvv4l2h264enc -> mpegtsmux -> srtsink -> mediamtx
#
# Why this shape:
#   * GStreamer, not ffmpeg: the Jetson has NO ffmpeg, but it does have gst-launch-1.0 and
#     nvv4l2h264enc, the Tegra HARDWARE H.264 encoder (/dev/nvhost-msenc). Hardware encode
#     costs almost no CPU and needs no binary shipped to the robot.
#   * SRT, not RTSP: rtspclientsink is not installed, srtsink is — and SRT is the right
#     protocol anyway for a lossy WAN link (Starlink), where RTSP-over-TCP degrades badly.
#     mediamtx accepts SRT publishers and re-publishes as RTSP/HLS/WebRTC to consumers.
#
# Env:
#   NIC       robot-internal interface for DDS   (default eth0)
#   MAXFPS    cap the capture rate               (default 15 — bounds field bandwidth)
#   SRT_HOST  where mediamtx listens             (required)
#   SRT_PORT  mediamtx SRT port                  (default 8890)
#   STREAM    mediamtx path to publish into      (default robot)
#   BITRATE   H.264 bitrate in bits/s            (default 2000000)
#   LATENCY   SRT latency budget in ms           (default 300; raise on satellite links)
set -uo pipefail    # NOT -e: the supervision loop must survive child failures
cd "$(dirname "$0")/.."

NIC="${NIC:-eth0}"
MAXFPS="${MAXFPS:-15}"
SRT_HOST="${SRT_HOST:?set SRT_HOST to the machine running mediamtx}"
SRT_PORT="${SRT_PORT:-8890}"
STREAM="${STREAM:-robot}"
BITRATE="${BITRATE:-2000000}"
LATENCY="${LATENCY:-300}"

# CycloneDDS must bind the interface explicitly. ChannelFactory::Init(0, nic) alone
# receives nothing — same hard-won detail as the desktop pipeline.
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$NIC\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>}"

[ -x ./go2_jpeg_stream ] || { echo "build first: UNITREE_SDK2_DIR=~/unitree_sdk2 ./build.sh" >&2; exit 1; }
command -v gst-launch-1.0 >/dev/null || { echo "gst-launch-1.0 missing" >&2; exit 1; }

SRT_URI="srt://${SRT_HOST}:${SRT_PORT}?streamid=publish:${STREAM}&latency=${LATENCY}"
echo "[robot-video] NIC=$NIC maxfps=$MAXFPS -> $SRT_URI"

running=1
cleanup() { running=0; pkill -P $$ 2>/dev/null || true; }
trap cleanup EXIT INT TERM

while [ "$running" = 1 ]; do
  echo "[robot-video] starting capture -> HW encode -> SRT publish"
  # go2_jpeg_stream exits after ~8 s without frames (robot's camera service down), which
  # EOFs the pipeline; the loop then republishes cleanly once video is back.
  # do-timestamp=true because the JPEGs arrive with no timestamps of their own and at an
  # irregular cadence. If the publish ever stalls, insert `videorate` after the decoder —
  # that is the GStreamer equivalent of the `-vsync cfr` fix the desktop pipeline needed.
  ./go2_jpeg_stream "$NIC" "$MAXFPS" | gst-launch-1.0 -q \
    fdsrc fd=0 do-timestamp=true ! jpegparse ! nvjpegdec ! nvvidconv \
    ! nvv4l2h264enc bitrate="$BITRATE" insert-sps-pps=1 idrinterval=15 iframeinterval=15 maxperf-enable=1 \
    ! h264parse config-interval=1 ! mpegtsmux ! srtsink uri="$SRT_URI" sync=false || true

  [ "$running" = 1 ] && { echo "[robot-video] pipeline ended; retry in 3s" >&2; sleep 3; }
done
