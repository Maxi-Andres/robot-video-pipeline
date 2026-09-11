#!/usr/bin/env bash
# Robot-side video: capture from DDS, encode in HARDWARE, push out over SRT.
#
# Runs ON THE ROBOT (its high-level Jetson). This is the half of the pipeline that must be
# L2-adjacent to the robot's DDS — proven, not assumed: from another subnet the robot pings
# fine (1.3 ms) but only 2 of 122 DDS topics are visible. See robot-splunk-docs/RED-Y-DDS.md.
#
#   go2_jpeg_stream (DDS) -> mjpeg_server -> nvjpegdec -> nvv4l2h264enc -> flvmux -> rtmpsink
#                                  |
#                                  +-- HTTP :8093/stream  (the LOW-LATENCY live path)
#
# Two consumers, one reader. The Unitree videohub is request/response, so a second
# go2_jpeg_stream would steal half the frames — hence a passthrough tee instead.
#
# Why this shape:
#   * GStreamer, not ffmpeg: the Jetson has NO ffmpeg, but it does have gst-launch-1.0 and
#     nvv4l2h264enc, the Tegra HARDWARE H.264 encoder (/dev/nvhost-msenc). Hardware encode
#     costs almost no CPU and needs no binary shipped to the robot.
#   * RTMP, not RTSP: rtspclientsink is not installed on the robot.
#   * RTMP, not SRT — and this one was measured, not assumed. SRT would be the better
#     protocol for a lossy WAN link, but the robot ships **libsrt 1.4.0** (Ubuntu 20.04,
#     2020) and mediamtx's own Go SRT implementation REJECTS its handshake: the robot logs
#     "REJECT reported from HS processing" and mediamtx logs nothing at all. Bisected by
#     pointing the same pipeline at an ffmpeg/libsrt listener, where it connected fine — so
#     the incompatibility is libsrt-1.4.0 <-> mediamtx, not the robot or the network.
#     RTMP rides TCP, so retransmission covers packet loss at the cost of latency and of
#     head-of-line blocking. Revisit SRT if libsrt on the robot is ever updated: set
#     PROTO=srt below.
#
# Env:
#   NIC       robot-internal interface for DDS   (default eth0)
#   MAXFPS    cap the capture rate               (default 15 — bounds field bandwidth)
#   PUBLISH_HOST  where mediamtx listens        (required)
#   PROTO         rtmp (default) or srt          (srt needs a newer libsrt on the robot)
#   PUBLISH_PORT  1935 for rtmp, 8890 for srt    (default follows PROTO)
#   STREAM        mediamtx path to publish into  (default robot)
#   BITRATE       H.264 bitrate in bits/s        (default 2000000)
#   CONTROL_RATE  1 = CBR (default), 0 = VBR      (VBR ignores BITRATE in practice)
#   LATENCY       SRT latency budget in ms       (default 300; raise on satellite links)
#   WIDTH/HEIGHT  scale before encoding          (empty = native 1920x1080)
#   IDR_FRAMES    keyframe interval in frames    (default 15 = 1/s at 15fps; lower = faster
#                                                 join for a new viewer, more bitrate)
#   MJPEG_ENABLE  1 (default) = also serve the raw JPEGs over HTTP for the live view
#   MJPEG_PORT    HTTP port for that            (default 8093)
#   MJPEG_FPS     cap for HTTP viewers only     (0 = every frame; does not affect the NVR)
set -uo pipefail    # NOT -e: the supervision loop must survive child failures
cd "$(dirname "$0")/.."

NIC="${NIC:-eth0}"
MAXFPS="${MAXFPS:-15}"
PUBLISH_HOST="${PUBLISH_HOST:-${SRT_HOST:?set PUBLISH_HOST to the machine running mediamtx}}"
PROTO="${PROTO:-rtmp}"
STREAM="${STREAM:-robot}"
BITRATE="${BITRATE:-2000000}"
CONTROL_RATE="${CONTROL_RATE:-1}"
LATENCY="${LATENCY:-300}"
WIDTH="${WIDTH:-}"
HEIGHT="${HEIGHT:-}"
IDR_FRAMES="${IDR_FRAMES:-15}"
MJPEG_ENABLE="${MJPEG_ENABLE:-1}"
export MJPEG_PORT="${MJPEG_PORT:-8093}"
export MJPEG_BIND="${MJPEG_BIND:-0.0.0.0}"
export MJPEG_FPS="${MJPEG_FPS:-0}"

# Scaling happens on the GPU (nvvidconv), so it costs the encoder less work AND less
# bitrate for the same quality — a real latency knob, not just a bandwidth one.
if [ -n "$WIDTH" ] && [ -n "$HEIGHT" ]; then
  SCALE="! video/x-raw(memory:NVMM),width=$WIDTH,height=$HEIGHT "
else
  SCALE=""
fi

# The live view is served by the tee; without it the pipe is just a passthrough cat.
if [ "$MJPEG_ENABLE" = 1 ]; then
  TEE=(python3 "$(dirname "$0")/mjpeg_server.py")
else
  TEE=(cat)
fi
case "$PROTO" in
  rtmp) PUBLISH_PORT="${PUBLISH_PORT:-1935}"
        SINK="flvmux streamable=true ! rtmpsink location=rtmp://${PUBLISH_HOST}:${PUBLISH_PORT}/${STREAM}" ;;
  srt)  PUBLISH_PORT="${PUBLISH_PORT:-8890}"
        SINK="mpegtsmux ! srtsink uri=srt://${PUBLISH_HOST}:${PUBLISH_PORT}?streamid=publish:${STREAM}&latency=${LATENCY} sync=false" ;;
  *)    echo "PROTO must be rtmp or srt (got '$PROTO')" >&2; exit 1 ;;
esac

# CycloneDDS must bind the interface explicitly. ChannelFactory::Init(0, nic) alone
# receives nothing — same hard-won detail as the desktop pipeline.
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$NIC\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>}"

[ -x ./go2_jpeg_stream ] || { echo "build first: UNITREE_SDK2_DIR=~/unitree_sdk2 ./build.sh" >&2; exit 1; }
command -v gst-launch-1.0 >/dev/null || { echo "gst-launch-1.0 missing" >&2; exit 1; }

echo "[robot-video] NIC=$NIC maxfps=$MAXFPS proto=$PROTO -> ${PUBLISH_HOST}:${PUBLISH_PORT}/${STREAM}"
[ "$MJPEG_ENABLE" = 1 ] && echo "[robot-video] low-latency live view: http://<robot>:${MJPEG_PORT}/stream"

running=1
# Kill the process GROUP, not just direct children. The encoder is supervised in a sub-shell
# now, so gst-launch is a grandchild and `pkill -P $$` would leave it alive holding the RTMP
# socket. Under systemd this is belt-and-braces — the default KillMode=control-group already
# tears down the whole cgroup — but it is what makes Ctrl-C on a manual run clean.
cleanup() { running=0; kill -- "-$$" 2>/dev/null || pkill -P $$ 2>/dev/null || true; }
trap cleanup EXIT INT TERM

# The encoder is the half that dies, and it used to take the recording branch down for good.
#
# MEASURED 2026-09-11: `nvv4l2h264enc` aborts with "free(): double free detected in tcache 2"
# at unpredictable intervals — 2 s, 10 s, and over 20 s in three runs of the same pipeline.
# The message lands right after the NVENC banner because that is the encoder's last line;
# the abort is in its teardown, a known L4T fault. Every input was cleared as innocent:
# bitrate, CBR, the nvjpegdec chain on real camera bytes, software jpegdec, 4:2:0 vs 4:2:2,
# restart markers, and rtmpsink against two different servers.
#
# What turned an intermittent crash into a PERMANENT outage was this loop. `gst-launch` dies,
# mjpeg_server takes the EPIPE on its next write, logs "downstream (NVR) closed" and — by
# design, because the live view must survive — keeps running. So the pipeline never ends,
# bash keeps waiting on it, and this loop never gets to iterate. The branch stays dead until
# someone restarts the unit by hand, which is how it was found disabled with NVR_ENABLE=0.
#
# So supervise the encoder SEPARATELY. Restarting it costs a few seconds of recording;
# restarting the whole chain would also blink the live view, which is what you steer by, and
# would re-open the camera. Nothing new is needed to hold the stream meanwhile: the sub-shell
# keeps the same pipe on fd 0 across encoder restarts, so while gst is down mjpeg_server just
# blocks on stdout — exactly what its nvr_writer is documented to be the only thing allowed
# to do — and its bounded queue drops frames rather than growing latency.
encode_and_publish() {
  local fails=0 started rc
  while :; do
    started=$SECONDS
    # do-timestamp=true because the JPEGs arrive with no timestamps of their own and at an
    # irregular cadence. If the publish ever stalls, insert `videorate` after the decoder —
    # that is the GStreamer equivalent of the `-vsync cfr` fix the desktop pipeline needed.
    # config-interval=-1 so SPS/PPS ride with every keyframe: a viewer joining mid-stream
    # otherwise gets "non-existing PPS" and never decodes a frame.
    # control-rate=1 is CBR, and it is the difference between `bitrate` being a setting and
    # being a suggestion. MEASURED 2026-09-11 without it: BITRATE=1500000 configured, and
    # the stream arrived at HQ at 6.5 Mbps with ~193 KB frames against the 234 KB JPEGs it
    # was meant to replace — i.e. the whole point of encoding, 7x less data, was not
    # happening. On a link where bandwidth is the constraint, an encoder that overshoots by
    # 4x is worse than no encoder at all, because it costs the latency too.
    #
    # peak-bitrate is deliberately NOT set: gst-inspect on this robot documents it as
    # "Peak bitrate in variable control-rate", so it applies to VBR only and would be
    # silently ignored here. Switch control-rate to 0 if you ever want that trade.
    gst-launch-1.0 -q \
      fdsrc fd=0 do-timestamp=true ! jpegparse ! nvjpegdec ! nvvidconv $SCALE\
      ! nvv4l2h264enc bitrate="$BITRATE" control-rate="$CONTROL_RATE" \
        insert-sps-pps=1 idrinterval="$IDR_FRAMES" \
        iframeinterval="$IDR_FRAMES" maxperf-enable=1 \
      ! h264parse config-interval=-1 ! $SINK
    rc=$?

    # Exit 0 is a clean EOS: the capture upstream closed, so there is nothing left to
    # encode and the OUTER loop should rebuild the whole chain. Anything else is the
    # encoder falling over on its own while frames are still coming.
    [ "$rc" = 0 ] && { echo "[robot-video] encoder reached EOS (upstream gone)" >&2; return 0; }

    # A run that lasted a while is a one-off; only back-to-back failures mean the encoder
    # cannot start at all, in which case rebuilding the capture side is worth a try.
    if [ $((SECONDS - started)) -ge 30 ]; then
      fails=0
    else
      fails=$((fails + 1))
    fi
    if [ "$fails" -ge 5 ]; then
      echo "[robot-video] encoder failed $fails times in a row (rc=$rc); rebuilding capture" >&2
      return 1
    fi
    echo "[robot-video] encoder died (rc=$rc) after $((SECONDS - started))s; restarting it in 2s" >&2
    sleep 2
  done
}

while [ "$running" = 1 ]; do
  echo "[robot-video] starting capture -> HW encode -> $PROTO publish"
  # go2_jpeg_stream exits after ~8 s without frames (robot's camera service down), which
  # EOFs the pipeline; the loop then republishes cleanly once video is back.
  ./go2_jpeg_stream "$NIC" "$MAXFPS" | "${TEE[@]}" | encode_and_publish || true

  [ "$running" = 1 ] && { echo "[robot-video] pipeline ended; retry in 3s" >&2; sleep 3; }
done
