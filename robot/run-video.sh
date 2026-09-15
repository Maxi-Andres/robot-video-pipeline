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
#   SOURCE    jpeg (default) or multicast       — see the SOURCE block below
#   NIC       robot-internal interface for DDS   (default eth0)
#   MAXFPS    cap the capture rate               (default 15 — bounds field bandwidth)
#   PUBLISH_HOST  where mediamtx listens        (required)
#   PROTO         rtmp (default) or srt          (srt needs a newer libsrt on the robot)
#   PUBLISH_PORT  1935 for rtmp, 8891 for srt    (default follows PROTO)
#   SRT_STREAMID  empty (default) for srt-live-transmit; publish:<path> only for mediamtx
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
#   NVR_FPS       rate mjpeg_server feeds the encoder (default 5). Read here ONLY to
#                 pre-divide the bitrate — see the rate-control block below.
set -uo pipefail    # NOT -e: the supervision loop must survive child failures
cd "$(dirname "$0")/.."

# WHERE THE PICTURE COMES FROM. Two sources, and they are not variations of each other:
#
#   jpeg       the camera's videohub, one JPEG per GetImageSample RPC, then decoded and
#              re-encoded to H.264 on the Jetson. What has always run.
#   multicast  the H.264 the Go2 ALREADY encodes, published as RTP on a multicast group
#              (Unitree's Multimedia Services). Nothing is decoded and nothing is
#              re-encoded: the bytes are depayloaded, parsed and muxed straight out.
#
# MEASURED 2026-09-14, same robot, same link:
#
#              | videohub + re-encode | multicast native
#   resolution | 1080p                | 1280x720
#   frames     | ~4.7 fps             | 13.9 fps
#   robot->HQ  | 1.42 Mbps            | 2.02 Mbps
#   per frame  | 0.30 Mbit            | 0.145 Mbit   (half)
#   Jetson     | JPEG decode + encode | nothing, it is a passthrough
#
# The reason to care is the frames, the bandwidth per frame, and the Jetson's load. It is NOT
# latency: measured the same day by two independent methods, `multicast` and `jpeg` have the
# SAME glass-to-glass. The "videohub is ~650 ms, about 90% of it" line that used to sit here
# was wrong twice over — that number came from comparing against a clock that was not ours
# (the same method produced -710 ms and -1400 ms, both impossible), and skipping the videohub
# entirely changed nothing. There is latency upstream of both paths and it is UNMEASURED.
#
# WHAT `multicast` COSTS YOU ON A FIELD LINK, and it is not NAT. The multicast never leaves
# the robot: udpsrc joins the group on $NIC, which is the robot's own internal bus, and what
# crosses LTE/Starlink is the same unicast RTMP over TCP as always. But the ENCODER is now
# Unitree's, inside the robot, and it has no knob we can reach:
#
#   BITRATE / NVR_FPS / MAXFPS / IDR_FRAMES apply to `jpeg` ONLY.  On `multicast` they are
#   silently inert -- see the ENC_BITRATE=0 branch below.
#
# So on `multicast` you cannot turn the video down, and it sends MORE: 1.78 Mbps against 1.43,
# and 14.25 fps against 4.5. On a lossy link that matters, because the freezing on this system
# is loss with TCP retransmission and sending less loses less. If a field link starts freezing,
# the move is `SOURCE=jpeg`, where the knobs exist. Nothing was deleted to make room for
# `multicast`, precisely so that fallback stays one variable away.
#
# NOT the DDS topic rt/frontvideostream. That one is a dead end: it exists and a subscriber
# MATCHES its publisher, but no sample is ever delivered — reproduced from INSIDE the Jetson,
# which is what the old plan assumed would fix it. See robot-splunk-docs/PLAN-VIDEO.md.
#
# Default stays `jpeg` until the latency of `multicast` is measured rather than assumed.
SOURCE="${SOURCE:-jpeg}"
MCAST_ADDR="${MCAST_ADDR:-230.1.1.1}"
MCAST_PORT="${MCAST_PORT:-1720}"

NIC="${NIC:-eth0}"
MAXFPS="${MAXFPS:-15}"
PUBLISH_HOST="${PUBLISH_HOST:-${SRT_HOST:?set PUBLISH_HOST to the machine running mediamtx}}"
PROTO="${PROTO:-rtmp}"
STREAM="${STREAM:-robot}"
SRT_STREAMID="${SRT_STREAMID:-}"
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
# These two used to have no default and no export: they reached mjpeg_server only because
# systemd loads video.env for the whole service and children inherit it. Run this script by
# hand, outside systemd, and the live view silently went back to full resolution.
export MJPEG_WIDTH="${MJPEG_WIDTH:-0}"
export MJPEG_QUALITY="${MJPEG_QUALITY:-75}"

# Scaling happens on the GPU (nvvidconv), so it costs the encoder less work AND less
# bitrate for the same quality — a real latency knob, not just a bandwidth one.
if [ -n "$WIDTH" ] && [ -n "$HEIGHT" ]; then
  SCALE="! video/x-raw(memory:NVMM),width=$WIDTH,height=$HEIGHT "
else
  SCALE=""
fi

# There is no JPEG stream to tee in `multicast` — the robot hands us H.264 directly, so the
# MJPEG live view simply does not exist on that source. Say so instead of failing obscurely.
if [ "$SOURCE" = multicast ] && [ "$MJPEG_ENABLE" = 1 ]; then
  echo "[robot-video] SOURCE=multicast carries no JPEG: the MJPEG live view is off" >&2
  MJPEG_ENABLE=0
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
  srt)  PUBLISH_PORT="${PUBLISH_PORT:-8891}"
        # streamid is EMPTY by default, and that is the right default now. It only ever
        # existed to tell mediamtx which path to publish into — and mediamtx's own SRT
        # listener rejects this robot's libsrt 1.4.0 handshake, so that target does not
        # work. What does work is srt-live-transmit (see systemd/srt-bridge.service), which
        # is a plain listener with a single destination and needs no streamid at all.
        # Set SRT_STREAMID=publish:<path> only if you ever point this back at mediamtx.
        SRT_URI="srt://${PUBLISH_HOST}:${PUBLISH_PORT}?latency=${LATENCY}"
        [ -n "$SRT_STREAMID" ] && SRT_URI="${SRT_URI}&streamid=${SRT_STREAMID}"
        # alignment=7 is NOT optional, and leaving it out fails in the most confusing way
        # possible. SRT in live mode carries at most 1316 bytes per message (7 x 188 TS
        # packets); mpegtsmux without alignment emits buffers of whatever size it likes, and
        # srtsink silently drops the ones that do not fit. The buffers that do not fit are
        # the BIG ones — the keyframes.
        #
        # MEASURED 2026-09-14, same pipeline, 26 s, only this property changed:
        #     mpegtsmux              ->  0 IDR,  0 SPS,  0 PPS
        #     mpegtsmux alignment=7  ->  8 IDR,  8 SPS,  8 PPS
        #
        # With no keyframes and no parameter sets nothing can start decoding: the browser
        # received 42388 packets and 49 MB with framesDecoded stuck at 0, and ffprobe said
        # "non-existing PPS 0 referenced". The stream looks alive at every layer except the
        # one that matters.
        SINK="mpegtsmux alignment=7 ! srtsink uri=${SRT_URI} sync=false" ;;
  *)    echo "PROTO must be rtmp or srt (got '$PROTO')" >&2; exit 1 ;;
esac

# CycloneDDS must bind the interface explicitly. ChannelFactory::Init(0, nic) alone
# receives nothing — same hard-won detail as the desktop pipeline.
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$NIC\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>}"

if [ "$SOURCE" = jpeg ]; then
  [ -x ./go2_jpeg_stream ] || { echo "build first: UNITREE_SDK2_DIR=~/unitree_sdk2 ./build.sh" >&2; exit 1; }
fi
command -v gst-launch-1.0 >/dev/null || { echo "gst-launch-1.0 missing" >&2; exit 1; }

# ---------------------------------------------------------------------------------------
# Rate control has no time base on this stack, so the bitrate has to be pre-divided.
#
# MEASURED ON THE ROBOT 2026-09-11: `fdsrc ! jpegparse` negotiates framerate=1/1, because a
# raw JPEG stream carries no timing of its own. nvv4l2h264enc therefore spends the whole
# `bitrate` budget on EVERY frame. At the 4.7 fps actually fed, that predicts
# 1.5 Mbps x 4.7 = 7.05 Mbps — and 6.97 Mbps was what arrived at HQ. The property is
# honoured; the time base is what is wrong. control-rate=1 (CBR) does not help, because CBR
# is still constant with respect to that same broken clock.
#
# Every honest way of fixing the time base FAILED here (GStreamer 1.16, L4T), each measured:
#   * framerate on the NVMM caps  -> nvvidconv cannot convert framerate; negotiation fails
#                                    and the pipeline never starts (0 frames)
#   * capssetter, join or replace -> encodes exactly ONE frame. Measured both ways.
#   * videorate in system memory  -> "Internal data stream error" (0 frames)
#
# So divide instead: against a fixed 1/1 time base, a per-frame budget of BITRATE/fps yields
# BITRATE per second. Measured 1452 kbps against a 1500 target while keeping all 94 frames —
# the bitrate falls, the picture rate does not.
#
# THE CATCH, and the reason this is printed at startup instead of hidden: the divisor has to
# match the rate actually reaching the encoder, and being wrong scales the bitrate by exactly
# that factor. mjpeg_server gates the recording branch to NVR_FPS, so that is the rate
# whenever it is the tee; with MJPEG_ENABLE=0 the tee is a plain passthrough and the encoder
# sees the capture rate instead.
# ---------------------------------------------------------------------------------------
NVR_FPS="${NVR_FPS:-5}"
# `multicast` never encodes, so there is no bitrate to divide and nothing to report. Saying
# it anyway would be a line of output about a knob that does not exist on that source.
if [ "$SOURCE" != jpeg ]; then
  ENC_FPS=0
  ENC_BITRATE=0
elif [ "$MJPEG_ENABLE" = 1 ] && awk "BEGIN{exit !($NVR_FPS > 0)}"; then
  ENC_FPS="$NVR_FPS"                 # the tee gates the encoder to this
elif [ "$MAXFPS" != 0 ]; then
  ENC_FPS="$MAXFPS"                  # no gate, but the capture is capped
else
  ENC_FPS=15
  echo "[robot-video] WARNING: capture uncapped and no tee gate; assuming ${ENC_FPS} fps" \
       "for rate control. If the real rate differs, the bitrate is off by that ratio." >&2
fi
if [ "$SOURCE" = jpeg ]; then
  ENC_BITRATE=$(awk "BEGIN{printf \"%d\", $BITRATE / $ENC_FPS}")
  echo "[robot-video] rate control: ${BITRATE} bps at ${ENC_FPS} fps" \
       "-> ${ENC_BITRATE} per frame (the encoder's time base is 1/1 on this stack)"
fi

# The half of the pipeline BEFORE the muxer, which is the only part the source changes.
#
# `multicast` is a passthrough: depayload the RTP, parse the Annex-B, done. No decoder and no
# encoder appear at all — which is also why the double free in nvv4l2h264enc (still unexplained,
# only mitigated by the supervisor below) cannot happen on this source.
#
# config-interval=-1 so SPS/PPS ride with every keyframe: a viewer joining mid-stream otherwise
# gets "non-existing PPS" and never decodes a frame. It matters on BOTH sources.
case "$SOURCE" in
  jpeg)
    HEAD="fdsrc fd=0 do-timestamp=true ! jpegparse ! nvjpegdec ! nvvidconv $SCALE\
      ! nvv4l2h264enc bitrate=$ENC_BITRATE control-rate=$CONTROL_RATE \
        insert-sps-pps=1 idrinterval=$IDR_FRAMES iframeinterval=$IDR_FRAMES maxperf-enable=1 \
      ! h264parse config-interval=-1" ;;
  multicast)
    HEAD="udpsrc address=$MCAST_ADDR port=$MCAST_PORT multicast-iface=$NIC \
      ! application/x-rtp,media=video,encoding-name=H264,payload=96 \
      ! rtph264depay ! h264parse config-interval=-1" ;;
  *) echo "SOURCE must be jpeg or multicast (got '$SOURCE')" >&2; exit 1 ;;
esac

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
# Elapsed time from /proc/uptime, NOT from $SECONDS. MEASURED on the robot 2026-09-11: its
# clock jumped backwards from September to January mid-run and the log printed
# "encoder died after -1788887772s", which made every run look instantaneous and tripped the
# give-up counter immediately. /proc/uptime is monotonic and survives that.
uptime_s() { awk '{print int($1)}' /proc/uptime; }

# Ends the capture -> tee -> encoder pipeline so the outer loop can rebuild it. Kills the
# children of the MAIN shell ($$ is not rewritten in a sub-shell), which includes this
# sub-shell itself — that is fine, we are on our way out either way. A blink of the live
# view is the price, and it is only paid on the paths that already gave up.
end_chain() { pkill -P $$ 2>/dev/null || true; }

encode_and_publish() {
  local fails=0 started rc elapsed
  while :; do
    started=$(uptime_s)
    # do-timestamp=true because the JPEGs arrive with no timestamps of their own and at an
    # irregular cadence. If the publish ever stalls, insert `videorate` after the decoder —
    # that is the GStreamer equivalent of the `-vsync cfr` fix the desktop pipeline needed.
    # config-interval=-1 so SPS/PPS ride with every keyframe: a viewer joining mid-stream
    # otherwise gets "non-existing PPS" and never decodes a frame.
    # bitrate is ENC_BITRATE, not BITRATE — see the block above main() for why.
    #
    # peak-bitrate is deliberately NOT set: gst-inspect on this robot documents it as
    # "Peak bitrate in variable control-rate", so it applies to VBR only and would be
    # silently ignored here. Switch CONTROL_RATE to 0 if you ever want that trade.
    gst-launch-1.0 -q $HEAD ! $SINK
    rc=$?

    # Exit 0 is a clean EOS: nothing left to encode, so the OUTER loop should rebuild the
    # whole chain. Anything else is the encoder falling over while frames are still coming.
    #
    # end_chain, not a bare return: mjpeg_server survives a broken pipe on purpose, so a
    # return alone leaves the shell waiting on a pipeline that will never finish and the
    # branch stays dead. That is the original defect, and it bites at BOTH exits of this
    # function — the give-up path and this one.
    [ "$rc" = 0 ] && { echo "[robot-video] encoder reached EOS (upstream gone)" >&2; end_chain; return 0; }

    elapsed=$(( $(uptime_s) - started ))
    [ "$elapsed" -lt 0 ] && elapsed=0

    # A run that lasted a while is a one-off; only back-to-back failures mean the encoder
    # cannot start at all, in which case rebuilding the capture side is worth a try.
    if [ "$elapsed" -ge 30 ]; then
      fails=0
    else
      fails=$((fails + 1))
    fi
    if [ "$fails" -ge 5 ]; then
      # Returning is NOT enough, and this is the same trap that made the original loop
      # useless one level down: mjpeg_server swallows the EPIPE and stays alive, so the
      # pipeline never ends and the outer `while` never gets to rebuild anything. MEASURED
      # 2026-09-12: the branch died here and stayed dead, with "dropped to NVR" climbing.
      # Kill the whole chain — including this sub-shell, which is also a child of $$ — so
      # the pipeline really ends. A blink of the live view is the price of last resort.
      echo "[robot-video] encoder failed $fails times in a row (rc=$rc); rebuilding capture" >&2
      end_chain
      return 1
    fi
    echo "[robot-video] encoder died (rc=$rc) after ${elapsed}s; restarting it in 2s" >&2
    sleep 2
  done
}

while [ "$running" = 1 ]; do
  if [ "$SOURCE" = multicast ]; then
    echo "[robot-video] starting multicast passthrough -> $PROTO publish"
    # No capture process and no tee: gst joins the multicast group itself, so the whole
    # chain is the one supervised command.
    encode_and_publish || true
  else
    echo "[robot-video] starting capture -> HW encode -> $PROTO publish"
    # go2_jpeg_stream exits after ~8 s without frames (robot's camera service down), which
    # EOFs the pipeline; the loop then republishes cleanly once video is back.
    ./go2_jpeg_stream "$NIC" "$MAXFPS" | "${TEE[@]}" | encode_and_publish || true
  fi

  [ "$running" = 1 ] && { echo "[robot-video] pipeline ended; retry in 3s" >&2; sleep 3; }
done
