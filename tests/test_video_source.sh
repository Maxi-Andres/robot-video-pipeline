#!/usr/bin/env bash
# What pipeline does run-video.sh actually build for each SOURCE?
#
#   bash tests/test_video_source.sh
#
# The two sources are not variations of each other — one decodes and re-encodes on the Jetson,
# the other is a passthrough — so the checks are about what must and must NOT appear. In
# particular `multicast` must contain no decoder and no encoder at all: if nvjpegdec ever shows
# up there, the source is doing the very work it exists to avoid, and the unexplained double
# free in nvv4l2h264enc comes back with it.
#
# This also pins the ordering bug that bit during implementation: the pipeline used to be built
# BEFORE ENC_BITRATE was computed, so the jpeg source came out with an empty bitrate.
set -u
BASE=$(mktemp -d); trap 'rm -rf "$BASE"' EXIT
REPO=$(cd "$(dirname "$0")/.." && pwd)

mkdir -p "$BASE/robot" "$BASE/bin"
cp "$REPO/robot/run-video.sh" "$BASE/robot/"
printf '#!/bin/sh\nsleep 30\n' >"$BASE/go2_jpeg_stream"; chmod +x "$BASE/go2_jpeg_stream"
# The stub records the pipeline it was handed and then blocks, so the supervisor sees a healthy
# encoder and the script does not spin.
cat >"$BASE/bin/gst-launch-1.0" <<'EOF'
#!/usr/bin/env bash
printf '%s\n' "$*" >> "$PIPELINE_OUT"
sleep 30
EOF
chmod +x "$BASE/bin/gst-launch-1.0"
export PATH="$BASE/bin:$PATH" PUBLISH_HOST=127.0.0.1

fail=0
check() {
  local label="$1" expect="$2" line="$3"
  case "$line" in
    *"$expect"*) printf '    ok      %s\n' "$label" ;;
    *) printf '    MISSING %s -- expected to find: %s\n' "$label" "$expect"; fail=1 ;;
  esac
}
refute() {
  local label="$1" expect="$2" line="$3"
  case "$line" in
    *"$expect"*) printf '    PRESENT %s -- must NOT contain: %s\n' "$label" "$expect"; fail=1 ;;
    *) printf '    ok      %s\n' "$label" ;;
  esac
}

capture() {
  export PIPELINE_OUT="$BASE/pipeline.txt"; : >"$PIPELINE_OUT"
  env "$@" PUBLISH_HOST=127.0.0.1 PIPELINE_OUT="$PIPELINE_OUT" \
    timeout 5 bash "$BASE/robot/run-video.sh" >"$BASE/log" 2>&1
  head -1 "$PIPELINE_OUT"
}

echo "SOURCE=jpeg (the path that has always run)"
line=$(capture SOURCE=jpeg MJPEG_ENABLE=0 BITRATE=1500000 NVR_FPS=5 MAXFPS=15)
check  "decodes the camera's JPEG" "nvjpegdec"            "$line"
check  "encodes H.264 in hardware" "nvv4l2h264enc"        "$line"
check  "bitrate is filled in"      "bitrate=100000"       "$line"
refute "no empty bitrate"          "bitrate= "            "$line"
check  "SPS/PPS on every keyframe" "config-interval=-1"   "$line"

echo
echo "SOURCE=multicast (the robot's own H.264, passthrough)"
line=$(capture SOURCE=multicast BITRATE=1500000)
check  "joins the multicast group" "udpsrc address=230.1.1.1 port=1720" "$line"
check  "binds the robot's NIC"     "multicast-iface=eth0"               "$line"
check  "depayloads the RTP"        "rtph264depay"                       "$line"
check  "SPS/PPS on every keyframe" "config-interval=-1"                 "$line"
refute "NO JPEG decoder"           "nvjpegdec"                          "$line"
refute "NO encoder"                "nvv4l2h264enc"                      "$line"

echo
echo "SOURCE=multicast turns the MJPEG live view off (there is no JPEG to serve)"
capture SOURCE=multicast MJPEG_ENABLE=1 >/dev/null
grep -q "carries no JPEG" "$BASE/log" \
  && echo "    ok      says so on stderr" \
  || { echo "    MISSING the warning"; fail=1; }

echo
echo "an unknown SOURCE fails loudly instead of guessing"
capture SOURCE=nonsense >/dev/null
grep -q "SOURCE must be jpeg or multicast" "$BASE/log" \
  && echo "    ok      rejected" \
  || { echo "    MISSING the rejection"; fail=1; }

echo
[ "$fail" = 0 ] && echo "all checks passed" || echo "FAILURES above"
exit "$fail"
