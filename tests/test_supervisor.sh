#!/usr/bin/env bash
# Exercise run-video.sh's encoder supervision with stubs, on the real file.
#
#   bash tests/test_supervisor.sh
#
# The encoder aborts at unpredictable intervals (a known L4T fault in nvv4l2h264enc that we
# could not pin down), so what matters is not that it never dies but that dying costs
# seconds of recording instead of the whole branch, and never blinks the live view.
#
# What must hold, and what none of it could do before the change:
#   1. the encoder crashing does NOT kill the capture side (the live view must survive)
#   2. a crash is retried, so an intermittent fault costs seconds, not the whole branch
#   3. five back-to-back crashes give up and rebuild the capture chain
#   4. a clean EOS unwinds instead of spinning
set -u
BASE=$(mktemp -d)
trap 'rm -rf "$BASE"' EXIT
REPO=$(cd "$(dirname "$0")/.." && pwd)

mkdir -p "$BASE/robot" "$BASE/bin"
cp "$REPO/robot/run-video.sh" "$BASE/robot/"

# Stub capture: never exits on its own, so anything that ends the run came from the encoder.
cat >"$BASE/go2_jpeg_stream" <<'EOF'
#!/usr/bin/env bash
echo "[go2_jpeg_stream] stub nic=$1 max_fps=$2" >&2
while :; do printf '\xff\xd8frame\xff\xd9'; sleep 0.05; done
EOF
chmod +x "$BASE/go2_jpeg_stream"

# Stub encoder: behaviour driven by a counter file so each case can script the failures.
cat >"$BASE/bin/gst-launch-1.0" <<'EOF'
#!/usr/bin/env bash
n=$(cat "$COUNTER" 2>/dev/null || echo 0); n=$((n+1)); echo "$n" >"$COUNTER"
echo "[gst stub] run #$n" >&2
head -c 16 >/dev/null || true          # consume a little of the stream
if [ "$n" -le "${CRASHES:-0}" ]; then
  exit 134                             # SIGABRT, the double free
fi
# "healthy" has to mean STILL RUNNING, not "exited 0" — otherwise the case never tests
# what it claims to: that a live encoder leaves the capture side alone.
if [ "${HEALTHY_BLOCKS:-1}" = 1 ]; then cat >/dev/null; fi
exit 0
EOF
chmod +x "$BASE/bin/gst-launch-1.0"

# Stub tee that behaves like the REAL mjpeg_server: it catches the broken pipe and STAYS
# ALIVE, because the live view has to survive the encoder. Using `cat` here (MJPEG_ENABLE=0)
# is what let the give-up bug through — cat exits on EPIPE, so the pipeline ended by itself
# and the rebuild looked like it worked.
cat >"$BASE/robot/mjpeg_server.py" <<'EOF'
import sys, time
out = sys.stdout.buffer
while True:
    b = sys.stdin.buffer.read(64)
    if not b:
        break
    try:
        out.write(b); out.flush()
    except (BrokenPipeError, OSError):
        sys.stderr.write("[mjpeg] downstream (NVR) closed\n"); sys.stderr.flush()
        while True:      # the real server keeps serving HTTP forever
            time.sleep(1)
EOF

export PATH="$BASE/bin:$PATH"
export PUBLISH_HOST=127.0.0.1 MJPEG_ENABLE=1 COUNTER="$BASE/n"

run_case() {
  local name="$1" crashes="$2" secs="$3"
  : >"$COUNTER"
  CRASHES="$crashes" timeout "$secs" bash "$BASE/robot/run-video.sh" \
    >"$BASE/out" 2>&1
  echo "--- $name (crashes=$crashes) ---"
  grep -cE 'gst stub. run #' "$BASE/out" | sed 's/^/  encoder starts: /'
  grep -c 'go2_jpeg_stream. stub' "$BASE/out" | sed 's/^/  capture starts: /'
  grep -E 'restarting it in|rebuilding capture|reached EOS|pipeline ended' "$BASE/out" \
    | sed 's/^/  | /' | head -8
  echo
}

echo "REQUIREMENT 1 is the one that matters: capture starts exactly once while the"
echo "encoder restarts several times."
echo
run_case "intermittent: 3 crashes, then healthy" 3 12
HEALTHY_BLOCKS=0 run_case "persistent: 7 crashes in a row" 7 14
HEALTHY_BLOCKS=0 run_case "clean EOS on the first run"     0 8
