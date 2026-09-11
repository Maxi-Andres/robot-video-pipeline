#!/usr/bin/env bash
# What the robot actually costs the uplink: bytes arriving on the RTMP socket, sampled from
# the kernel.
#
# This is the number the plan's "robot -> HQ under 2 Mbps" target is written in, and it is
# the only one that cannot be fooled. The browser's WebRTC bitrate is HQ -> browser over
# loopback, and the encoder's `bitrate` property was measured lying by 4.7x (see the
# rate-control block in robot/run-video.sh).
#
#   ./rtmp_bitrate.sh [seconds]        # default 30
set -u
SECS="${1:-30}"
PORT="${PORT:-1935}"

samp() {
  ss -tin state established "( sport = :$PORT )" 2>/dev/null \
    | grep -oE 'bytes_received:[0-9]+' | head -1 | cut -d: -f2
}

a=$(samp)
[ -z "$a" ] && { echo "no publisher connected on :$PORT — is robot-video running?" >&2; exit 1; }
t0=$(date +%s.%N)
timeout "$SECS" tail -f /dev/null      # a blocking wait that needs no sleep builtin
b=$(samp)
t1=$(date +%s.%N)
[ -z "$b" ] && { echo "the publisher disconnected during the window" >&2; exit 1; }

awk -v a="$a" -v b="$b" -v t0="$t0" -v t1="$t1" 'BEGIN {
  d = b - a; dt = t1 - t0; mbps = d * 8 / dt / 1e6
  printf "robot -> HQ on :%s : %.0f kB/s = %.2f Mbps over %.0fs\n", "'"$PORT"'", d/dt/1024, mbps, dt
  printf "plan target < 2 Mbps : %s\n", (mbps < 2 ? "MET" : "NOT MET")
}'
