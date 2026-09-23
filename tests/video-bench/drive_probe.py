#!/usr/bin/env python3
"""The drive branch as HQ sees it: latency, cadence and STALLS, read off the backend.

Reads `/ws/view-h264` — exactly what the drive page receives — and uses the capture time the
relay prefixes to every frame (the robot's clock when the camera produced it). Transport
agnostic on purpose: it reports the same numbers whether the robot -> HQ hop is TCP or UDP,
which is the comparison it was written for (PLAN-VIDEO.md 6.i).

Every stall is attributed. If the capture times of the two frames around a gap are also far
apart, the ROBOT did not produce frames (source); if they are the normal ~70 ms apart while
arrival jumped, the frames were produced on time and held up on the way (link). MEASURED
2026-09-23 over Starlink on TCP: 10 stalls of 250-916 ms in 150 s, all "link".

The clock offset uses the round trip with the LOWEST RTT, not a median: over Starlink single
samples ranged -8 to +1032 ms and a median once landed 430 ms off (see field_probe.py).

    python3 drive_probe.py [seconds] [backend_ws]

Needs `websockets` (the AI-VL backend's venv has it). Read-only: one more viewer of the
backend, which costs the robot link nothing — the backend fans out locally.
"""
from __future__ import annotations

import json
import ssl
import statistics as st
import struct
import sys
import time
import urllib.request

from websockets.sync.client import connect

HEALTH = "http://10.1.254.18:8093/health"
STALL_MS = 250.0


def clock_offset(samples=21):
    best = None
    for _ in range(samples):
        t1 = time.time()
        with urllib.request.urlopen(HEALTH, timeout=5) as r:
            now = float(json.loads(r.read())["now"])
        t2 = time.time()
        if best is None or t2 - t1 < best[0]:
            best = (t2 - t1, now - (t1 + t2) / 2)
    return best


def pct(xs, p):
    s = sorted(xs)
    return s[min(len(s) - 1, int(len(s) * p))]


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 150.0
    url = sys.argv[2] if len(sys.argv) > 2 else "wss://127.0.0.1:8443/ws/view-h264"
    rtt, off = clock_offset()
    print(f"clock offset (robot - us): {off * 1000:+.1f} ms  (from a {rtt * 1000:.0f} ms round trip)")
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE      # the app's own self-signed cert, on loopback
    rows, nbytes = [], 0
    with connect(url, ssl=ctx, max_size=None) as ws:
        t0 = time.time()
        while time.time() - t0 < secs:
            m = ws.recv()
            now = time.time()
            if isinstance(m, bytes) and len(m) > 8:
                rows.append((now, struct.unpack("<d", m[:8])[0]))
                nbytes += len(m) - 8
    if len(rows) < 2:
        print("no frames — is the drive branch on and the relay running?")
        return
    lat = [(t + off - c) * 1000 for t, c in rows if c > 0]
    gaps = [((b - a) * 1000, (cb - ca) * 1000, b) for (a, ca), (b, cb) in zip(rows, rows[1:])]
    print(f"\n{len(rows)} frames in {secs:.0f} s = {len(rows) / secs:.2f} fps, "
          f"{nbytes * 8 / secs / 1e6:.3f} Mbps, {nbytes / len(rows):.0f} B/frame")
    if lat:
        print(f"latency capture -> HQ : p50 {pct(lat, .5):.0f}  p95 {pct(lat, .95):.0f}  "
              f"max {max(lat):.0f} ms")
    print(f"arrival cadence      : {st.median(g for g, _, _ in gaps):.0f} ms median")
    stalls = [g for g in gaps if g[0] > STALL_MS]
    print(f"\nSTALLS over {STALL_MS:.0f} ms: {len(stalls)}")
    for gap, cgap, t in stalls:
        where = "source" if cgap > 0.6 * gap else "link"
        stamp = time.strftime("%H:%M:%S", time.localtime(t))
        print(f"  {stamp}.{int(t % 1 * 1000):03d}  gap {gap:5.0f} ms  capture-gap {cgap:5.0f} ms"
              f"  -> {where}")


if __name__ == "__main__":
    main()
