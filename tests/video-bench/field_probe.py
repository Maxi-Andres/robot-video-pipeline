#!/usr/bin/env python3
"""Field baseline for the MJPEG live branch, read straight off the robot's :8093.

Reports the three things the plan's targets are written in: latency, GAPS OVER 200 ms, and
bitrate. The gap count is the one that decides — "lo inaceptable es el congelamiento de
medio segundo" — and nothing in the repo measured it until now.

Reuses mjpeg_server's COM stamp and the SNTP-style clock offset from tests/_latency_probe.py
(the two machines are not NTP-locked to each other). Differs from that probe in what it is
for: it reads :8093 directly instead of the backend's /ws/view, because here the question is
what the FIELD LINK does to this branch, not what the backend adds on top.

    python3 field_probe.py [seconds]

Read-only. Opens ONE connection: the repo warns that several readers on :8093 inflate the
numbers with big frames.
"""
from __future__ import annotations

import statistics as st
import sys
import time
import urllib.request

MJPEG = "http://10.1.254.18:8093"
_TAG = b"AVL1 "


def read_stamp(jpeg: bytes):
    """Mirror of mjpeg_server.stamp(). Keep the two in step."""
    if not jpeg.startswith(b"\xff\xd8") or jpeg[2:4] != b"\xff\xfe":
        return None
    n = (jpeg[4] << 8) | jpeg[5]
    body = jpeg[6:4 + n]
    if not body.startswith(_TAG):
        return None
    try:
        a, b = body[len(_TAG):].split()
        return float(a), float(b)
    except (ValueError, IndexError):
        return None


def clock_offset(samples: int = 9) -> float:
    """Robot clock minus ours, in seconds. Median of SNTP-style round trips."""
    offs = []
    for _ in range(samples):
        t1 = time.time()
        with urllib.request.urlopen(f"{MJPEG}/health", timeout=5) as r:
            now = float(__import__("json").loads(r.read())["now"])
        t2 = time.time()
        offs.append(now - (t1 + t2) / 2)
    return st.median(offs)


def pct(xs, p):
    s = sorted(xs)
    return s[min(len(s) - 1, int(len(s) * p))]


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    off = clock_offset()
    print(f"clock offset (robot - us): {off * 1000:+.1f} ms")

    r = urllib.request.urlopen(f"{MJPEG}/stream", timeout=15)
    buf = b""
    lat, ivals, nbytes, n, unstamped = [], [], 0, 0, 0
    last = None
    t_end = time.time() + secs
    while time.time() < t_end:
        chunk = r.read(65536)
        if not chunk:
            break
        buf += chunk
        while True:
            s = buf.find(b"\xff\xd8")
            if s < 0:
                break
            e = buf.find(b"\xff\xd9", s + 2)
            if e < 0:
                break
            frame = buf[s:e + 2]
            buf = buf[e + 2:]
            now = time.time()
            n += 1
            nbytes += len(frame)
            if last is not None:
                ivals.append((now - last) * 1000)
            last = now
            stamp = read_stamp(frame)
            if stamp is None:
                unstamped += 1
            else:
                t_in, _t_out = stamp
                # t_in is on the robot's clock; correct it onto ours before subtracting.
                lat.append((now - (t_in - off)) * 1000)
    r.close()

    dur = secs
    print(f"\nMJPEG branch, {n} frames in {dur:.0f} s")
    print(f"  rate    : {n/dur:.2f} fps")
    print(f"  bitrate : {nbytes/dur/1024:.0f} kB/s  ({nbytes*8/dur/1e6:.2f} Mbps)")
    print(f"  frame   : {nbytes/max(n,1)/1024:.0f} KB mean")
    if lat:
        print(f"  latency : p50 {pct(lat,.5):.0f}  p95 {pct(lat,.95):.0f}  "
              f"max {max(lat):.0f}  min {min(lat):.0f} ms   (capture -> here)")
    else:
        print(f"  latency : not measurable — {unstamped} frames carried no COM stamp "
              f"(set STAMP=1 in video.env)")
    if ivals:
        med = st.median(ivals)
        # An absolute >200 ms threshold is meaningless when the branch is CAPPED at 5 fps:
        # 200 ms is then the nominal spacing, so every frame would count as a gap. What a
        # freeze actually looks like is a frame arriving late RELATIVE to the cadence, so
        # report both and let the cadence be visible.
        stalls = [g for g in ivals if g > 2 * med]
        print(f"  cadence : {med:.0f} ms median between frames "
              f"({1000/med:.1f} fps nominal)")
        print(f"  gaps >200ms      : {sum(1 for g in ivals if g > 200)}  "
              f"(meaningless if the cadence above is near 200)")
        flag = "" if not stalls else "   <-- the freeze the plan is about"
        print(f"  STALLS >2x cadence: {len(stalls)}  worst {max(ivals):.0f} ms{flag}")


if __name__ == "__main__":
    main()
