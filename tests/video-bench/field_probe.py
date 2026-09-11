#!/usr/bin/env python3
"""Field baseline for the MJPEG live branch, read straight off the robot's :8093.

Reports what the plan's targets are written in — latency, stalls and bitrate — and then
SPLITS the stall by stage, which is the part that decides where to look next.

mjpeg_server already stamps two robot-side times into every frame and nothing has ever read
the difference between them:

    t_in           pump() pulled the frame off go2_jpeg_stream's stdout  -> the VIDEOHUB's
                   own cadence, since GetImageSample has already returned by then
    t_out - t_in   what this process costs (a resize, if MJPEG_WIDTH > 0)
    now  - t_out   everything after the robot: link, and our own read

So a stall seen here is attributable: if consecutive t_in values also jump, the source
stalled and no transport will fix it; if they do not, it happened downstream. MEASURED
2026-09-11 by cable, both branches froze with packetsLost=0, which is what makes this split
the next question rather than a curiosity.

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
    frames = []          # (arrival, t_in, t_out) — t_in/t_out on the ROBOT's clock
    nbytes, n, unstamped = 0, 0, 0
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
            stamp = read_stamp(frame)
            if stamp is None:
                unstamped += 1
            else:
                frames.append((now, stamp[0], stamp[1]))
    r.close()

    print(f"\nMJPEG branch, {n} frames in {secs:.0f} s")
    print(f"  rate    : {n/secs:.2f} fps")
    print(f"  bitrate : {nbytes/secs/1024:.0f} kB/s  ({nbytes*8/secs/1e6:.2f} Mbps) per viewer")
    print(f"  frame   : {nbytes/max(n,1)/1024:.0f} KB mean")
    if not frames:
        print(f"  no stamped frames ({unstamped} unstamped) — set STAMP=1 in video.env")
        return

    def deltas(xs):
        return [(b - a) * 1000 for a, b in zip(xs, xs[1:])]

    arrival = deltas([f[0] for f in frames])
    source = deltas([f[1] for f in frames])
    inside = [(f[2] - f[1]) * 1000 for f in frames]
    transport = [(f[0] - (f[2] - off)) * 1000 for f in frames]

    def stalls(ds):
        """Intervals over twice the median. An absolute threshold is useless here: a branch
        capped at 5 fps has a 200 ms nominal spacing, so every frame would count."""
        med = st.median(ds)
        return med, [i for i, d in enumerate(ds) if d > 2 * med]

    med_a, st_a = stalls(arrival)
    med_s, st_s = stalls(source)
    # A viewer stall is "explained" when the very interval it sits on also stalled at source.
    explained = len(set(st_a) & set(st_s))

    print(f"\n  per-stage, {len(frames)} stamped frames")
    print(f"    source cadence (t_in)      : {med_s:.0f} ms median, {1000/med_s:.1f} fps")
    print(f"    mjpeg_server cost (t_out-t_in): p50 {pct(inside,.5):.1f}  max {max(inside):.1f} ms")
    print(f"    transport (t_out -> here)  : p50 {pct(transport,.5):.0f}  p95 {pct(transport,.95):.0f}"
          f"  max {max(transport):.0f} ms")
    print(f"    viewer cadence (arrival)   : {med_a:.0f} ms median")

    print("\n  STALLS (interval over 2x its own median)")
    print(f"    at the source (videohub)   : {len(st_s)}  worst {max(source):.0f} ms")
    print(f"    at the viewer (here)       : {len(st_a)}  worst {max(arrival):.0f} ms")
    if st_a:
        print(f"    of the {len(st_a)} viewer stalls, {explained} coincide with a source stall "
              f"({100*explained/len(st_a):.0f}%)")
        if explained >= 0.8 * len(st_a):
            print("    -> the SOURCE stalls. No transport change can fix this.")
        elif explained <= 0.2 * len(st_a):
            print("    -> stalls appear DOWNSTREAM of mjpeg_server: link or reader.")
        else:
            print("    -> mixed: both the source and something downstream contribute.")


if __name__ == "__main__":
    main()
