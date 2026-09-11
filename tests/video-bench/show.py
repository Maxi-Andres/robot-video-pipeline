#!/usr/bin/env python3
"""Pretty-print the bench's report.jsonl windows. Skips windows with no WebRTC frames —
those come from a stale tab that was opened before the page auto-started."""
import json
import sys


def row(tag, s):
    lat = (f"p50={s['p50']:>5} p95={s['p95']:>5} max={s['max']:>5} ms   "
           if s.get("nlat") else "latency: no barcode          ")
    return (f"{tag:7s} n={s['n']:4d} {s['fps']:6.2f} fps   " + lat +
            f"gaps>200ms={s['gaps_over_200ms']} (worst {s['max_gap_ms']})")


PREV = {}


for line in sys.stdin:
    line = line.strip()
    if not line:
        continue
    d = json.loads(line)
    a, b = d["branches"]
    if not b["n"]:
        continue
    r = d.get("rtc") or {}
    print(d["at"])
    print("  " + row("MJPEG", a))
    print("  " + row("WebRTC", b))
    kbps = ""
    if r.get("bytesReceived") is not None and PREV.get("b") is not None:
        db = r["bytesReceived"] - PREV["b"]
        dt = (r["at"] - PREV["t"]) / 1000.0
        if dt > 0:
            kbps = f"   bitrate={db*8/dt/1000:.0f} kbps"
    if r.get("bytesReceived") is not None:
        PREV["b"], PREV["t"] = r["bytesReceived"], r["at"]
    print(f"     rtc: jitterBuffer={r.get('jitterBufferMs')} ms   "
          f"freezes={r.get('freezeCount')} ({r.get('totalFreezesDuration')} s)   "
          f"lost={r.get('packetsLost')}/{r.get('packetsReceived')}   "
          f"rtt={r.get('rttMs')} ms   {r.get('frameWidth')}x{r.get('frameHeight')}" + kbps)
    print()
