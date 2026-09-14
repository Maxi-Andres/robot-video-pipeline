#!/usr/bin/env python3
"""Summarise the bench's report.jsonl.

GROUPS BY SESSION, and that is not a nicety: every open tab measures its own
RTCPeerConnection and posts to the same file, so a forgotten tab from an earlier run mixes
its windows into the aggregate. It shows up as a NEGATIVE bitrate — bytesReceived appearing
to go backwards — which is exactly how it was found. Pass --all to print every session;
by default only the newest one is summarised, which is the run you just did.

    python3 show.py < report.jsonl
    python3 show.py --all < report.jsonl
"""
import json
import sys


def summarise(sid, rows):
    first, last = rows[0], rows[-1]
    rf, rl = first.get("rtc") or {}, last.get("rtc") or {}
    frames = sum(r["branches"][1]["n"] for r in rows)
    worst = max(r["branches"][1]["max_gap_ms"] for r in rows)
    gaps = sum(r["branches"][1]["gaps_over_200ms"] for r in rows)
    print(f"session {sid}: {len(rows)} windows")
    if len(rows) < 2 or rf.get("at") is None:
        print("  too short to report a rate\n")
        return
    dur = (rl["at"] - rf["at"]) / 1000
    if dur <= 0:
        print("  windows out of order\n")
        return
    dec = rl.get("framesDecoded", 0) - rf.get("framesDecoded", 0)
    print(f"  span          : {dur:.0f} s, {dec} frames decoded ({dec/dur:.2f} fps)")
    if frames:
        print(f"  presented     : {frames} ({frames/dur:.2f} fps as the compositor showed them)")
    if rl.get("bytesReceived") is not None and rf.get("bytesReceived") is not None:
        mbps = (rl["bytesReceived"] - rf["bytesReceived"]) * 8 / dur / 1e6
        flag = "   <-- NEGATIVE: sessions are mixed, this is not one connection" if mbps < 0 else ""
        print(f"  bitrate       : {mbps:.2f} Mbps{flag}")
    print(f"  freezes       : {rl.get('freezeCount')} ({rl.get('totalFreezesDuration')} s cumulative)")
    print(f"  packets lost  : {rl.get('packetsLost')} / {rl.get('packetsReceived')}")
    print(f"  jitter buffer : {rl.get('jitterBufferMs')} ms")
    hidden = sum(r.get("hidden_ms", 0) for r in rows)
    note = "" if hidden < 500 else f"   <-- tab was hidden {hidden/1000:.0f}s; gaps under-counted"
    print(f"  worst gap     : {worst} ms   (gaps>200ms {gaps}, meaningless below ~5 fps){note}")
    print("  NOTE: when freezes and worst-gap disagree, believe freezes — it comes from the")
    print("        decoder, the gap counter only sees frames the compositor presented.")
    print(f"  resolution    : {rl.get('frameWidth')}x{rl.get('frameHeight')}\n")


def main():
    show_all = "--all" in sys.argv
    sessions = {}
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        d = json.loads(line)
        # framesDecoded, not the rVFC frame count: a BACKGROUNDED tab stops firing
        # requestVideoFrameCallback entirely, so branches[1].n goes to zero and a whole run
        # looks like it never happened — which is exactly what it did once. getStats keeps
        # counting in the decoder whether or not anything is painted.
        if not (d.get("rtc") or {}).get("framesDecoded"):
            continue                      # a tab that never connected
        sessions.setdefault(d.get("sid", "unknown"), []).append(d)
    if not sessions:
        print("no windows with WebRTC frames")
        return
    if len(sessions) > 1 and not show_all:
        print(f"note: {len(sessions)} sessions in this file "
              f"({', '.join(sessions)}) — showing the newest only, pass --all for the rest\n")
    order = sorted(sessions.items(), key=lambda kv: kv[1][-1]["at"])
    for sid, rows in (order if show_all else order[-1:]):
        summarise(sid, rows)


if __name__ == "__main__":
    main()
