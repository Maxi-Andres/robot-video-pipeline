#!/usr/bin/env python3
"""control_to_photon.py — the latency the operator actually feels, with no clock to trust.

THE QUESTION. Press a key, how long until the video shows the robot moving? That is the
number that decides whether this thing can be driven, and it is the one still missing: every
attempt so far tried to read an absolute timestamp off something, and every one failed.

WHY THIS ONE CANNOT FAIL THE SAME WAY. Both ends of the interval are OUR OWN monotonic clock
on THIS machine:

    T   = the instant we hand the move to the relay
    T'  = the arrival time, here, of the first video frame that shows motion
    T'-T = control-to-photon

Nothing is read off a screen, nothing is compared against the robot's clock, nothing needs
NTP. The previously recorded attempts are all in PLAN-VIDEO.md §9; the short version is that
a stopwatch filmed off a laptop gave -710 ms and then -1400 ms — negative, therefore
impossible — because that laptop's clock is neither ours nor stable. The robot's own clock is
no better: its journal jumps from "Jan 07" to the real date once NTP lands after boot.

WHAT IT MEASURED, 2026-09-14, AND THE LIMIT THAT FOUND (read before re-running):

    onset over 6 pulses: 311 / 640 / 971 / 1181 / 1518 / 1855 ms   median 1076, stdev 516

The MEDIAN is the answer to "what does the operator feel": about a second. The SPREAD is the
more important result. 516 ms of scatter is the same order as the number being chased, and it
is NOT the video path: frames arrive here every 70.5 ms with a stdev of 5.4 ms, zero bursts
and zero gaps over 200 ms, measured over 45 s (--check-video does this). A transport that
regular cannot scatter an onset by half a second.

So the scatter is the ROBOT's own delay between accepting a move and visibly starting one.
One pulse even onset at 1855 ms, i.e. AFTER the 1500 ms dead-man had already stopped it.

    => This method answers "can it be driven". It CANNOT isolate video latency, and no number
       of extra trials fixes that: the robot's start delay is a BIAS plus large variance, not
       noise that averages away. For the video number use PLAN-VIDEO.md §9 option 2 (a screen
       we serve, flashing on our command — no mechanics between the command and the photons)
       or option 3 (RTCP sender reports).

WHAT THE NUMBER INCLUDES, stated plainly because it is not the whole story:

    control-to-photon = command path (~7 ms, measured: see --probe-command)
                      + the robot's own mechanical response to a move
                      + capture, encode, link, mediamtx, and our decode

The middle term is real and is NOT video latency. So this is an UPPER BOUND on the video
path, and simultaneously the RIGHT number for "can I drive it". Both readings are useful;
neither should be quoted as the other.

It also stops at mediamtx+RTSP on this machine. A browser adds its WebRTC hop and jitter
buffer on top — ~70 ms and ~54 ms measured in isolation (PLAN-VIDEO.md §1).

WHERE IT RUNS. It needs OpenCV to decode H.264, which on this workstation exists only inside
the unitree_ros2 devcontainer. That container does not mount this repo, so pipe the script in
rather than copying it somewhere it will rot:

    docker exec -i -e RELAY_TOKEN="$(cat /path/to/token)" \
        unitree_ros2_devcontainer-devcontainer-humble-1 \
        python3 - --trials 10 < control_to_photon.py

THE NEGATIVE CONTROL IS THE DEFAULT. With no --confirm-move the run sends `keepalive`, which
can never initiate motion, and asserts that the detector finds NOTHING. A detector that fires
on a robot that did not move would manufacture any latency you like, so it is made to prove
it stays quiet BEFORE it is allowed to report a number.
"""
from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import threading
import time
import urllib.request

import cv2
import numpy as np

RTSP_DEFAULT = "rtsp://127.0.0.1:8554/robot"
RELAY_DEFAULT = "http://10.1.254.18:8092"
_FFMPEG_OPTS = "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|stimeout;5000000"
# Frames are reduced to this height before differencing. Small is better: it averages out
# sensor noise and JPEG/H.264 mosquito noise, and a whole-frame shift (which is what the
# robot's own camera sees when the robot turns) survives any downscale.
_THUMB_H = 64


class Capture:
    """Reads the stream continuously and timestamps each frame ON ARRIVAL HERE.

    Runs in its own thread and never blocks on the consumer, because a reader that falls
    behind its source accumulates latency without bound and would silently inflate every
    number this script prints. That the OpenCV reader keeps up on this path is measured, not
    assumed: unitree_ros2/robot_camera_bridge/tests/reader_bench.py, +0.0 ms/s over 180 s.
    """

    def __init__(self, url):
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = _FFMPEG_OPTS
        self._cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if not self._cap.isOpened():
            raise SystemExit(f"could not open {url}")
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._frames: list[tuple[float, np.ndarray]] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            ok, bgr = self._cap.read()
            t = time.monotonic()          # AFTER the read returns: when the frame got here
            if not ok:
                self._stop.set()
                return
            g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            h = _THUMB_H
            w = max(1, int(g.shape[1] * h / g.shape[0]))
            thumb = cv2.resize(g, (w, h), interpolation=cv2.INTER_AREA).astype(np.float32)
            with self._lock:
                self._frames.append((t, thumb))
                # Bounded on purpose. An unbounded list is the same defect as an unbounded
                # frame buffer, just slower to bite.
                if len(self._frames) > 4000:
                    del self._frames[:2000]

    def since(self, t0):
        with self._lock:
            return [(t, f) for t, f in self._frames if t >= t0]

    def wait_for_frames(self, n, timeout=20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if len(self._frames) >= n:
                    return True
            time.sleep(0.05)
        return False

    def close(self):
        self._stop.set()
        self._thread.join(timeout=3.0)
        self._cap.release()


def diffs(frames):
    """Mean absolute difference between consecutive thumbnails, with the LATER timestamp.

    The later one is the right label: the change is first VISIBLE in that frame, so that is
    the frame whose arrival time answers "when could the operator have seen it".
    """
    out = []
    for (_, a), (tb, b) in zip(frames, frames[1:], strict=False):
        out.append((tb, float(np.mean(np.abs(b - a)))))
    return out


def onset(series, baseline, k):
    """First sample exceeding the baseline by k robust sigmas. Returns (t, value) or None.

    Robust because the baseline is a real scene, not a test pattern: one person walking
    through it would drag a mean-and-stdev threshold far enough to miss the real onset.
    MAD * 1.4826 is the stdev of a normal distribution, so k keeps its usual meaning.
    """
    med, mad = baseline
    sigma = max(mad * 1.4826, 1e-6)
    for t, v in series:
        if v > med + k * sigma:
            return t, v, (v - med) / sigma
    return None


def robust_baseline(values):
    med = float(np.median(values))
    mad = float(np.median(np.abs(np.asarray(values) - med)))
    return med, mad


class Relay:
    def __init__(self, base, token):
        self._base, self._token = base.rstrip("/"), token

    def _post(self, body):
        data = json.dumps(body).encode()
        req = urllib.request.Request(
            f"{self._base}/cmd", data=data,
            headers={"Authorization": f"Bearer {self._token}",
                     "Content-Type": "application/json"})
        return json.loads(urllib.request.urlopen(req, timeout=10).read())

    def health(self):
        with urllib.request.urlopen(f"{self._base}/health", timeout=10) as r:
            return json.loads(r.read())

    def move(self, vx, vy, vyaw):
        return self._post({"verb": "move", "vx": vx, "vy": vy, "vyaw": vyaw})

    def stop(self):
        return self._post({"verb": "stop_move"})

    def keepalive(self):
        return self._post({"verb": "keepalive"})


def learn_baseline(cap, seconds):
    """What "quiet" looks like, learned ONCE while the robot is known to be still.

    Deliberately not re-learned per pulse. The first version did that, and it was wrong: a
    baseline sampled just after a pulse is contaminated by the robot still settling, which
    inflates the threshold and reports the NEXT onset late. Measured 2026-09-14, that alone
    moved an onset from 679 ms to 1229 ms — a 550 ms error, invented entirely by the
    instrument, in a measurement whose whole point is a few hundred milliseconds.

    The cost of learning it once is that a slow lighting change eventually drifts the scene
    away from it. That is visible and loud (every pulse reads a huge sigma count, or none
    read at all), whereas the contaminated baseline was silent and plausible.
    """
    t0 = time.monotonic()
    time.sleep(seconds)
    frames = cap.since(t0)
    if len(frames) < 30:
        raise SystemExit("not enough video to learn a baseline")
    return robust_baseline([v for _, v in diffs(frames)])


def one_trial(cap, relay, args, moving, baseline):
    """One pulse. Returns a dict, or None if there was not enough video."""
    # Fire. T is taken as late as possible before the request leaves.
    t_cmd = time.monotonic()
    reply = relay.move(args.vx, args.vy, args.vyaw) if moving else relay.keepalive()
    t_replied = time.monotonic()

    # Watch, then let the robot come fully to rest before the caller fires again. Visible
    # motion outlasts the command by a lot: a single move, stopped by the 1500 ms dead-man,
    # produced 6.3 s of movement on camera (measured, and remarkably repeatable: 6.30 /
    # 6.30 / 6.30 / 6.29 / 6.31 s). Pulsing faster than that measures the previous pulse.
    time.sleep(args.watch)
    if moving:
        relay.stop()
    series = [(t, v) for t, v in diffs(cap.since(t_cmd - 0.2)) if t >= t_cmd]
    hit = onset(series, baseline, args.k)
    time.sleep(args.settle)

    return {
        "t_cmd": t_cmd,
        "rtt_ms": (t_replied - t_cmd) * 1000.0,
        "reply": reply,
        "baseline_med": baseline[0],
        "baseline_mad": baseline[1],
        "n_frames": len(series),
        "latency_ms": (hit[0] - t_cmd) * 1000.0 if hit else None,
        "sigmas": hit[2] if hit else None,
        "peak": max((v for _, v in series), default=0.0),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default=RTSP_DEFAULT)
    ap.add_argument("--relay", default=RELAY_DEFAULT)
    ap.add_argument("--token-file", default="",
                    help="relay bearer token; or set RELAY_TOKEN, which is what the relay "
                         "itself documents and what lets this run piped into a container")
    ap.add_argument("--trials", type=int, default=10)
    ap.add_argument("--vx", type=float, default=0.0)
    ap.add_argument("--vy", type=float, default=0.0)
    ap.add_argument("--vyaw", type=float, default=0.5,
                    help="default is yaw only: the robot turns IN PLACE, so it cannot walk "
                         "into anything, and a rotation moves every pixel in frame")
    ap.add_argument("--watch", type=float, default=1.2,
                    help="seconds of video to search after the command (< the 1500 ms "
                         "dead-man, so an unanswered stop still ends the motion)")
    ap.add_argument("--settle", type=float, default=10.0,
                    help="rest between pulses. Default 10 s because a single move, stopped "
                         "by the dead-man at 1500 ms, still produced 6.3 s of visible "
                         "movement — pulse faster and you measure the previous pulse")
    ap.add_argument("--baseline", type=float, default=12.0,
                    help="seconds of guaranteed-still video used to learn the threshold, once")
    ap.add_argument("--k", type=float, default=8.0, help="detection threshold, robust sigmas")
    ap.add_argument("--confirm-move", action="store_true",
                    help="ACTUALLY MOVE THE ROBOT. Without it this is a negative control: it "
                         "sends keepalive and checks that the detector reports nothing.")
    ap.add_argument("--probe-command", action="store_true",
                    help="only measure the relay round-trip, move nothing, and exit")
    ap.add_argument("--check-video", type=float, default=0.0, metavar="SECONDS",
                    help="only measure how REGULARLY frames arrive, move nothing, and exit. "
                         "This is what decides whether a scattered onset can be blamed on "
                         "the video path")
    args = ap.parse_args()

    token = os.environ.get("RELAY_TOKEN", "").strip()
    if not token and args.token_file:
        with open(args.token_file) as fh:
            token = fh.read().strip()
    if not token:
        raise SystemExit("no relay token: set RELAY_TOKEN or pass --token-file")
    relay = Relay(args.relay, token)

    health = relay.health()
    if not health.get("sender_alive"):
        # Worth saying loudly: /health reports ok:true in this state, and the sender is only
        # respawned by the NEXT command — so the first trial would silently pay the spawn
        # cost and read high. Warm it here instead of measuring it.
        print("sender was not alive; warming it with a keepalive", file=sys.stderr)
        relay.keepalive()
        if not relay.health().get("sender_alive"):
            raise SystemExit("command_sender will not start — cannot measure")

    if args.probe_command:
        ts = []
        for _ in range(20):
            t = time.monotonic()
            relay.keepalive()
            ts.append((time.monotonic() - t) * 1000.0)
            time.sleep(0.15)
        print(f"relay round-trip: median {statistics.median(ts):.1f} ms  "
              f"min {min(ts):.1f}  max {max(ts):.1f}  (n={len(ts)})")
        return 0

    if args.check_video:
        cap = Capture(args.url)
        try:
            if not cap.wait_for_frames(10):
                raise SystemExit("no video arriving")
            t0 = time.monotonic()
            time.sleep(args.check_video)
            ts = [t for t, _ in cap.since(t0)]
        finally:
            cap.close()
        gaps = [(b - a) * 1000.0 for a, b in zip(ts, ts[1:], strict=False)]
        if len(gaps) < 10:
            raise SystemExit("not enough frames to judge regularity")
        gaps.sort()
        print(f"{len(ts)} frames in {args.check_video:.0f}s = {len(ts)/args.check_video:.2f} fps")
        print(f"  arrival spacing: median {statistics.median(gaps):.1f} ms  "
              f"stdev {statistics.pstdev(gaps):.1f}  max {gaps[-1]:.1f}")
        print(f"  gaps over 200 ms: {sum(1 for g in gaps if g > 200)}")
        print(f"  arrivals under 5 ms (a burst): {sum(1 for g in gaps if g < 5)}")
        print("\nRegular spacing with no bursts means the transport cannot be what scatters")
        print("an onset. Measured 2026-09-14: median 70.5 ms, stdev 5.4, 0 gaps, 0 bursts.")
        return 0

    moving = args.confirm_move
    print(f"{'MOVING THE ROBOT' if moving else 'NEGATIVE CONTROL (no motion commanded)'}: "
          f"{args.trials} trials, vx={args.vx} vy={args.vy} vyaw={args.vyaw}\n")

    cap = Capture(args.url)
    try:
        if not cap.wait_for_frames(20):
            raise SystemExit("no video arriving — check the stream before measuring")
        print(f"learning the quiet baseline for {args.baseline:.0f}s — "
              f"the robot must not move now")
        baseline = learn_baseline(cap, args.baseline)
        print(f"  baseline {baseline[0]:.3f} +/- {baseline[1]:.3f} "
              f"(threshold at k={args.k})\n")
        rows = []
        for i in range(args.trials):
            r = one_trial(cap, relay, args, moving, baseline)
            if r is None:
                print(f"  trial {i+1:2d}: skipped, not enough frames for a baseline")
                continue
            rows.append(r)
            lat = (f"{r['latency_ms']:7.0f} ms  ({r['sigmas']:.1f}s)"
                   if r["latency_ms"] is not None else "      no motion detected")
            print(f"  trial {i+1:2d}: {lat}   rtt {r['rtt_ms']:5.1f} ms   "
                  f"base {r['baseline_med']:.2f}±{r['baseline_mad']:.2f}  peak {r['peak']:.2f}")
    finally:
        if moving:
            try:
                relay.stop()
            except Exception:
                pass
        cap.close()

    hits = [r["latency_ms"] for r in rows if r["latency_ms"] is not None]
    print()
    if not moving:
        if hits:
            print(f"NEGATIVE CONTROL FAILED: {len(hits)}/{len(rows)} trials 'detected' motion "
                  f"on a robot that was never commanded to move. The threshold (k={args.k}) is "
                  f"too low for this scene, or something else in frame is moving. Do not "
                  f"trust a latency from this setup until this run comes back clean.")
            return 1
        print(f"negative control clean: 0/{len(rows)} false detections at k={args.k}.")
        print("Re-run with --confirm-move, with someone watching the robot, for the number.")
        return 0

    if len(hits) < 3:
        print(f"only {len(hits)}/{len(rows)} trials detected motion — not enough to report.")
        return 1
    # The video is quantised at ~14 fps, so a single trial carries ~70 ms of rounding. The
    # MEDIAN over trials is what to quote; the spread says how much of it is quantisation.
    med, sd = statistics.median(hits), statistics.pstdev(hits)
    print(f"control-to-photon over {len(hits)} trials:")
    print(f"  median {med:6.0f} ms")
    print(f"  mean   {statistics.mean(hits):6.0f} ms   stdev {sd:5.0f} ms")
    print(f"  min    {min(hits):6.0f} ms   max {max(hits):6.0f} ms")
    if sd > med * 0.25:
        print(f"\n  !! the spread ({sd:.0f} ms) is {sd / med * 100:.0f}% of the median. That is")
        print("     the ROBOT's own delay in starting a move, not the video path — run")
        print("     --check-video and you will find frame arrivals regular to a few ms.")
        print("     Quote the median as 'what the operator feels'. Do NOT quote it as video")
        print("     latency, and do not add trials hoping it tightens: a mechanical start")
        print("     delay is a bias with variance, not noise that averages away.")
    print(f"  relay round-trip, median {statistics.median([r['rtt_ms'] for r in rows]):.1f} ms "
          f"— subtract at most this for the non-video part")
    print("\nIncludes the robot's own mechanical response, so it is an UPPER BOUND on the")
    print("video path and the RIGHT number for 'can it be driven'. A browser adds its")
    print("WebRTC hop and jitter buffer on top (~70 ms and ~54 ms, PLAN-VIDEO.md §1).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
