#!/usr/bin/env python3
"""latency_clock.py — glass-to-glass latency by filming a clock we serve ourselves.

THE PROBLEM WITH EVERY PREVIOUS ATTEMPT was never the idea, it was the clock. Filming a
stopwatch and comparing it against the robot's clock gave -710 ms and then -1400 ms: negative,
therefore impossible, because that laptop's clock is neither ours nor stable. The robot's own
clock is no better — its journal jumps from "Jan 07" to the real date once NTP lands at boot.

THE FIX IS OWNING BOTH ENDS. This process serves the page AND reads the video, so "when it was
shown" and "when it arrived" are the same clock. When the page runs on another screen (a phone
or laptop next to the robot) it disciplines itself to ours over HTTP, NTP style, and shows the
residual on screen so a bad sync is visible instead of silently becoming "latency".

TWO CHANNELS ON ONE PAGE, because they fail differently:

  1. A HUGE high-contrast clock — white on black, bold, monospace. This is what a human reads,
     and what works in a single photo of screen-plus-video-window. It needs nothing from this
     script to be useful.

  2. A PANEL THAT MODULATES between black and mid-grey on a schedule this server hands out.
     This is the channel that gets measured automatically, and it exists because reading
     DIGITS off a FILMED screen does not work: the screen arrives rotated, out of focus and at
     an unknown scale, and the millisecond digits smear into an unreadable blur. That is
     measured, and it is why the stopwatch method was abandoned for this robot. Mean
     brightness survives all of it — no locating, no perspective correction, no OCR.

  Deliberately NOT a white flash. A bright screen in a dim room blows out the camera's
  auto-exposure, which was the OTHER half of why the stopwatch failed. Black against mid-grey
  moves the mean enough to correlate and saturates nothing.

WHY A PSEUDORANDOM SCHEDULE AND NOT A BLINK. A square wave correlates with itself at every
period, so it cannot distinguish a latency of L from L plus one period. A pseudorandom
sequence has a single sharp autocorrelation peak. That peak IS the latency.

ONE IMPLEMENTATION OF THE SCHEDULE, ON PURPOSE. The page could generate it too, but then the
same PRNG would exist twice, in two languages, with 32-bit arithmetic that differs between
them — and a drift there would not crash, it would quietly produce a wrong latency. So the
server ships the bits and the page only indexes into them.

USAGE

  # 1. serve; open http://<this-host>:8099/ on a screen the robot's camera can see
  python3 latency_clock.py serve --port 8100

  # 2. with that on camera, measure one or more paths in the same run
  python3 latency_clock.py measure --seconds 60 \\
      --source h264=rtsp://127.0.0.1:8554/robot \\
      --source mjpeg=http://10.1.254.18:8093/stream

MJPEG vs H.264 IN ONE RUN. Both sources film the SAME panel at the SAME instants, so the
DIFFERENCE between their numbers is what the two transports cost relative to each other — and
that difference survives even if the absolute number is off, because everything they share
(the screen, the optics, the sensor, the robot's own pipeline up to the split) cancels.

  NOTE: the robot only serves MJPEG while it runs SOURCE=jpeg. Under SOURCE=multicast its
  mjpeg_server is off and there is nothing on :8093 to compare against.
"""
from __future__ import annotations

import argparse
import functools
import hashlib
import http.server
import json
import socketserver
import sys
import threading
import time
import urllib.parse
import urllib.request

import numpy as np

# A slot must span more than one video frame or the panel aliases: at 14.25 fps a frame is
# 70 ms, so 150 ms is ~2 frames per slot and still packs 400 slots into a 60 s run.
SLOT_MS = 150
LEVEL_LO, LEVEL_HI = 0, 150      # black against mid-grey; see the docstring on saturation
SEED = b"aivl-latency-clock-v1"

# What the page last told us about its own clock discipline. The measurement refuses to quote
# a number without it: a page that never synced silently measures ITS clock, not ours, and
# that is precisely how this question got answered wrong twice before.
REPORT: dict = {}
REPORT_LOCK = threading.Lock()


@functools.lru_cache(maxsize=1 << 20)
def slot_bit(slot: int) -> int:
    """The panel's state for one slot. A pure function of the slot index, so the reader can
    regenerate the entire transmitted sequence after the fact without the page reporting
    anything, and two runs weeks apart are directly comparable.

    Cached because the correlation evaluates it once per sample per candidate lag — with an
    800-lag sweep over a 60 s capture that is hundreds of thousands of calls, and the slots
    repeat heavily across lags."""
    return hashlib.sha256(SEED + b":" + str(slot).encode()).digest()[0] & 1


def schedule(first_slot: int, count: int) -> list[int]:
    return [slot_bit(first_slot + i) for i in range(count)]


# --------------------------------------------------------------------------------- serving

PAGE = r"""<!doctype html>
<meta charset="utf-8"><title>latency clock</title>
<style>
  html,body{margin:0;height:100%;background:#000;color:#fff;overflow:hidden}
  body{display:flex;flex-direction:column;font-family:"DejaVu Sans Mono","Courier New",monospace}
  #clock{flex:0 0 auto;text-align:center;font-size:20vw;font-weight:700;line-height:1;
         letter-spacing:-0.03em;padding-top:2vh}
  #panel{flex:1 1 auto;margin:2vh 3vw 8vh;background:#000}
  /* Big on purpose. At 15px this came back off the camera as an unreadable green smear —
     tried it — and the sync is the one thing that must be verifiable from the picture. */
  #sync{position:fixed;left:2vw;bottom:1vh;font-size:4vw;font-weight:700;color:#0f0}
  #warn{position:fixed;right:2vw;bottom:1vh;font-size:4vw;font-weight:700;color:#f44}
</style>
<div id="clock">--.---</div>
<div id="panel"></div>
<div id="sync">sincronizando...</div>
<div id="warn"></div>
<script>
const SLOT_MS = __SLOT_MS__, HI = "rgb(__HI__,__HI__,__HI__)", LO = "#000";

// Offset from this browser's clock to the SERVER's, NTP style: assume the request and the
// reply take the same time, so the server's instant sits at the midpoint of our send and
// receive. Keep the sample with the LOWEST round trip rather than the last or the mean — a
// single sample carries whatever that one request's scheduling jitter was, and the minimum is
// the one least contaminated by it.
let offset = 0, bestRtt = Infinity;
async function syncOnce() {
  const t0 = performance.timeOrigin + performance.now();
  const r = await fetch("/time", {cache: "no-store"});
  const t1 = performance.timeOrigin + performance.now();
  const server = (await r.json()).epoch_ms;
  const rtt = t1 - t0;
  if (rtt < bestRtt) { bestRtt = rtt; offset = server + rtt / 2 - t1; }
}
async function sync() {
  for (let i = 0; i < 12; i++) {
    try { await syncOnce(); } catch (e) {}
    await new Promise(r => setTimeout(r, 120));
  }
  // "SYNC OK" survives a camera far better than a number does: the reader only has to
  // distinguish two words, not read three digits through motion blur.
  document.getElementById("sync").textContent =
      (bestRtt <= 50 ? "SYNC OK " : "SYNC ? ") + bestRtt.toFixed(0) + "ms";
  // A sync error the size of a video frame is indistinguishable from real latency, so it is
  // shown loudly rather than folded silently into the answer.
  document.getElementById("warn").textContent =
      bestRtt > 50 ? "RTT " + bestRtt.toFixed(0) + " ms - acerca la pagina o usa cable" : "";
  // Report it back. The green line on screen is unreadable once it has been through a camera
  // -- tried, it comes back as a smear -- and an unverified sync is exactly the failure that
  // produced -710 ms and -1400 ms before. So the page states its own sync over HTTP and the
  // reader refuses to quote a latency without it.
  try {
    await fetch("/report", {method: "POST", headers: {"Content-Type": "application/json"},
      body: JSON.stringify({offset: offset, rtt: bestRtt, ua: navigator.userAgent,
                            screen: screen.width + "x" + screen.height})});
  } catch (e) {}
}
sync(); setInterval(sync, 30000);

// The schedule comes from the server; this page never generates bits. Fetched well ahead and
// topped up, so a network hiccup cannot leave the panel guessing.
let bits = null, baseSlot = 0;
async function loadSchedule() {
  const now = Date.now() + offset;
  const from = Math.floor(now / SLOT_MS) - 20;
  const r = await fetch("/schedule?from=" + from + "&count=8000", {cache: "no-store"});
  const j = await r.json();
  baseSlot = j.first_slot; bits = j.bits;
}
loadSchedule(); setInterval(loadSchedule, 15 * 60 * 1000);

const clock = document.getElementById("clock"), panel = document.getElementById("panel");
let lastSlot = null;
function frame() {
  const now = Date.now() + offset;                 // the server's clock
  const ms = Math.floor(now) % 60000;
  clock.textContent = String(Math.floor(ms / 1000)).padStart(2, "0") + "." +
                      String(ms % 1000).padStart(3, "0");
  const slot = Math.floor(now / SLOT_MS);
  if (slot !== lastSlot && bits) {
    lastSlot = slot;
    const i = slot - baseSlot;
    if (i >= 0 && i < bits.length) panel.style.background = bits[i] ? HI : LO;
  }
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
</script>
"""


# A single screenshot of this answers "does the browser see something fresher than our
# reader": the live clock and the video are in the SAME image, so there is no question of when
# the shot was taken. The video shows the filmed tablet, whose clock is the same server's.
COMPARE = r"""<!doctype html>
<meta charset="utf-8"><title>compare</title>
<style>
  html,body{margin:0;height:100%;background:#000;color:#fff;
            font-family:"DejaVu Sans Mono",monospace}
  .wrap{display:flex;height:100%;align-items:stretch}
  .half{flex:1 1 50%;display:flex;flex-direction:column;align-items:center;justify-content:center}
  h2{font-size:22px;margin:8px;color:#8cf}
  #live{font-size:9vw;font-weight:700}
  iframe{width:96%;height:80%;border:2px solid #333;background:#111}
</style>
<div class="wrap">
  <div class="half"><h2>AHORA (reloj del servidor)</h2><div id="live">--.---</div></div>
  <div class="half"><h2>VIDEO H.264 (mediamtx WebRTC)</h2><iframe src="__WHEP__"></iframe></div>
</div>
<script>
function tick(){
  const ms = Date.now() % 60000;
  document.getElementById("live").textContent =
    String(Math.floor(ms/1000)).padStart(2,"0") + "." + String(ms%1000).padStart(3,"0");
  requestAnimationFrame(tick);
}
tick();
</script>
"""


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass          # the journal gets our own lines, not one per asset fetch

    def _send(self, code, body, ctype):
        raw = body.encode() if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self):
        path, _, query = self.path.partition("?")
        if path == "/":
            page = (PAGE.replace("__SLOT_MS__", str(SLOT_MS))
                        .replace("__HI__", str(LEVEL_HI)))
            return self._send(200, page, "text/html; charset=utf-8")
        if path == "/time":
            # Read the clock as late as possible: anything between here and the write is
            # error the caller will attribute to the network and halve.
            return self._send(200, json.dumps({"epoch_ms": time.time() * 1000.0}),
                              "application/json")
        if path == "/compare":
            q = dict(p.split("=", 1) for p in query.split("&") if "=" in p)
            whep = urllib.parse.unquote(q.get("whep", "https://192.168.20.99:8889/robot"))
            return self._send(200, COMPARE.replace("__WHEP__", whep),
                              "text/html; charset=utf-8")
        if path == "/report":
            with REPORT_LOCK:
                r = dict(REPORT)
            if r:
                r["age_s"] = round(time.time() - r.get("at", 0), 1)
            return self._send(200, json.dumps(r), "application/json")
        if path == "/schedule":
            q = dict(p.split("=", 1) for p in query.split("&") if "=" in p)
            try:
                first = int(q.get("from", "0"))
                count = min(20000, max(1, int(q.get("count", "1000"))))
            except ValueError:
                return self._send(400, '{"error":"bad from/count"}', "application/json")
            return self._send(200, json.dumps({"first_slot": first,
                                               "bits": schedule(first, count)}),
                              "application/json")
        self._send(404, '{"error":"not found"}', "application/json")


    def do_POST(self):
        if self.path != "/report":
            return self._send(404, '{"error":"not found"}', "application/json")
        try:
            n = int(self.headers.get("Content-Length", "0"))
            body = json.loads(self.rfile.read(min(n, 4096)) or b"{}")
        except (ValueError, TypeError):
            return self._send(400, '{"error":"bad json"}', "application/json")
        with REPORT_LOCK:
            REPORT.clear()
            REPORT.update({"offset_ms": float(body.get("offset", 0.0)),
                           "rtt_ms": float(body.get("rtt", 0.0)),
                           "ua": str(body.get("ua", ""))[:200],
                           "screen": str(body.get("screen", ""))[:40],
                           "at": time.time()})
            print(f"[clock] page synced: offset {REPORT['offset_ms']:+.1f} ms, "
                  f"rtt {REPORT['rtt_ms']:.1f} ms, screen {REPORT['screen']}")
        self._send(200, '{"ok":true}', "application/json")


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def serve(port, bind):
    # Binding to every interface is the point, not an oversight: the screen the camera films
    # is usually a phone or a laptop next to the robot, not this machine. What is served is a
    # static clock page, a timestamp and a bit sequence — no secrets, and nothing that can
    # change any state. Pass --bind 127.0.0.1 when the screen IS this machine.
    srv = Server((bind, port), Handler)
    print(f"latency clock on http://{bind}:{port}/   slot={SLOT_MS} ms  "
          f"levels {LEVEL_LO}/{LEVEL_HI}")
    print("Open it on a screen the robot camera can see, fill the frame with it, then run")
    print("`measure` from this same machine. Ctrl-C to stop.")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()
    return 0


# -------------------------------------------------------------------------------- measuring

# The options OpenCV's FFmpeg backend gets. Exposed per source because WHICH of these is set
# turned out to matter more than anything in the robot: the same stream read with the default
# set measured ~2.4 s while a browser on the same mediamtx path was visibly instant.
FFMPEG_DEFAULT = "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|stimeout;5000000"


def read_mjpeg(name, url, seconds, roi):
    """Read an MJPEG-over-HTTP stream by scanning for JPEG markers, with NO FFmpeg.

    This is not a detail. Measured 2026-09-15, OpenCV's FFmpeg-backed reader hands over
    frames that are 2.4 s old on this system while a browser on the same stream is at 0.2 s,
    and no demuxer option changes it. So measuring an MJPEG branch THROUGH FFmpeg would
    charge it for a defect that belongs to the reader, not the branch. This scanner is the
    same shape the bridge's HttpStreamSource uses, which is what makes the number comparable
    to what the drive view actually gets.
    """
    import cv2
    out = []
    req = urllib.request.Request(url, headers={"User-Agent": "latency-clock"})
    t_end = time.monotonic() + seconds
    with urllib.request.urlopen(req, timeout=10) as resp:
        buf = b""
        while time.monotonic() < t_end:
            chunk = resp.read1(65536)
            if not chunk:
                break
            buf += chunk
            while True:
                i = buf.find(b"\xff\xd8")
                if i < 0:
                    break
                j = buf.find(b"\xff\xd9", i + 2)
                if j < 0:
                    break
                jpg, buf = buf[i:j + 2], buf[j + 2:]
                t = time.time() * 1000.0
                g = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_GRAYSCALE)
                if g is None:
                    continue
                if roi:
                    x, y, w, h = roi
                    g = g[y:y + h, x:x + w]
                out.append((t, float(np.mean(g))))
    return out


def read_frames(name, url, seconds, roi, ffmpeg_opts=None):
    """Capture (arrival_epoch_ms, mean_brightness) for `seconds`.

    Brightness is the mean over the ROI, which defaults to the whole frame. The whole frame
    works whenever the screen is a decent share of the view: everything that is NOT the screen
    is static, so it adds a constant that the z-scoring downstream removes. A tighter ROI only
    improves the signal-to-noise, it is never required to get an answer.
    """
    import os

    import cv2
    # "inherit" leaves the environment alone. It exists because setting this variable from
    # INSIDE the process is unreliable: OpenCV caches the parameter on first read, so a late
    # assignment can be silently ignored -- verified here, `stimeout;100` in-process changed
    # nothing while `probesize;32` exported before python started took open() from 3701 ms to
    # 89 ms. Anything that must really apply has to be exported, not assigned.
    if ffmpeg_opts != "inherit":
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = ffmpeg_opts or FFMPEG_DEFAULT
    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        raise OSError(f"[{name}] could not open {url}")
    out = []
    t_end = time.monotonic() + seconds
    try:
        while time.monotonic() < t_end:
            ok, bgr = cap.read()
            # The wall clock is read immediately after the frame lands, and it is the SAME
            # clock the page is disciplined to. That equality is the whole method.
            t = time.time() * 1000.0
            if not ok:
                break
            g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            if roi:
                x, y, w, h = roi
                g = g[y:y + h, x:x + w]
            out.append((t, float(np.mean(g))))
    finally:
        cap.release()
    return out


def correlate(samples, lo_ms, hi_ms, step_ms):
    """Best lag, by Pearson correlation of the received brightness against what was sent.

    Returns (best_lag_ms, peak_r, sigmas, curve). `sigmas` is how far the peak stands above
    the rest of the curve; a peak that is not clearly above its own background is not a
    measurement, and is reported as such rather than quoted.
    """
    t = np.array([s[0] for s in samples], dtype=np.float64)
    y = np.array([s[1] for s in samples], dtype=np.float64)
    if len(t) < 30:
        return None
    y = (y - y.mean()) / (y.std() or 1.0)
    lags = np.arange(lo_ms, hi_ms + step_ms, step_ms, dtype=np.float64)
    rs = []
    for lag in lags:
        expected = expected_for(t, lag)
        x = (expected - expected.mean()) / (expected.std() or 1.0)
        rs.append(float(np.mean(x * y)))
    rs = np.array(rs)
    best = int(np.argmax(rs))
    # The background is everything more than 500 ms away from the peak: near-peak lags are
    # correlated with it by construction and would flatter the significance.
    far = np.abs(lags - lags[best]) > 500
    sigmas = ((rs[best] - rs[far].mean()) / (rs[far].std() or 1e-9)) if far.sum() > 5 else 0.0
    return float(lags[best]), float(rs[best]), float(sigmas), (lags, rs)


def expected_for(times_ms: np.ndarray, lag_ms: float) -> np.ndarray:
    slots = np.floor((times_ms - lag_ms) / SLOT_MS).astype(np.int64)
    return np.array([slot_bit(int(s)) for s in slots], dtype=np.float64)


def check_clock(url_base):
    """Confirm the server we are correlating against is the one on this machine."""
    t0 = time.time() * 1000.0
    with urllib.request.urlopen(f"{url_base}/time", timeout=5) as r:
        server = json.loads(r.read())["epoch_ms"]
    t1 = time.time() * 1000.0
    return server - (t0 + t1) / 2, t1 - t0


def measure(args):
    sources = []
    for spec in args.source:
        name, _, url = spec.partition("=")
        if not url:
            raise SystemExit(f"--source must be name=url, got {spec!r}")
        sources.append((name, url))

    base = f"http://127.0.0.1:{args.port}"
    try:
        off, rtt = check_clock(base)
    except Exception as exc:
        raise SystemExit(f"the clock server is not answering on {base}: {exc}\n"
                         f"start it with:  {sys.argv[0]} serve --port {args.port}") from None
    print(f"clock server on {base}: offset {off:+.2f} ms, rtt {rtt:.2f} ms "
          f"(same machine, so this must be ~0)")

    # The page's own sync, straight from the page. Without this the whole measurement is
    # unfalsifiable: an unsynced page looks exactly like a large latency.
    try:
        with urllib.request.urlopen(f"{base}/report", timeout=5) as r:
            rep = json.loads(r.read())
    except Exception:
        rep = {}
    if not rep:
        raise SystemExit(
            f"the page has not reported its clock sync.\n"
            f"Reload http://<this-host>:{args.port}/ on the screen being filmed, wait ~2 s.\n"
            f"Refusing to measure: an unsynced page measures ITS clock, not ours, which is\n"
            f"how this question got answered -710 ms and -1400 ms before.")
    if rep["age_s"] > 120:
        raise SystemExit(f"the page's last sync is {rep['age_s']:.0f}s old — is it still open?")
    print(f"page reports: offset {rep['offset_ms']:+.1f} ms, rtt {rep['rtt_ms']:.1f} ms, "
          f"screen {rep['screen']}, {rep['age_s']:.0f}s ago")
    if rep["rtt_ms"] > 50:
        print(f"  !! rtt {rep['rtt_ms']:.0f} ms — the sync is worth up to +/-{rep['rtt_ms']/2:.0f} ms "
              f"of the answer")
    print()

    roi = None
    if args.roi:
        roi = tuple(int(v) for v in args.roi.split(","))
        if len(roi) != 4:
            raise SystemExit("--roi must be x,y,w,h")

    results = {}
    threads = []

    def run(name, url):
        try:
            # An http(s) URL is served as MJPEG here, and it is read WITHOUT FFmpeg on
            # purpose -- see read_mjpeg. Force the FFmpeg path with ffmpeg+http://... when
            # you specifically want to measure what FFmpeg costs.
            if url.startswith("ffmpeg+"):
                results[name] = read_frames(name, url[7:], args.seconds, roi, args.ffmpeg_opts)
            elif url.startswith(("http://", "https://")):
                results[name] = read_mjpeg(name, url, args.seconds, roi)
            else:
                results[name] = read_frames(name, url, args.seconds, roi, args.ffmpeg_opts)
        except Exception as exc:
            results[name] = exc

    # Captured in parallel, because the point of comparing two paths is that they see the
    # SAME instants. Running them one after the other would compare two different minutes.
    for name, url in sources:
        th = threading.Thread(target=run, args=(name, url), daemon=True)
        th.start()
        threads.append(th)
    print(f"capturing {args.seconds:.0f}s from {len(sources)} source(s)...")
    for th in threads:
        th.join(timeout=args.seconds + 60)

    print(f"\n{'source':>10} {'frames':>7} {'fps':>6} {'latency':>10} {'r':>7} {'sigmas':>7}")
    print("-" * 54)
    table = {}
    for name, _ in sources:
        got = results.get(name)
        if isinstance(got, Exception) or not got:
            print(f"{name:>10} {'-':>7} {'-':>6}   FAILED: {got}")
            continue
        got = [s for s in got if s[1] == s[1]]
        res = correlate(got, args.min_lag, args.max_lag, args.step)
        if res is None:
            print(f"{name:>10} {len(got):>7} {'-':>6}   too few frames")
            continue
        lag, r, sig, _ = res
        span = (got[-1][0] - got[0][0]) / 1000.0
        table[name] = (lag, sig)
        print(f"{name:>10} {len(got):>7} {len(got)/max(span,1e-9):>6.2f} "
              f"{lag:>8.0f} ms {r:>7.3f} {sig:>7.1f}")

    weak = [n for n, (_, sig) in table.items() if sig < 4.0]
    if weak:
        print(f"\n!! weak peak on {', '.join(weak)} (under 4 sigmas). The panel probably does")
        print("   not cover enough of the frame, or the room light swamps it. Fill more of the")
        print("   view with the screen, or pass --roi x,y,w,h around it. Do NOT quote these.")
    if len(table) == 2:
        (a, (la, _)), (b, (lb, _)) = table.items()
        print(f"\n{a} vs {b}: {la - lb:+.0f} ms")
        print("Both filmed the same panel at the same instants, so everything upstream of the")
        print("split cancels and this difference is the more trustworthy of the three numbers.")
    print("\nThe absolute number is screen-to-here: it includes the panel's own render (one")
    print("display frame, ~16 ms at 60 Hz) and the camera's exposure, and excludes whatever a")
    print("browser would add downstream (WebRTC hop and jitter buffer, ~70 and ~54 ms).")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("serve", help="serve the clock page")
    s.add_argument("--port", type=int, default=8100,
                   help="8100 and not 8099: measure.html's own server already "
                        "sits on 8099 in this directory")
    s.add_argument("--bind", default="0.0.0.0",  # noqa: S104
                   help="default reaches a phone or laptop by the robot; use 127.0.0.1 when "
                        "the screen being filmed is this machine")

    m = sub.add_parser("measure", help="read the video and report the latency")
    m.add_argument("--source", action="append", required=True, metavar="NAME=URL",
                   help="repeatable; measured in PARALLEL so they see the same instants")
    m.add_argument("--port", type=int, default=8100, help="where `serve` is running")
    m.add_argument("--seconds", type=float, default=60.0)
    m.add_argument("--roi", default="", metavar="X,Y,W,H",
                   help="crop to the screen; optional, improves signal-to-noise")
    m.add_argument("--min-lag", type=float, default=0.0)
    m.add_argument("--max-lag", type=float, default=4000.0,
                   help="search window. Wide on purpose: the absolute latency here has never "
                        "been measured, and a window that excludes the answer finds a "
                        "confident wrong one at its edge")
    m.add_argument("--step", type=float, default=5.0)
    m.add_argument("--ffmpeg-opts", default="",
                   help="override OPENCV_FFMPEG_CAPTURE_OPTIONS for every source. The reader's "
                        "own buffering is a first-class suspect, not a detail: measure it "
                        "before blaming the robot")

    args = ap.parse_args()
    if args.cmd == "serve":
        return serve(args.port, args.bind)
    return measure(args)


if __name__ == "__main__":
    sys.exit(main())
