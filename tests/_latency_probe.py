#!/usr/bin/env python3
"""Per-stage latency of the DRIVE video path. TEMPORARY DIAGNOSTIC, read-only.

WHAT IT ANSWERS: the live view is what you steer by, and it was ~700-1000 ms
glass-to-glass. This splits that number into the stages that produce it, so the work goes
where the milliseconds actually are instead of where they are assumed to be.

HOW: `mjpeg_server` with STAMP=1 splices two robot-side timestamps into a JPEG COM segment
(t_in = frame pulled off go2_jpeg_stream, t_out = published to viewers). COM is ignored by
every decoder and every hop downstream forwards the bytes untouched, so the same stamp is
still readable at the browser socket. Costs 1.4 us per frame — measured — which is why it
can be left on during a real drive without changing what it measures.

The robot's clock and this machine's are not locked to each other, so the probe first
measures the offset SNTP-style against /health's `now` field. Everything cross-machine is
corrected by it; t_out - t_in is robot-internal and needs no correction.

    docker exec <devcontainer> python3 - < tests/_latency_probe.py

It runs in the devcontainer because that is where websocket-client lives and where the
backend's self-signed cert is already tolerated. Nothing is written and no command is sent.
"""
import hashlib
import json
import ssl
import statistics as st
import sys
import threading
import time
import urllib.request

import websocket

MJPEG = "http://10.1.254.18:8093"
VIEW = "wss://localhost:8443/ws/view"
SECS = int(sys.argv[1]) if len(sys.argv) > 1 else 25
_TAG = b"AVL1 "


def read_stamp(jpeg):
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


def clock_offset(samples=9):
    """Robot clock minus ours, in seconds. Median of SNTP-style round trips."""
    offs = []
    for _ in range(samples):
        t1 = time.time()
        with urllib.request.urlopen(f"{MJPEG}/health", timeout=5) as r:
            d = json.loads(r.read())
        t3 = time.time()
        if "now" in d:
            offs.append(d["now"] - (t1 + t3) / 2)
    return (st.median(offs) if offs else 0.0), (max(offs) - min(offs) if offs else 0.0), d


def pct(v, p):
    return sorted(v)[min(len(v) - 1, int(len(v) * p))]


def summarize(name, v, unit="ms"):
    if not v:
        print(f"  {name:<44s}  (sin datos)")
        return
    print(f"  {name:<44s}  p50={st.median(v):7.1f}  p95={pct(v, .95):7.1f}  "
          f"max={max(v):7.1f} {unit}   n={len(v)}")


def main():
    off, spread, health = clock_offset()
    print(f"config del robot: {health}")
    print(f"offset de reloj robot-PC: {off * 1000:+.1f} ms  (dispersion {spread * 1000:.1f} ms)")
    if not health.get("stamp"):
        print("\n!! STAMP no esta activo en el robot: pone STAMP=1 en video.env y reinicia "
              "robot-video. Sin eso solo se mide la cadencia, no la latencia.")

    stop = time.time() + SECS
    at_8093, at_view, seen = {}, {}, {}
    lock = threading.Lock()

    def mjpeg(_k):
        try:
            r = urllib.request.urlopen(f"{MJPEG}/stream", timeout=10)
            buf = b""
            while time.time() < stop:
                # read1(), NOT read(): read(n) blocks for the full buffer and would add
                # a whole frame period to every measurement — the very defect this probe
                # found in the camera bridge (213 ms, measured).
                c = r.read1(2048)
                if not c:
                    break
                buf += c
                now = time.time()
                while True:
                    i = buf.find(b"\xff\xd8")
                    j = buf.find(b"\xff\xd9", i + 2)
                    if i < 0 or j < 0:
                        break
                    f = buf[i:j + 2]
                    # A content fingerprint to pair the same frame at two points, not a
                    # security digest — the frames are ours and never leave the LAN.
                    h = hashlib.sha1(f).hexdigest()  # noqa: S324
                    with lock:
                        if h not in at_8093 or now < at_8093[h]:
                            at_8093[h] = now
                        seen.setdefault(h, read_stamp(f))
                    buf = buf[j + 2:]
            r.close()
        except Exception as e:
            print("mjpeg:", e)

    def view():
        try:
            ws = websocket.create_connection(
                VIEW, sslopt={"cert_reqs": ssl.CERT_NONE}, timeout=10)
            while time.time() < stop:
                op = ws.recv()
                now = time.time()
                if isinstance(op, bytes):
                    h = hashlib.sha1(op).hexdigest()  # noqa: S324
                    with lock:
                        at_view.setdefault(h, now)
                        seen.setdefault(h, read_stamp(op))
            ws.close()
        except Exception as e:
            print("view:", e)

    # Several MJPEG clients on purpose: MJPEG_FPS is enforced PER CLIENT, so one client
    # sees only its own subset of frames and would rarely overlap with what the bridge
    # forwarded. The union covers the stream.
    ts = [threading.Thread(target=mjpeg, args=(k,)) for k in range(6)]
    ts.append(threading.Thread(target=view))
    for t in ts:
        t.start()
    for t in ts:
        t.join()

    # Everything cross-machine is corrected by the offset; t_out - t_in is not.
    resize, to_8093, to_view, cadence = [], [], [], []
    for h, sp in seen.items():
        if not sp:
            continue
        t_in, t_out = sp
        resize.append((t_out - t_in) * 1000)
        if h in at_8093:
            to_8093.append((at_8093[h] + off - t_out) * 1000)
        if h in at_view:
            to_view.append((at_view[h] + off - t_out) * 1000)
    tv = sorted(at_view.values())
    cadence = [(tv[i + 1] - tv[i]) * 1000 for i in range(len(tv) - 1)]

    print(f"\nframes: :8093={len(at_8093)}  /ws/view={len(at_view)}  "
          f"con stamp={sum(1 for v in seen.values() if v)}")
    print(f"tasa real del pipeline del robot: {len(at_8093) / SECS:.1f} fps   "
          f"tasa que llega al browser: {len(at_view) / SECS:.1f} fps")
    print("\nDESGLOSE (cada etapa, no acumulado):")
    summarize("mjpeg_server: resize+publicar (t_out-t_in)", resize)
    summarize("robot -> mi cliente en :8093 (red)", to_8093)
    summarize("robot -> /ws/view (red+bridge+backend+hub)", to_view)
    summarize("cadencia entre frames en el browser", cadence)
    if to_view:
        print(f"\n  medido de punta a punta DESDE t_out: {st.median(to_view):.1f} ms p50")
        print("  lo que falta hasta el total glass-to-glass es: sensor -> videohub -> "
              "GetImageSample (arriba de t_in) + decode/render del browser (abajo).")


if __name__ == "__main__":
    main()
