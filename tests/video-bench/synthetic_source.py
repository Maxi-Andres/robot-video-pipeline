#!/usr/bin/env python3
"""Synthetic source for the video-latency bench.

Renders frames that carry their own creation time as a BINARY BARCODE burned into the
pixels. A barcode survives H.264 (unlike the JPEG COM stamp the robot uses today), so the
same frame can be timed on both branches:

    render ──┬── JPEG over HTTP  :PORT/mjpeg          (mirrors the robot's :8093 live path)
             └── JPEG on stdout -> ffmpeg -> RTMP     (mirrors the robot's NVR branch)

Both consumers get the SAME frame object with the SAME stamp, so the difference between
the two measured latencies is exactly what H.264+RTMP+mediamtx+WebRTC costs.

Barcode layout, one band of CELL x CELL squares at the top-left corner:

    [white][black]  <24 bits: ms since EPOCH_BASE, mod 2^24>  <8 bits checksum>  [black][white]

24 bits of milliseconds wraps every 16.7 s, which is far longer than any latency we would
call acceptable, so the wrap is resolvable at read time.
"""
from __future__ import annotations

import argparse
import http.server
import io
import socketserver
import sys
import threading
import time

import numpy as np
from PIL import Image, ImageDraw, ImageFont

EPOCH_BASE = 1789000000000  # fixed origin so the 24-bit counter is well defined
CELL = 16  # barcode cell side in px; big enough to survive chroma subsampling
BAND = 32  # barcode band height in px
NBITS = 24
NCHK = 8


def checksum(value: int) -> int:
    b0 = value & 0xFF
    b1 = (value >> 8) & 0xFF
    b2 = (value >> 16) & 0xFF
    return (b0 + b1 + b2) & 0xFF


def encode_cells(stamp_ms: int) -> list[int]:
    v = (stamp_ms - EPOCH_BASE) & 0xFFFFFF
    bits = [(v >> (NBITS - 1 - i)) & 1 for i in range(NBITS)]
    c = checksum(v)
    bits += [(c >> (NCHK - 1 - i)) & 1 for i in range(NCHK)]
    return [1, 0, *bits, 0, 1]


class Renderer:
    def __init__(self, width: int, height: int, quality: int):
        self.w, self.h = width, height
        self.quality = quality
        # Static backdrop: a coarse checkerboard gives the encoder real detail to chew on,
        # so the bitrate is not the unrealistically low one of a flat test card.
        yy, xx = np.mgrid[0:height, 0:width]
        base = (((xx // 40) + (yy // 40)) % 2).astype(np.uint8) * 40 + 60
        self.backdrop = np.dstack([base, base, base])
        try:
            self.font = ImageFont.truetype(
                "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", 48
            )
        except OSError:
            self.font = ImageFont.load_default()

    def render(self, stamp_ms: int, seq: int) -> bytes:
        img = self.backdrop.copy()

        # Moving bar: keeps inter-frame motion realistic and gives the eye something to
        # follow when watching the two branches side by side.
        x = int((seq * 17) % max(1, self.w - 60))
        img[BAND + 8 : BAND + 88, x : x + 60] = 255

        # Barcode band.
        cells = encode_cells(stamp_ms)
        img[0:BAND, 0 : len(cells) * CELL] = 0
        for i, bit in enumerate(cells):
            if bit:
                img[0:BAND, i * CELL : (i + 1) * CELL] = 255

        pil = Image.fromarray(img)
        d = ImageDraw.Draw(pil)
        d.text((8, BAND + 96), f"{stamp_ms % 100000:05d} ms", fill=(255, 255, 0), font=self.font)

        buf = io.BytesIO()
        pil.save(buf, format="JPEG", quality=self.quality)
        return buf.getvalue()


class Hub:
    """Latest-frame-wins fan-out. Never queues: a slow reader gets the newest frame, never
    a backlog — the same rule the robot bridge needed (fix #8 in the plan)."""

    def __init__(self):
        self.cv = threading.Condition()
        self.frame: bytes | None = None
        self.seq = 0

    def publish(self, jpg: bytes) -> None:
        with self.cv:
            self.frame = jpg
            self.seq += 1
            self.cv.notify_all()

    def wait(self, last_seq: int, timeout: float = 5.0):
        with self.cv:
            if self.seq == last_seq:
                self.cv.wait(timeout)
            return self.seq, self.frame


HUB = Hub()
PAGE_DIR = None


class Handler(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):  # keep stderr for real diagnostics
        pass

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/mjpeg":
            return self.serve_mjpeg()
        if path in ("/", "/measure.html"):
            return self.serve_file("measure.html", "text/html; charset=utf-8")
        if path == "/rig.js":
            return self.serve_file("rig.js", "application/javascript; charset=utf-8")
        self.send_error(404)

    def do_POST(self):
        if self.path.split("?", 1)[0] != "/report":
            return self.send_error(404)
        n = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(n)
        with open(f"{PAGE_DIR}/report.jsonl", "ab") as fh:
            fh.write(body + b"\n")
        self.send_response(204)
        self.send_header("Content-Length", "0")
        self.end_headers()

    def serve_file(self, name: str, ctype: str):
        try:
            with open(f"{PAGE_DIR}/{name}", "rb") as fh:
                body = fh.read()
        except OSError:
            return self.send_error(404)
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def serve_mjpeg(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        seq = -1
        try:
            while True:
                seq, jpg = HUB.wait(seq)
                if jpg is None:
                    continue
                self.wfile.write(
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(jpg)).encode()
                    + b"\r\n\r\n"
                    + jpg
                    + b"\r\n"
                )
        except (BrokenPipeError, ConnectionResetError):
            pass


class Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    global PAGE_DIR
    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=float, default=15.0)
    ap.add_argument("--quality", type=int, default=75)
    ap.add_argument("--port", type=int, default=8099)
    # 127.0.0.1 by default: the only consumer is a browser on this machine, and the page has
    # no authentication. Pass --bind 0.0.0.0 only to measure from another host.
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--pagedir", default=".")
    ap.add_argument("--stdout", action="store_true", help="also write JPEGs to stdout")
    ap.add_argument("--serve-only", action="store_true",
                    help="serve measure.html and nothing else — for measuring the REAL "
                         "robot, where the page needs a same-origin host but no synthetic "
                         "source is wanted")
    args = ap.parse_args()
    PAGE_DIR = args.pagedir

    srv = Server((args.bind, args.port), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"[rig] http://{args.bind}:{args.port}/  (page)  /mjpeg (stream)", file=sys.stderr)

    if args.serve_only:
        print("[rig] serve-only: no frames generated", file=sys.stderr)
        threading.Event().wait()

    r = Renderer(args.width, args.height, args.quality)
    out = sys.stdout.buffer if args.stdout else None
    period = 1.0 / args.fps
    seq = 0
    next_t = time.monotonic()
    while True:
        next_t += period
        stamp = int(time.time() * 1000)
        jpg = r.render(stamp, seq)
        HUB.publish(jpg)
        if out is not None:
            try:
                out.write(jpg)
                out.flush()
            except BrokenPipeError:
                break
        seq += 1
        sleep = next_t - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_t = time.monotonic()  # fell behind; resync instead of spiralling


if __name__ == "__main__":
    main()
