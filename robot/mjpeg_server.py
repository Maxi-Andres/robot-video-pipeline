#!/usr/bin/env python3
"""
mjpeg_server — a passthrough tee: serves the robot's JPEG frames over HTTP while forwarding
them, byte for byte, to whatever comes next in the pipe.

    go2_jpeg_stream | mjpeg_server.py | gst-launch-1.0 ... (H.264 -> RTMP -> NVR)
                           |
                           +-- HTTP /stream  -> AI-VL camera bridge, Splunk <img>, browsers

WHY: the live view must NOT travel through the recording chain. Reading the camera off DDS on
the same subnet used to be ~instant because the app got the JPEGs directly; routing the live
view through encode -> RTMP -> mediamtx -> Frigate -> MJPEG added ~7 s, because an NVR buffers
on purpose. This restores the short path with HTTP instead of DDS as the transport, so it
works with the robot on any network, and leaves the H.264/RTMP path untouched for recording —
where latency does not matter.

Latency discipline, and it is the whole point of this file:
  * ONE frame of state. Only the newest JPEG is kept; there is no queue to fall behind in.
  * A slow client SKIPS frames instead of delaying everyone. Nothing is ever buffered for it.
  * Passthrough to stdout is byte-exact and never blocks on HTTP clients.
  * No decode, no re-encode, no resize: the bytes the robot produced are the bytes served.

Standard library only (Python 3.8 on the robot).
"""
import os
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PORT = int(os.environ.get("MJPEG_PORT", "8093"))
BIND = os.environ.get("MJPEG_BIND", "0.0.0.0")
# Cap for HTTP viewers only. Independent of the rate flowing to the NVR, so the live view can
# be made cheaper without touching the recording.
FPS = float(os.environ.get("MJPEG_FPS", "0")) or 0.0      # 0 = every frame
BOUNDARY = "frame"


def log(msg):
    print(f"[mjpeg] {msg}", file=sys.stderr, flush=True)


class Latest:
    """The newest frame, and a way to wait for one newer than the one you last saw.

    Deliberately not a queue: a queue is how latency accumulates. Clients are told the
    sequence number they received, and simply miss whatever went by while they were busy.
    """

    def __init__(self):
        self._cv = threading.Condition()
        self._jpeg = None
        self._seq = 0
        self.clients = 0

    def put(self, jpeg):
        with self._cv:
            self._jpeg = jpeg
            self._seq += 1
            self._cv.notify_all()

    def get_newer_than(self, seq, timeout=5.0):
        with self._cv:
            if self._seq == seq:
                self._cv.wait(timeout)
            if self._seq == seq:
                return None, seq
            return self._jpeg, self._seq


LATEST = Latest()


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "robot-mjpeg"

    def log_message(self, *a):
        pass

    def do_GET(self):
        path = self.path.split("?")[0].rstrip("/") or "/"
        if path in ("/", "/stream"):
            return self._stream()
        if path == "/snapshot":
            return self._snapshot()
        if path == "/health":
            body = (
                b'{"ok":true,"clients":%d,"fps_cap":%s}'
                % (LATEST.clients, str(FPS or "none").encode())
            )
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404)

    def _snapshot(self):
        jpeg, _ = LATEST.get_newer_than(-1, timeout=3.0)
        if not jpeg:
            return self.send_error(503, "no frame yet")
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(jpeg)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(jpeg)

    def _stream(self):
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type",
                         f"multipart/x-mixed-replace; boundary={BOUNDARY}")
        self.end_headers()
        LATEST.clients += 1
        seq = -1
        min_gap = (1.0 / FPS) if FPS > 0 else 0.0
        last = 0.0
        try:
            while True:
                jpeg, seq = LATEST.get_newer_than(seq, timeout=10.0)
                if jpeg is None:
                    continue                      # no new frame yet; keep the socket open
                if min_gap:
                    now = time.monotonic()
                    if now - last < min_gap:
                        continue                  # honour the cap by DROPPING, not delaying
                    last = now
                self.wfile.write(
                    b"--" + BOUNDARY.encode() + b"\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
                    + jpeg + b"\r\n")
        except (BrokenPipeError, ConnectionResetError, socket.timeout):
            pass                                   # viewer went away; normal
        finally:
            LATEST.clients -= 1


def pump():
    """stdin -> stdout passthrough, publishing each JPEG as it goes by.

    Frames are found by SOI/EOI markers rather than by trusting any framing, which is what
    makes this composable with go2_jpeg_stream's raw concatenated output.
    """
    src = sys.stdin.buffer
    dst = sys.stdout.buffer
    buf = b""
    frames = 0
    t0 = time.monotonic()
    while True:
        # read1(), NOT read(): read(n) on a pipe blocks until it has ALL n bytes, so with
        # 15 fps of ~200 KB frames it would sit on a full second of video before publishing
        # any of it — a second of latency, in the one file whose job is to remove latency.
        # read1() returns whatever the pipe has right now.
        chunk = src.read1(65536)
        if not chunk:
            log("stdin closed, exiting")
            return
        # Forward FIRST, always: the recording path must never depend on the HTTP half.
        dst.write(chunk)
        dst.flush()
        buf += chunk
        while True:
            start = buf.find(b"\xff\xd8")
            if start < 0:
                buf = buf[-1:]
                break
            end = buf.find(b"\xff\xd9", start + 2)
            if end < 0:
                if start > 0:
                    buf = buf[start:]
                break
            LATEST.put(buf[start:end + 2])
            buf = buf[end + 2:]
            frames += 1
            if frames % 300 == 0:
                dt = time.monotonic() - t0
                log(f"{frames} frames, {frames / dt:.1f} fps in, {LATEST.clients} viewer(s)")


def main():
    srv = ThreadingHTTPServer((BIND, PORT), Handler)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    log(f"serving http://{BIND}:{PORT}/stream  (fps cap: {FPS or 'none'})")
    try:
        pump()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
