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

The recording branch CANNOT throttle the live branch. rtmpsink blocks when the uplink is
full, GStreamer then stops reading its stdin, the pipe fills, and a blocking write here used
to propagate that stall all the way back into the camera capture — measured, 14 fps of capture
collapsing to 5.9 because the NVR could not keep up. Frames for the NVR now go through a
bounded queue and the OLDEST is dropped when it overflows: the recorder degrades, nothing else
does.

Latency discipline, and it is the whole point of this file:
  * ONE frame of state. Only the newest JPEG is kept; there is no queue to fall behind in.
  * A slow client SKIPS frames instead of delaying everyone. Nothing is ever buffered for it.
  * Passthrough to stdout is byte-exact and never blocks on HTTP clients — the recording
    branch always gets the original 1080p bytes, whatever the live branch is doing.
  * By default nothing is decoded or re-encoded. Set MJPEG_WIDTH to trade a little Jetson
    CPU for much smaller frames, which is what a constrained link to the robot needs: the
    resize happens ONCE per frame, not once per viewer.

Standard library only (Python 3.8 on the robot).
"""
import collections
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
# Downscale for HTTP viewers only. The videohub always answers 1080p (~200 KB a frame), and
# MJPEG has no inter-frame compression, so at 3 fps that is ~600 KB/s — more than a
# constrained link to the robot delivers, which then costs frames AND latency. Shrinking is
# what actually helps here; dropping frames (MJPEG_FPS) does not make each one smaller.
# The NVR branch is NEVER touched: it keeps the original bytes at full resolution.
WIDTH = int(os.environ.get("MJPEG_WIDTH", "0") or 0)      # 0 = native, no re-encode
QUALITY = int(os.environ.get("MJPEG_QUALITY", "75") or 75)
BOUNDARY = "frame"

_cv2 = None
if WIDTH > 0:
    try:
        import cv2 as _cv2  # noqa: N813
        import numpy as np
    except ImportError:
        _cv2 = None


def _shrink(jpeg):
    """Decode, resize, re-encode — once per frame, not once per client.

    Returns the original bytes on any failure: a viewer seeing a big frame is much better
    than a viewer seeing nothing, and this must never be able to break the stream.
    """
    if not _cv2 or WIDTH <= 0:
        return jpeg
    try:
        img = _cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), _cv2.IMREAD_COLOR)
        if img is None:
            return jpeg
        h, w = img.shape[:2]
        if w <= WIDTH:
            return jpeg
        small = _cv2.resize(img, (WIDTH, int(h * WIDTH / w)),
                            interpolation=_cv2.INTER_AREA)
        ok, buf = _cv2.imencode(".jpg", small,
                                [int(_cv2.IMWRITE_JPEG_QUALITY), QUALITY])
        return buf.tobytes() if ok else jpeg
    except Exception:
        return jpeg


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


# Two slots on purpose. pump() publishes to RAW and returns to reading immediately; a
# worker thread does the expensive decode/resize/encode and publishes to LATEST. Doing the
# resize inline in pump() throttled the CAPTURE itself — measured 3.2 fps dropping to 2.3 —
# because every cycle waited for a 1080p decode before reading the next frame.
RAW = Latest()
LATEST = Latest()


def resizer():
    """Shrink the newest frame, forever. Skipping intermediate frames is correct: a viewer
    wants the latest picture, not a backlog of stale ones."""
    seq = -1
    while True:
        jpeg, seq = RAW.get_newer_than(seq, timeout=5.0)
        if jpeg is not None:
            LATEST.put(_shrink(jpeg))


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
                b'{"ok":true,"clients":%d,"fps_cap":%s,"width":%d,"quality":%d,'
                b'"nvr_queue":%d,"nvr_dropped":%d}'
                % (LATEST.clients, str(FPS or "none").encode(), WIDTH, QUALITY,
                   len(_nvr), _nvr_dropped)
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


PUBLISH = None      # set in main(): RAW.put when resizing, LATEST.put when not

# The NVR branch gets a DELIBERATE, REGULAR rate — not "whatever is left over".
#
# Dropping the oldest frame whenever the queue filled did keep the capture free, but it fed
# the H.264 encoder an irregular 27% of frames: with fdsrc do-timestamp=true the timestamps
# then jump around, and the FLV/RTMP stream that comes out is one Frigate cannot even start
# on ("no frames have been received"). A DECODER tolerates dropped frames; an ENCODER needs a
# cadence. So pick frames on a fixed interval instead, and keep the bounded queue only as a
# safety net for a genuine stall.
NVR_FPS = float(os.environ.get("NVR_FPS", "5"))
NVR_MAX = int(os.environ.get("NVR_QUEUE", "8"))
_nvr_last = 0.0
_nvr = collections.deque()
_nvr_cv = threading.Condition()
_nvr_dropped = 0


def nvr_offer(frame):
    """Offer a frame to the recording branch at a steady rate. Never blocks the caller."""
    global _nvr_dropped, _nvr_last
    if NVR_FPS > 0:
        now = time.monotonic()
        if now - _nvr_last < 1.0 / NVR_FPS:
            return                      # not this one: keeps the cadence regular
        _nvr_last = now
    with _nvr_cv:
        while len(_nvr) >= NVR_MAX:
            _nvr.popleft()
            _nvr_dropped += 1
        _nvr.append(frame)
        _nvr_cv.notify()


def nvr_writer():
    """The only thing allowed to block on stdout."""
    dst = sys.stdout.buffer
    while True:
        with _nvr_cv:
            while not _nvr:
                _nvr_cv.wait(5.0)
            frame = _nvr.popleft()
        try:
            dst.write(frame)
            dst.flush()
        except (BrokenPipeError, OSError):
            log("downstream (NVR) closed")
            return


def pump():
    """stdin -> stdout passthrough, publishing each JPEG as it goes by.

    Frames are found by SOI/EOI markers rather than by trusting any framing, which is what
    makes this composable with go2_jpeg_stream's raw concatenated output.
    """
    src = sys.stdin.buffer
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
            frame = buf[start:end + 2]
            buf = buf[end + 2:]
            # Live first: it is the branch whose latency we care about.
            PUBLISH(frame)
            nvr_offer(frame)
            frames += 1
            if frames % 300 == 0:
                dt = time.monotonic() - t0
                log(f"{frames} frames, {frames / dt:.1f} fps in, "
                    f"{LATEST.clients} viewer(s), {_nvr_dropped} dropped to NVR")


def main():
    global PUBLISH
    if WIDTH > 0 and _cv2 is not None:
        # Resizing: hand frames to the worker so the capture loop never waits for it.
        PUBLISH = RAW.put
        threading.Thread(target=resizer, name="resizer", daemon=True).start()
    else:
        # Nothing to do per frame: publish straight to the served slot.
        PUBLISH = LATEST.put

    threading.Thread(target=nvr_writer, name="nvr-writer", daemon=True).start()
    srv = ThreadingHTTPServer((BIND, PORT), Handler)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    size = f"{WIDTH}px wide" if WIDTH > 0 else "native"
    if WIDTH > 0 and _cv2 is None:
        log("MJPEG_WIDTH is set but cv2 is missing — serving frames at native size")
        size = "native (cv2 missing)"
    log(f"serving http://{BIND}:{PORT}/stream  (fps cap: {FPS or 'none'}, {size})")
    try:
        pump()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
