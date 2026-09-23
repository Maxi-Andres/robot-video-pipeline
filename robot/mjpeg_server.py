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
import atexit
import collections
import json
import os
import select
import socket
import struct
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs

PORT = int(os.environ.get("MJPEG_PORT", "8093"))
BIND = os.environ.get("MJPEG_BIND", "0.0.0.0")  # noqa: S104  # known finding P0-1: binds broadly, no auth yet
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
np = None
# Resize on the Jetson's JPEG hardware (NVJPG) instead of OpenCV. Measured 2026-09-16 on this
# Orin NX: 10.5 ms a frame against 28 ms for the cv2 path, and ~0% CPU instead of ~36% of a
# core. Set MJPEG_HW=0 to force the cv2 path; it is also a live parameter, so the whole thing
# can be switched off through the relay without SSH.
HW = os.environ.get("MJPEG_HW", "1") != "0"
# Deadline for one frame through the hardware child. Anything slower means the child is wedged,
# and a wedged child is worse than a slow one: the live view would freeze on an old picture
# instead of degrading. On a timeout the child is killed and the frame goes through cv2.
HW_TIMEOUT_MS = int(os.environ.get("MJPEG_HW_TIMEOUT_MS", "200"))
# How many threads OpenCV may use. One, on purpose: its pool is sized for the machine, not for
# this process's share of it, and several native threads finish the same work while burning
# far more of whatever CPU budget the cgroup allows.
CV_THREADS = int(os.environ.get("MJPEG_CV_THREADS", "1"))

_cv2_tried = False


def _load_cv2():
    """Import cv2 on FIRST USE, not at import time.

    WHY: this used to be `if WIDTH > 0: import cv2` at module scope, which quietly made the
    resize un-switchable. WIDTH can now be changed at runtime (POST /config), and a process
    that started at WIDTH=0 would have found `_cv2 is None` and gone on serving native
    frames — the operator moves the control, the number changes, and nothing happens. The
    import is cheap and happens at most once.
    """
    global _cv2, np, _cv2_tried
    if _cv2_tried:
        return _cv2
    _cv2_tried = True
    try:
        import cv2
        import numpy
    except ImportError:
        log("cv2 is not installed — frames stay at native size whatever MJPEG_WIDTH says")
        return None
    _cv2, np = cv2, numpy
    try:
        cv2.setNumThreads(CV_THREADS)
    except Exception:
        pass
    return _cv2


def _jpeg_size(jpeg):
    """(width, height) from the JPEG's SOF marker, WITHOUT decoding it. Microseconds.

    The hardware path needs the source size to compute a height, and nvvidconv does NOT
    preserve aspect ratio — give it only a width and it picks whatever height it likes. Doing
    a decode just to learn the size would defeat the entire point of the hardware path.
    """
    i = 2
    n = len(jpeg)
    while i + 9 < n:
        if jpeg[i] != 0xFF:
            i += 1
            continue
        marker = jpeg[i + 1]
        if marker in (0xC0, 0xC1, 0xC2, 0xC3):          # SOF0/1/2/3
            h, w = struct.unpack(">HH", jpeg[i + 5:i + 9])
            return w, h
        if marker in (0xD8, 0xD9) or 0xD0 <= marker <= 0xD7:   # SOI/EOI/RST carry no length
            i += 2
            continue
        i += 2 + struct.unpack(">H", jpeg[i + 2:i + 4])[0]
    return None, None


def stop_child(proc, label):
    """Shut a gst-launch child down GENTLY first. This is not politeness, it is a leak fix.

    The NV elements hold NVMM buffer pools, which live in nvmap — a pool separate from system
    RAM and invisible to `free`. Killed with SIGKILL they never release them, and NVMM does not
    come back until reboot. Observed 2026-09-16: three quality changes in a row, each
    rebuilding the child, and the fourth start died with
    `PosixMemMap:84 mmap failed : Cannot allocate memory` on a machine with 12 GB free.

    Closing stdin makes fdsrc emit EOS, which walks the pipeline down and frees the pools.
    SIGKILL stays as the last resort for a child that is genuinely wedged — the case the
    read/write deadlines exist to catch.

    Module-level because there are now TWO of these children (the JPEG resizer and the H.264
    drive branch) and they must not learn this the hard way twice.
    """
    if proc is None:
        return
    try:
        proc.stdin.close()          # -> EOS -> clean teardown -> NVMM released
    except Exception:
        pass
    for step, wait in ((None, 1.5), (proc.terminate, 1.0), (proc.kill, 1.0)):
        if step is not None:
            try:
                step()
            except Exception:
                pass
        try:
            proc.wait(timeout=wait)
            break
        except Exception:  # noqa: S112  # still alive: fall through to the harder signal
            continue
    # Close every fd explicitly. The size can be moved from the panel, and each move rebuilds
    # the child: leaking three fds a time turns an afternoon of tuning into EMFILE.
    for handle in (proc.stdin, proc.stdout, proc.stderr):
        try:
            handle and handle.close()
        except Exception:
            pass
    log(f"{label}: child stopped")


def drain_stderr(proc, label):
    """Drain a child's stderr forever. gst-launch is chatty; an unread stderr pipe fills at
    64 KB and then the child blocks writing to it, which looks exactly like a wedged encoder."""
    try:
        for line in iter(proc.stderr.readline, b""):
            text = line.decode("utf-8", "replace").strip()
            if text:
                log(f"{label} child: {text}")
    except Exception:
        pass


class _HwResizer:
    """Resize on the Jetson's JPEG hardware, through a persistent gst-launch child.

    THE PIPELINE, and the two things that took a validation run to learn:

        fdsrc ! jpegparse ! nvjpegdec ! nvvidconv
              ! video/x-raw(memory:NVMM),width=W,height=H ! nvjpegenc ! fdsink

      * The caps MUST be `(memory:NVMM)`. With plain `video/x-raw` the pipeline NEGOTIATES
        AND FAILS: `not-negotiated`, and it produces a zero-byte output while looking alive.
      * `nvvidconv` does not preserve aspect ratio, so both width AND height are pinned, both
        even — hence _jpeg_size() above.

    WHY A CHILD PROCESS and not gi/Gst in this one. This process owns the tee: its stdout is
    the recording branch. Loading the NV GStreamer elements in-process puts a stack with a
    documented double-free (see run-video.sh) inside the one process that must not die, and
    appsink's callbacks would take the GIL we are trying to get off. A child can crash on its
    own time; we notice, and fall back.

    LOCKSTEP, ONE FRAME IN FLIGHT. Write one, read one, never queue. That makes the pairing of
    a frame with its t_in trivially unambiguous, and it makes it IMPOSSIBLE for latency to pile
    up inside GStreamer — which is the only way this could end up worse than cv2. If the
    hardware ever ran slower than the source, frames get skipped, which is what this whole file
    already does everywhere else.
    """

    # Consecutive failures before the hardware path is abandoned for the life of the process.
    # A child that cannot start once is a hiccup; three in a row is a broken machine, and
    # retrying forever would put a multi-hundred-ms timeout in front of every single frame.
    _MAX_FAILS = 3
    # The first frame pays the pipeline's preroll. Measured ~300 ms, so allow well over it
    # before concluding the child is wedged.
    _FIRST_TIMEOUT_MS = 2500

    def __init__(self):
        self._proc = None
        self._key = None            # (width, quality, src_w, src_h) the child was built for
        self._lock = threading.Lock()
        self._fails = 0
        self._dead = False
        self._first = True
        self.path = "cv2"           # what actually served the last frame, for /health
        self.fallbacks = 0

    # -- child lifecycle ---------------------------------------------------------------
    def _stop(self):
        proc, self._proc, self._key = self._proc, None, None
        stop_child(proc, "hw resize")

    def _start(self, width, quality, src_w, src_h):
        height = max(2, round(src_h * width / float(src_w)) & ~1)
        caps = f"video/x-raw(memory:NVMM),width={width},height={height}"
        cmd = ["gst-launch-1.0", "-q",
               "fdsrc", "fd=0", "!", "jpegparse", "!", "nvjpegdec", "!", "nvvidconv", "!",
               caps, "!", "nvjpegenc", f"quality={quality}", "!", "fdsink", "fd=1"]
        self._proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                      stderr=subprocess.PIPE, bufsize=0)
        self._key = (width, quality, src_w, src_h)
        self._first = True
        # Drain stderr forever. gst-launch is chatty; an unread stderr pipe fills at 64 KB and
        # then the child blocks writing to it, which looks exactly like a wedged encoder.
        threading.Thread(target=drain_stderr, args=(self._proc, "hw resize"),
                         daemon=True).start()
        log(f"hw resize: {src_w}x{src_h} -> {width}x{height} "
            f"q{quality} (pid {self._proc.pid})")

    # -- the frame path ----------------------------------------------------------------
    def shrink(self, jpeg, width, quality):
        """Resized JPEG bytes, or None to tell the caller to use its own fallback."""
        if self._dead:
            return None
        with self._lock:
            try:
                out = self._shrink_locked(jpeg, width, quality)
            except Exception as exc:
                out = None
                log(f"hw resize failed: {exc}")
            if out:
                self._fails = 0
                self.path = "hw"
                return out
            # Any failure leaves the child's stream out of step with ours; there is no safe
            # way to resynchronise a half-written frame, so the child goes and a fresh one is
            # started on the next frame.
            self._stop()
            self.fallbacks += 1
            self.path = "cv2"
            self._fails += 1
            if self._fails >= self._MAX_FAILS:
                self._dead = True
                log(f"hw resize gave up after {self._fails} consecutive failures "
                    f"— staying on cv2")
            return None

    def _shrink_locked(self, jpeg, width, quality):
        src_w, src_h = _jpeg_size(jpeg)
        if not src_w or src_w <= width:
            return None                      # unknown size, or nothing to shrink
        key = (width, quality, src_w, src_h)
        if self._proc is None or self._proc.poll() is not None or self._key != key:
            self._stop()
            self._start(width, quality, src_w, src_h)

        deadline = time.monotonic() + (
            self._FIRST_TIMEOUT_MS if self._first else HW_TIMEOUT_MS) / 1000.0
        self._first = False

        if not self._write(jpeg, deadline):
            return None
        return self._read_jpeg(deadline)

    def _write(self, data, deadline):
        """Write the whole frame, never blocking past the deadline.

        A 1080p JPEG is ~200 KB and a pipe holds 64 KB, so this WILL block if the child is not
        reading — which is precisely the wedged case we must not wait out.
        """
        fd = self._proc.stdin.fileno()
        view = memoryview(data)
        while view:
            left = deadline - time.monotonic()
            if left <= 0 or not select.select([], [fd], [], left)[1]:
                return False
            view = view[os.write(fd, view):]
        return True

    def _read_jpeg(self, deadline):
        """One complete JPEG off the child's stdout, or None."""
        fd = self._proc.stdout.fileno()
        buf = b""
        while True:
            left = deadline - time.monotonic()
            if left <= 0 or not select.select([fd], [], [], left)[0]:
                return None
            chunk = os.read(fd, 65536)
            if not chunk:
                return None                  # child closed: treat as a failure, not as EOF
            buf += chunk
            start = buf.find(b"\xff\xd8")
            if start < 0:
                continue
            end = buf.find(b"\xff\xd9", start + 2)
            if end >= 0:
                return bytes(buf[start:end + 2])


_HW = _HwResizer()
atexit.register(_HW._stop)


def _shrink(jpeg):
    """Decode, resize, re-encode — once per frame, not once per client.

    Reads WIDTH/QUALITY at CALL time, not at import: both are runtime-tunable.

    Returns the original bytes on any failure: a viewer seeing a big frame is much better
    than a viewer seeing nothing, and this must never be able to break the stream.

    Three levels, each catching the one above: the JPEG hardware, then OpenCV on the CPU,
    then the original bytes. None of them may raise.
    """
    if WIDTH <= 0:
        return jpeg
    if HW:
        out = _HW.shrink(jpeg, WIDTH, QUALITY)
        if out:
            return out
    if not _load_cv2():
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


# --------------------------------------------------------------------------- #
# Latency instrumentation — OFF by default, and free when off.
#
# WHY: the live view is what you steer by, and it was measured at ~700-1000 ms
# glass-to-glass while the whole downstream chain (network + camera_bridge + backend +
# hub) accounts for only 4.8 ms p50 (measured 2026-09-10 by hash-correlating the same
# frame at :8093 and at the browser socket). So essentially ALL of it is upstream of
# here, and the only way to split "the robot's camera service" from "this process" is to
# carry a capture time INSIDE the frame.
#
# HOW: a JPEG COM segment spliced in right after SOI. Two bytes of marker, two of length,
# then the payload — no decode, no re-encode, one bytes concat per frame. Every decoder
# ignores COM, so the frame stays valid all the way to the browser and the passthrough
# fast paths downstream (camera_bridge quality=0/native, backend hub fan-out) forward it
# untouched. That is what makes the measurement honest: the thing being measured is not
# perturbed by measuring it.
#
# Set STAMP=1 to enable. Read it downstream with read_stamp().
# --------------------------------------------------------------------------- #
STAMP = os.environ.get("STAMP", "0") == "1"

# ---------------------------------------------------------------------------------------
# THE DRIVE BRANCH: all-intra H.264, half the bytes of the JPEG at the same quality.
#
# OFF BY DEFAULT. It costs a second hardware encode, and nothing consumes it until the
# browser side lands — see PLAN-VIDEO.md 6.g C/D. Turning it on changes nothing for the
# MJPEG or the NVR: it is a third reader of RAW, gated on its own.
#
# MEASURED 2026-09-21 on this robot, against the uncompressed original: at PSNR 29.34 dB an
# all-intra frame costs 4151 B where the JPEG at 29.30 dB costs 8663. QP 40 is that point;
# QP 32 lands at the JPEG's size with ~1 dB more quality.
# ---------------------------------------------------------------------------------------
H264_ENABLE = os.environ.get("H264_ENABLE", "0") == "1"
H264_WIDTH = int(os.environ.get("H264_WIDTH", "480") or 480)
H264_HEIGHT = int(os.environ.get("H264_HEIGHT", "270") or 270)
H264_QP = int(os.environ.get("H264_QP", "40") or 40)
H264_FPS = float(os.environ.get("H264_FPS", "0") or 0.0)   # 0 = every frame
_SOI = b"\xff\xd8"
_COM = b"\xff\xfe"
_TAG = b"AVL1 "


def stamp(jpeg, t_in, t_out):
    """Splice a COM segment carrying the two robot-side timestamps, as ASCII seconds.

    t_in  — when pump() pulled the frame off go2_jpeg_stream's stdout
    t_out — when it was published to viewers, i.e. after any resize

    Their difference is this process's own cost; the difference between t_out and a
    downstream arrival time is everything after the robot.
    """
    payload = _TAG + b"%.6f %.6f" % (t_in, t_out)
    seg = _COM + bytes(((len(payload) + 2) >> 8, (len(payload) + 2) & 0xFF)) + payload
    return jpeg[:2] + seg + jpeg[2:]


def read_stamp(jpeg):
    """(t_in, t_out) from a stamped frame, or None. Mirror of stamp(); used by the
    measuring client, which lives off-robot — keep the two in step."""
    if not jpeg.startswith(_SOI) or jpeg[2:4] != _COM:
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


# --------------------------------------------------------------------------- #
# Live-tunable parameters
#
# An ALLOWLIST with a range per key, not a generic setter. Same reasoning as the relay's
# verb table: a generic "write any name to any value" is how a typo takes the video off the
# air with no way back except SSH — which is the exact trip this endpoint exists to save.
#
# Ranges: fps 0 = uncapped; width 0 = native (no decode at all); quality only matters when
# width > 0, since at native size the bytes are forwarded untouched.
# --------------------------------------------------------------------------- #
LIVE_PARAMS = {
    "fps":     ("FPS", float, 0.0, 60.0),
    "width":   ("WIDTH", int, 0, 1920),
    "quality": ("QUALITY", int, 1, 100),
    # The hardware resize, on/off from the relay. This is the cheapest possible rollback for
    # the riskiest part of the live path: no SSH, no restart, no redeploy.
    "hw":      ("HW", int, 0, 1),
    # The drive branch, live. Same reasoning as `hw`: the rollback for a new path must not
    # need SSH. `h264_qp` is the quality/size dial — lower is better and bigger.
    "h264":       ("H264_ENABLE", int, 0, 1),
    "h264_qp":    ("H264_QP", int, 10, 51),
    "h264_width": ("H264_WIDTH", int, 64, 1920),
    "h264_fps":   ("H264_FPS", float, 0.0, 60.0),
}


def live_params():
    return {"fps": FPS, "width": WIDTH, "quality": QUALITY, "hw": int(bool(HW))}


def set_live_params(body):
    """Apply a {fps,width,quality} subset. Returns what changed. Raises ValueError.

    Validated fully BEFORE anything is applied, so a bad value in a two-key request cannot
    leave the stream half-reconfigured.
    """
    unknown = set(body) - set(LIVE_PARAMS)
    if unknown:
        raise ValueError(f"unknown parameter(s): {sorted(unknown)}; "
                         f"allowed: {sorted(LIVE_PARAMS)}")
    staged = {}
    for key, raw in body.items():
        name, cast, lo, hi = LIVE_PARAMS[key]
        try:
            value = cast(raw)
        except (TypeError, ValueError):
            raise ValueError(f"'{key}' must be {cast.__name__}, got {raw!r}") from None
        if not lo <= value <= hi:
            raise ValueError(f"'{key}' must be between {lo} and {hi}, got {value}")
        staged[name] = value
    globals().update(staged)
    return {k: globals()[LIVE_PARAMS[k][0]] for k in body}


class Latest:
    """The newest frame, and a way to wait for one newer than the one you last saw.

    Deliberately not a queue: a queue is how latency accumulates. Clients are told the
    sequence number they received, and simply miss whatever went by while they were busy.
    """

    def __init__(self):
        self._cv = threading.Condition()
        self._jpeg = None
        self._t_in = 0.0
        self._seq = 0
        self.clients = 0

    def put(self, jpeg, t_in=0.0):
        # t_in rides ALONGSIDE the bytes rather than inside them so the resize path does
        # not have to splice a stamp, throw it away in the re-encode, and splice it again.
        with self._cv:
            self._jpeg = jpeg
            self._t_in = t_in
            self._seq += 1
            self._cv.notify_all()

    def get_newer_than(self, seq, timeout=5.0):
        with self._cv:
            if self._seq == seq:
                self._cv.wait(timeout)
            if self._seq == seq:
                return None, seq, 0.0
            return self._jpeg, self._seq, self._t_in


# Two slots on purpose. pump() publishes to RAW and returns to reading immediately; a
# worker thread does the expensive decode/resize/encode and publishes to LATEST. Doing the
# resize inline in pump() throttled the CAPTURE itself — measured 3.2 fps dropping to 2.3 —
# because every cycle waited for a 1080p decode before reading the next frame.
RAW = Latest()
LATEST = Latest()
# The drive branch's own slot. Third reader of RAW, independent of the other two: the whole
# point of the second encode is that the NVR and the drive view stop competing for one.
H264 = Latest()


# WAIT vs WORK, and why both are needed. `t_out - t_in` — the number field_probe reports — is
# NOT the cost of the resize: it is (how long the frame sat in RAW) + (how long the resize
# took). Reading only the sum, a resize that got faster while the queue got longer looks like
# no change at all. That ambiguity is exactly what made the 105 ms hard to attribute, so both
# halves are now reported separately on /health. Percentiles are computed only when /health is
# served — never on the frame path.
_WAIT_MS = collections.deque(maxlen=100)
_WORK_MS = collections.deque(maxlen=100)


def _json_num(value):
    """A number or a bare `null` — never Python's `None`, which is not JSON. The fps_cap field
    right below shipped that bug once and every reader had to special-case it."""
    return b"null" if value is None else (b"%g" % value)


def _p50(samples):
    if not samples:
        return None
    ordered = sorted(samples)
    return round(ordered[len(ordered) // 2], 1)


class AuReader:
    """Frames `multipartmux` output into access units, one per `feed()`-able chunk.

    WHY MULTIPART AND NOT RAW ANNEX-B. The encoder's byte stream carries no lengths, so the
    only way to know an access unit ended is to see the NEXT one begin — a full frame of
    lookahead, ~70 ms at the camera's cadence, on the one path whose entire purpose is being
    fast. `multipartmux` writes an explicit `Content-Length` per frame for ~80 bytes of
    overhead, measured on the robot 2026-09-21 (380857 bytes for the same 90 frames that are
    373657 raw). The lookahead disappears.

    Boundary-agnostic on purpose: it scans for the length header rather than a boundary
    string, so it does not care what `multipartmux` calls its separator. Same reasoning as
    the SOI/EOI scanner in `pump()`.
    """

    # A 480x270 access unit is a few kB. The cap is what keeps a wedged or desynchronised
    # child from growing this buffer without bound — the defect the JPEG scanners in this
    # repo were audited for.
    _MAX_PART = 4 << 20

    def __init__(self):
        self._buf = b""

    def feed(self, chunk):
        """Add bytes; return a list of complete access units (usually 0 or 1)."""
        self._buf += chunk
        out = []
        while True:
            head = self._buf.find(b"Content-Length:")
            if head < 0:
                if len(self._buf) > self._MAX_PART:
                    self._buf = b""         # no header in sight: desynchronised, resync
                break
            eol = self._buf.find(b"\r\n", head)
            if eol < 0:
                break
            try:
                n = int(self._buf[head + 15:eol])
            except ValueError:
                self._buf = self._buf[eol + 2:]
                continue
            body = self._buf.find(b"\r\n\r\n", eol)
            if body < 0:
                break
            start = body + 4
            if n > self._MAX_PART:
                self._buf = self._buf[start:]
                continue
            if len(self._buf) < start + n:
                break                       # the part has not arrived in full yet
            out.append(self._buf[start:start + n])
            self._buf = self._buf[start + n:]
        return out


# ---------------------------------------------------------------------------------------
# THE DRIVE BRANCH OVER UDP — why it exists.
#
# Over TCP one lost packet freezes the WHOLE stream until it is retransmitted, and the
# retransmission waits at least the RTO. MEASURED 2026-09-23 over Starlink (3.4-4% loss, in
# bursts at the 15 s satellite handovers): 10 stalls of 250-916 ms in 150 s on /h264, every
# one of them on the link and none at the source; the socket showed `rto:252`, 4492
# retransmissions, and BBR is not in this kernel. The frames are all-intra, so a frame that
# does not arrive costs ONE frame and nothing else — which is only true if nothing waits for
# it. UDP is what stops the waiting.
#
# MEASURED the same day with this exact datagram size and cadence (7 x 1200 B every 70 ms,
# 60 s): 89.8% of frames arrived whole, 4.4% more were missing exactly one datagram (what one
# XOR parity per group recovers), and the rest were lost in runs of 1-4 frames — at most
# ~280 ms, against up to 916 ms of freeze on TCP.
#
# THE LEASE. A client opens `GET /h264?udp=PORT` over TCP and keeps it open; while it is open
# this process sends the access units as datagrams to THE TCP PEER'S OWN ADDRESS on that port.
# Three properties follow from that one choice:
#   * no reflection: the destination is an address that completed a TCP handshake with us,
#     so this cannot be pointed at a third party. What it receives is exactly what it could
#     already read from /h264.
#   * no viewer, no bytes: the lease closing — or TCP_USER_TIMEOUT firing when the peer
#     vanishes without a FIN — stops the datagrams, same as the TCP branch.
#   * a health channel for free: one heartbeat line a second carries how many frames were
#     sent, so the far end can tell "UDP is blocked" from "the encoder is idle".
#
# WIRE FORMAT, little-endian, one header per datagram (UDP_HEADER, 32 bytes):
#   magic "AV" | version u8 | kind u8 (0 data, 1 parity) | group u8 | pad | payload u16
#   | session u32 | frame u32 | index u16 | count u16 | au_len u32 | capture f64
# `index` is the fragment number for data and the group number for parity; `count` is always
# the number of DATA fragments; `payload` is the sender's fragment size, so the receiver can
# size a recovered fragment without guessing.
#
# SECOND COPY, ON PURPOSE: the receiver is `h264_relay.UdpReassembler` in
# unitree_ros2/robot_camera_bridge. The network boundary forbids sharing a module, so both
# tests assert the SAME golden datagrams (`tests/test_h264_udp.py` on each side). If you
# change a field here, change it there and update both vectors.
# ---------------------------------------------------------------------------------------
UDP_MAGIC = b"AV"
UDP_VERSION = 1
UDP_HEADER = struct.Struct("<2sBBBxHIIHHId")
# 1200 B of payload + 32 of header + 28 of IP/UDP = 1260, under the path's 1408 (the TCP MSS
# measured on this link is 1368, so there is a tunnel in the way). Fragmenting at the IP layer
# instead would turn ONE lost fragment into a lost datagram with no way to recover it.
UDP_PAYLOAD = 1200
# One parity per 8 data fragments: +12.5% bytes. A 480x270 frame at QP 40 is ~7.6 kB, i.e.
# 7 fragments, so in practice one parity per frame — which is what was measured above.
UDP_GROUP = 8
# Bounded input: nothing this encoder produces is near it, and a frame this size would be
# 870 datagrams of which one lost kills the lot. Refuse rather than flood the link.
UDP_MAX_AU = 1 << 20
UDP_KIND_DATA = 0
UDP_KIND_PARITY = 1
# socket.TCP_USER_TIMEOUT exists on Linux from Python 3.6, but not on every build; 18 is the
# Linux value, and this file only ever runs on Linux.
_TCP_USER_TIMEOUT = getattr(socket, "TCP_USER_TIMEOUT", 18)
# Read by /health. Plain ints mutated from lease threads: a lost increment under the GIL would
# skew a diagnostic counter, never a frame.
_udp_stats = {"leases": 0, "send_drops": 0}


def _xor(chunks, size):
    """XOR of `chunks`, each zero-padded to `size`.

    Through Python ints, not a per-byte loop: that loop is ~8000 interpreter steps a frame on
    the Jetson, while int.from_bytes does the same work in C. Little-endian, so zero padding
    at the END of a chunk is just high-order zeros.
    """
    acc = 0
    for c in chunks:
        acc ^= int.from_bytes(c, "little")
    return acc.to_bytes(size, "little")


def udp_packets(session, frame, au, capture, payload=UDP_PAYLOAD, group=UDP_GROUP):
    """Split one access unit into datagrams: the data fragments, then one parity per group.

    Returns [] for an empty or oversized access unit — nothing is sent rather than something
    the receiver cannot use. Data goes first so a receiver can deliver the frame the moment
    the last data fragment lands, without waiting for parity it does not need.
    """
    if not au or len(au) > UDP_MAX_AU:
        return []
    frags = [au[i:i + payload] for i in range(0, len(au), payload)]
    count = len(frags)

    def head(kind, index):
        return UDP_HEADER.pack(UDP_MAGIC, UDP_VERSION, kind, group, payload,
                               session & 0xFFFFFFFF, frame & 0xFFFFFFFF, index, count,
                               len(au), capture)

    out = [head(UDP_KIND_DATA, i) + f for i, f in enumerate(frags)]
    for g in range(0, count, group):
        members = frags[g:g + group]
        out.append(head(UDP_KIND_PARITY, g // group)
                   + _xor(members, max(len(m) for m in members)))
    return out


def udp_lease_port(query):
    """The UDP port asked for in `/h264?udp=PORT`, or None when absent or not acceptable.

    Fails SAFE: anything that is not a plain integer in 1024-65535 means "no UDP", and the
    caller refuses the request instead of guessing a port.
    """
    values = parse_qs(query, keep_blank_values=True).get("udp")
    # isascii() too: str.isdigit() is true for "²", which int() then rejects.
    if not values or len(values) != 1 or not (values[0].isascii() and values[0].isdigit()):
        return None
    port = int(values[0])
    return port if 1024 <= port <= 65535 else None


class H264Encoder:
    """The second encode: all-intra H.264 at the live view's size, for the DRIVE branch.

    WHY A SECOND ENCODE AT ALL, and it is the operator's idea (PLAN-VIDEO.md 6.g C): one
    stream cannot serve two consumers with opposite needs. The NVR wants resolution and does
    not mind waiting; the drive view wants to arrive now and does not need 1080p. Today that
    tension is resolved by keeping MJPEG alive as the only branch without buffers.

    MEASURED 2026-09-21, same original frame, same hardware resize, against the uncompressed
    original: at INDISTINGUISHABLE quality (PSNR 29.34 vs 29.30 dB) an all-intra H.264 frame
    costs **4151 bytes against the JPEG's 8663** — half. At equal size it is ~1 dB better.

    ALL-INTRA, not a normal GOP, and that is the point: every frame stands alone, so a lost
    packet costs ONE FRAME instead of freezing until the next IDR. That is the property that
    lets this ride a transport with no retransmission and no jitter buffer — the same
    resilience MJPEG has, at half the bytes. A normal GOP is three times cheaper still, and
    that is what the NVR branch already uses; it is the wrong trade here.

    Mirrors `_HwResizer`: a persistent gst-launch child, LOCKSTEP, one frame in flight.
    Write one JPEG, read one access unit, never queue.
    """

    _MAX_FAILS = 3
    _FIRST_TIMEOUT_MS = 2500

    def __init__(self):
        self._proc = None
        self._key = None            # (width, height, qp, src_w, src_h) the child was built for
        self._reader = AuReader()
        self._lock = threading.Lock()
        self._fails = 0
        self._dead = False
        self._first = True

    def _spawn(self, width, height, qp, src_w, src_h):
        caps = f"video/x-raw(memory:NVMM),width={width},height={height}"
        cmd = ["gst-launch-1.0", "-q",
               "fdsrc", "fd=0", "!", "jpegparse", "!", "nvjpegdec", "!", "nvvidconv", "!",
               caps, "!",
               "nvv4l2h264enc",
               # Every frame a keyframe. idrinterval AND iframeinterval: the first makes them
               # IDR (a decoder may start on any one), the second makes them intra at all.
               "iframeinterval=1", "idrinterval=1",
               # B-frames break WebRTC and buy nothing here; pinned, not trusted to default.
               "num-B-Frames=0",
               # Fixed QP instead of a bitrate target: the size then follows the SCENE, which
               # is what makes the comparison against JPEG meaningful and what keeps quality
               # steady when the picture gets busy. ratecontrol-enable=0 + preset-level=0 are
               # what the property's own documentation requires for quant-i-frames to apply.
               "ratecontrol-enable=0", "preset-level=0", f"quant-i-frames={qp}",
               "insert-sps-pps=1", "!",
               # config-interval=-1 so SPS/PPS ride with every frame: a viewer joining
               # mid-stream otherwise never gets the parameter sets and decodes nothing.
               "h264parse", "config-interval=-1", "!",
               "multipartmux", "!", "fdsink", "fd=1"]
        proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, bufsize=0)
        threading.Thread(target=drain_stderr, args=(proc, "h264"), daemon=True).start()
        self._proc, self._key, self._first = proc, (width, height, qp, src_w, src_h), True
        self._reader = AuReader()
        return proc

    def encode(self, jpeg, width, height, qp):
        """One JPEG in, one access unit out (or None if the hardware path is unusable)."""
        if self._dead:
            return None
        src_w, src_h = _jpeg_size(jpeg)
        if not src_w:
            return None
        with self._lock:
            try:
                key = (width, height, qp, src_w, src_h)
                if self._proc is None or self._key != key or self._proc.poll() is not None:
                    if self._proc is not None:
                        stop_child(self._proc, "h264")
                    self._spawn(*key)
                proc = self._proc
                proc.stdin.write(jpeg)
                proc.stdin.flush()
                budget = self._FIRST_TIMEOUT_MS if self._first else 800
                deadline = time.monotonic() + budget / 1000.0
                while time.monotonic() < deadline:
                    if not select.select([proc.stdout], [], [], 0.05)[0]:
                        continue
                    parts = self._reader.feed(os.read(proc.stdout.fileno(), 65536))
                    if parts:
                        self._first = False
                        self._fails = 0
                        return parts[-1]        # newest, never a backlog
                raise TimeoutError("the H.264 child produced no access unit in time")
            except Exception as exc:
                self._fails += 1
                log(f"h264 branch failed ({self._fails}/{self._MAX_FAILS}): {exc}")
                if self._proc is not None:
                    stop_child(self._proc, "h264")
                    self._proc = None
                if self._fails >= self._MAX_FAILS:
                    # Three in a row is a broken machine, not a hiccup, and retrying forever
                    # would put a multi-hundred-ms timeout in front of every frame.
                    self._dead = True
                    log("h264 branch disabled for the life of this process")
                return None


_H264 = H264Encoder()


def h264_worker():
    """Encode the newest frame to all-intra H.264, forever.

    Skipping intermediate frames is correct and deliberate — the drive view wants the latest
    picture, never a backlog. Its own gate, independent of the NVR's: the two branches answer
    to different consumers and must not be able to starve one another.
    """
    seq = -1
    gate = RateGate()
    while True:
        jpeg, seq, t_in = RAW.get_newer_than(seq, timeout=5.0)
        if jpeg is None or not H264_ENABLE:
            continue
        if not gate.allows(time.monotonic(), (1.0 / H264_FPS) if H264_FPS > 0 else 0.0):
            continue
        au = _H264.encode(jpeg, H264_WIDTH, H264_HEIGHT, H264_QP)
        if au:
            H264.put(au, t_in)


def resizer():
    """Shrink the newest frame, forever. Skipping intermediate frames is correct: a viewer
    wants the latest picture, not a backlog of stale ones."""
    seq = -1
    while True:
        jpeg, seq, t_in = RAW.get_newer_than(seq, timeout=5.0)
        if jpeg is not None:
            got = time.time()
            small = _shrink(jpeg)
            done = time.time()
            if t_in:
                _WAIT_MS.append((got - t_in) * 1000.0)
                _WORK_MS.append((done - got) * 1000.0)
            # Stamp AFTER the resize: _shrink re-encodes, so anything spliced in before
            # would be dropped. t_out is therefore the true "ready for viewers" instant,
            # and t_out - t_in is exactly what the resize costs.
            #
            # The COM payload stays EXACTLY two floats. Three readers parse it with
            # `a, b = body.split()` (read_stamp here, tests/_latency_probe.py, video-bench's
            # field_probe.py) and a third number makes all of them return None silently — the
            # frames would simply count as "unstamped". New metrics go to /health, never here.
            LATEST.put(stamp(small, t_in, done) if STAMP else small)


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "robot-mjpeg"

    def log_message(self, *a):
        pass

    def do_GET(self):
        path, _, query = self.path.partition("?")
        path = path.rstrip("/") or "/"
        if path in ("/", "/stream"):
            return self._stream()
        if path == "/h264":
            if "udp" in parse_qs(query, keep_blank_values=True):
                return self._h264_udp_lease(udp_lease_port(query))
            return self._h264_stream()
        if path == "/snapshot":
            return self._snapshot()
        if path == "/health":
            # `now` is the robot's wall clock at the instant this answer was built. A
            # caller that records its own clock before and after the request gets the
            # offset between the two machines the way SNTP does — offset =
            # now - (t_before + t_after) / 2 — accurate to about half the RTT. Without it
            # the stamps in the frames cannot be compared against an off-robot clock at
            # all, and the two machines are not NTP-locked to each other.
            body = (
                # `null`, not `none`: with no cap this used to emit a bare Python None,
                # which is not JSON — so /health was unparseable in exactly the DEFAULT
                # configuration (MJPEG_FPS=0), and every reader had to special-case it.
                b'{"ok":true,"clients":%d,"fps_cap":%s,"width":%d,"quality":%d,'
                b'"nvr_queue":%d,"nvr_dropped":%d,"stamp":%s,'
                # wait vs work: see the comment above resizer(). Both null until the first
                # frame goes through a resize, which is correct — at WIDTH=0 there is none.
                b'"resize_path":"%s","wait_ms_p50":%s,"work_ms_p50":%s,"hw_fallbacks":%d,'
                b'"h264":%s,"h264_qp":%d,"h264_size":"%dx%d","h264_clients":%d,'
                b'"h264_udp_leases":%d,"h264_udp_send_drops":%d,'
                b'"now":%.6f}'
                % (LATEST.clients, (b"%g" % FPS) if FPS > 0 else b"null", WIDTH, QUALITY,
                   len(_nvr), _nvr_dropped, b"true" if STAMP else b"false",
                   (_HW.path if WIDTH > 0 else "none").encode(),
                   _json_num(_p50(_WAIT_MS)), _json_num(_p50(_WORK_MS)), _HW.fallbacks,
                   b"true" if H264_ENABLE else b"false", H264_QP,
                   H264_WIDTH, H264_HEIGHT, H264.clients,
                   _udp_stats["leases"], _udp_stats["send_drops"],
                   time.time())
            )
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404)

    def do_POST(self):
        """POST /config {fps?, width?, quality?} — retune the live view without a restart.

        LOCALHOST ONLY, and that is the security design, not a convenience. This port binds
        0.0.0.0 with no authentication (known finding P0-1), so a WRITE route reachable from
        the network would be a straight downgrade. The relay is the authenticated surface:
        it validates the request against its allowlist and then calls this from 127.0.0.1.
        Nothing else can reach it, so no second token has to exist.
        """
        if self.path.split("?")[0].rstrip("/") != "/config":
            return self.send_error(404)
        if self.client_address[0] not in ("127.0.0.1", "::1"):
            log(f"rejected /config from {self.client_address[0]} (localhost only)")
            return self._json(403, {"ok": False, "error": "localhost only"})
        try:
            n = int(self.headers.get("Content-Length", "0"))
            body = json.loads(self.rfile.read(n) or b"{}")
        except (ValueError, json.JSONDecodeError):
            return self._json(400, {"ok": False, "error": "bad json"})
        try:
            applied = set_live_params(body)
        except ValueError as exc:
            return self._json(400, {"ok": False, "error": str(exc)})
        log(f"live config: {applied}")
        return self._json(200, {"ok": True, **live_params()})

    def _json(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _snapshot(self):
        jpeg, _, _ = LATEST.get_newer_than(-1, timeout=3.0)
        if not jpeg:
            return self.send_error(503, "no frame yet")
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(jpeg)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(jpeg)

    def _h264_stream(self):
        """The drive branch: all-intra H.264 access units, one per multipart part.

        SAME SHAPE AS THE MJPEG STREAM ON PURPOSE. The consumer already exists for that
        framing (the camera bridge reads it, and a browser can), so the only thing that
        changes downstream is what is inside each part — which is the whole point: half the
        bytes for the same picture, with every frame still independent.

        Each part carries `X-Capture`, the robot clock at the instant the frame came off the
        camera. The MJPEG branch splices the same number into a JPEG COM segment; H.264 has no
        such free-form field that survives, so it rides in the part header instead. That is
        what lets the far end measure true capture-to-glass latency without a shared clock
        assumption — the method every latency number in PLAN-VIDEO.md rests on.
        """
        if not H264_ENABLE:
            return self._json(503, {"ok": False,
                                    "error": 'h264 branch is off; POST /config {"h264":1}'})
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type",
                         f"multipart/x-mixed-replace; boundary={BOUNDARY}")
        self.end_headers()
        H264.clients += 1
        seq = -1
        gate = RateGate()
        try:
            while True:
                au, seq, t_in = H264.get_newer_than(seq, timeout=10.0)
                if au is None:
                    continue                      # no new frame yet; keep the socket open
                # Per-viewer cap, re-read every frame like the MJPEG one, and with the same
                # gate: a cap that cannot deliver the rate it is given is worse than none.
                if not gate.allows(time.monotonic(),
                                   (1.0 / H264_FPS) if H264_FPS > 0 else 0.0):
                    continue
                self.wfile.write(
                    b"--" + BOUNDARY.encode() + b"\r\n"
                    b"Content-Type: video/x-h264\r\n"
                    b"X-Capture: " + (b"%.6f" % t_in) + b"\r\n"
                    b"Content-Length: " + str(len(au)).encode() + b"\r\n\r\n"
                    + au + b"\r\n")
        except (BrokenPipeError, ConnectionResetError, socket.timeout):
            pass                                   # viewer went away; normal
        finally:
            H264.clients -= 1

    def _h264_udp_lease(self, port):
        """The drive branch over UDP, for as long as this TCP request stays open.

        See the block above `udp_packets()` for why, and for the wire format. The TCP body is
        a heartbeat: one line a second with the number of frames sent so far, which is how
        the far end tells a blocked UDP path (count rising, nothing arriving) from an idle
        encoder (count flat). It is also what makes a vanished peer END the lease: the write
        fails, or TCP_USER_TIMEOUT aborts it, and the datagrams stop.
        """
        if port is None:
            return self._json(400, {"ok": False,
                                    "error": "udp must be a port number in 1024-65535"})
        if not H264_ENABLE:
            return self._json(503, {"ok": False,
                                    "error": 'h264 branch is off; POST /config {"h264":1}'})
        peer = self.client_address[0]
        session = struct.unpack("<I", os.urandom(4))[0]
        try:
            udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except OSError as exc:
            return self._json(500, {"ok": False, "error": f"udp socket: {exc}"})
        try:
            # connect() fixes the destination once, and lets an ICMP "port unreachable" come
            # back as an error on a later send instead of vanishing.
            udp.connect((peer, port))
            # Never block the frame loop on the NIC: a full send buffer drops the rest of that
            # frame (counted), exactly like a slow HTTP viewer skips frames.
            udp.setblocking(False)
            # A peer that disappears without a FIN would otherwise keep this lease — and the
            # datagrams — alive until tcp_retries2 gives up, ~15 minutes. Ten seconds of
            # unacknowledged heartbeat is plenty: the worst loss burst measured over Starlink
            # froze TCP for under one.
            self.connection.setsockopt(socket.IPPROTO_TCP, _TCP_USER_TIMEOUT, 10000)
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Connection", "close")
            self.send_header("X-Udp-Session", str(session))
            self.send_header("X-Udp-Payload", str(UDP_PAYLOAD))
            self.end_headers()
            self.close_connection = True
            log(f"h264 udp lease: {peer}:{port} session {session}")
            _udp_stats["leases"] += 1
            H264.clients += 1
            try:
                self._udp_loop(udp, session)
            finally:
                H264.clients -= 1
                _udp_stats["leases"] -= 1
                log(f"h264 udp lease ended: {peer}:{port}")
        except OSError:
            # The lease holder went away — a reset, a broken pipe, or TCP_USER_TIMEOUT
            # (ETIMEDOUT) after it vanished without a FIN. All of them mean the same: stop.
            pass
        finally:
            udp.close()

    def _udp_loop(self, udp, session):
        seq = -1
        frame = 0
        gate = RateGate()
        next_beat = 0.0
        while True:
            now = time.monotonic()
            if now >= next_beat:
                self.wfile.write(b"%d\n" % frame)
                self.wfile.flush()
                next_beat = now + 1.0
            au, seq, t_in = H264.get_newer_than(seq, timeout=1.0)
            if au is None or not H264_ENABLE:
                continue
            if not gate.allows(time.monotonic(),
                               (1.0 / H264_FPS) if H264_FPS > 0 else 0.0):
                continue
            frame += 1
            for datagram in udp_packets(session, frame, au, t_in):
                try:
                    udp.send(datagram)
                except OSError:
                    # The rest of THIS frame is dropped, not queued: a frame missing its tail
                    # is useless, and holding it would make the next one late. That covers a
                    # full send buffer (BlockingIOError), an ICMP refusal from an earlier send
                    # and a route that blinked. Whether the lease lives is the TCP side's call.
                    _udp_stats["send_drops"] += 1
                    break

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
        gate = RateGate()
        try:
            while True:
                jpeg, seq, _ = LATEST.get_newer_than(seq, timeout=10.0)
                if jpeg is None:
                    continue                      # no new frame yet; keep the socket open
                # Re-read the cap EVERY frame. It used to be computed once when the client
                # connected, which meant a live change reached nobody: the camera bridge
                # holds one connection open for hours, so the operator would move the
                # control and the stream it actually feeds would never notice.
                if not gate.allows(time.monotonic(),
                                   (1.0 / FPS) if FPS > 0 else 0.0):
                    continue                      # honour the cap by DROPPING, not delaying
                self.wfile.write(
                    b"--" + BOUNDARY.encode() + b"\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
                    + jpeg + b"\r\n")
        except (BrokenPipeError, ConnectionResetError, socket.timeout):
            pass                                   # viewer went away; normal
        finally:
            LATEST.clients -= 1


class RateGate:
    """A frames-per-second cap that delivers the rate it was ASKED for.

    THE DEFECT THIS REPLACES, measured on the robot 2026-09-16: the old gate compared against
    the arrival time of the last frame it accepted —

        if now - last < min_gap: continue
        last = now

    — which can only ever deliver `source / k` for whole k, because frames arrive at the
    camera's cadence and nothing lands exactly on the deadline. With the camera at 14.3 fps
    (70 ms apart) and a cap of 10 (100 ms), the frame at 70 ms is "early" and dropped, the
    next lands at 140, and the stream settles at **7.13 fps** — measured, against a cap that
    said 10. The only reachable rates were 14.3, 7.15, 4.77, 3.58…, so EVERY cap between 7.2
    and 14.2 delivered exactly 7.15.

    THE FIX is to advance a deadline by exactly one period per frame SENT, rather than
    anchoring it on when a frame happened to arrive. The deadline then carries the remainder
    forward, so the gate can take two frames out of three and average the rate it was given.

    The clamp is the other half and it is not decoration: if the source stalls, the deadline
    falls into the past, and without the clamp the gate would pass every frame back-to-back
    until it caught up — a burst of stale pictures on the view the operator steers by, which
    is exactly the failure `Latest` exists to prevent one layer up.

    Per viewer, because the cap is per viewer: `self` holds one deadline and each HTTP client
    gets its own instance.
    """

    __slots__ = ("_due",)

    def __init__(self):
        self._due = 0.0     # monotonic deadline; 0.0 = not seeded yet

    def allows(self, now, min_gap):
        """True if a frame arriving at `now` may be sent under a cap of `min_gap` seconds."""
        if min_gap <= 0:
            self._due = 0.0     # no cap; re-seed if one is turned back on
            return True
        if not self._due:
            self._due = now     # first frame of this viewer sets the phase
        if now < self._due:
            return False
        self._due += min_gap
        if self._due < now:
            # The source stalled and the deadline fell into the past. Resync a FULL period
            # ahead, not to `now`: setting it to `now` lets the very next frame through
            # 70 ms later, which is the burst this clamp exists to stop. Caught by
            # test_a_stall_does_not_release_a_burst, which failed on exactly that.
            self._due = now + min_gap
        return True


PUBLISH = None      # set in main(): RAW.put when resizing, LATEST.put when not

# The NVR branch gets a DELIBERATE, REGULAR rate — not "whatever is left over".
#
# Dropping the oldest frame whenever the queue filled did keep the capture free, but it fed
# the H.264 encoder an irregular 27% of frames: with fdsrc do-timestamp=true the timestamps
# then jump around, and the FLV/RTMP stream that comes out is one Frigate cannot even start
# on ("no frames have been received"). A DECODER tolerates dropped frames; an ENCODER needs a
# cadence. So pick frames on a fixed interval instead, and keep the bounded queue only as a
# safety net for a genuine stall.
# Set NVR_ENABLE=0 to stop feeding the recording branch entirely. Exists for diagnosis: the
# two branches share one uplink, so turning this off is the one-step way to tell whether a
# live-view stall is caused by the recorder competing for bandwidth.
NVR_ENABLE = os.environ.get("NVR_ENABLE", "1") != "0"
NVR_FPS = float(os.environ.get("NVR_FPS", "5"))
NVR_MAX = int(os.environ.get("NVR_QUEUE", "8"))
_nvr_gate = RateGate()
_nvr = collections.deque()
_nvr_cv = threading.Condition()
_nvr_dropped = 0


def nvr_offer(frame):
    """Offer a frame to the recording branch at a steady rate. Never blocks the caller.

    Uses the SAME gate as the live view, and it has to: this one used to compare against the
    arrival time of the last accepted frame, which costs whole frames as soon as the cap sits
    anywhere near the source rate.

    MEASURED 2026-09-21, and this one hid behind the link for days. `NVR_FPS=15` puts the
    minimum gap at 66.7 ms against a camera delivering every 70 ms — close enough that normal
    cadence jitter pushes frames under the threshold, each costing the one after it too. The
    camera fed 14.3 fps, the queue dropped NOTHING, SRT reported ZERO loss, and 8.7 fps came
    out the far end. With `NVR_FPS=20` (a 50 ms gap, comfortably clear of 70) the same
    pipeline delivered 14.2.

    That is also why this variable is treacherous: it is BOTH the gate's ceiling, which wants
    to sit well above the source rate, AND the divisor that pre-computes the encoder's
    per-frame bitrate (see run-video.sh), which wants to BE the source rate. Two jobs, opposite
    optima. The deadline-based gate removes the conflict: a cap of 15 against a 14.3 fps
    source now passes every frame, so the divisor can be honest.
    """
    global _nvr_dropped
    if not NVR_ENABLE:
        return
    if not _nvr_gate.allows(time.monotonic(), (1.0 / NVR_FPS) if NVR_FPS > 0 else 0.0):
        return                          # not this one: keeps the cadence regular
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
            # t_in is read here, the earliest instant this process can see the frame —
            # go2_jpeg_stream's GetImageSample has already returned and the pipe is
            # effectively free, so it doubles as "when the robot handed us the frame".
            # t_in is taken unconditionally, ~50 ns: STAMP controls what is spliced INTO
            # the frame, but /health's wait/work split needs the timestamp either way.
            PUBLISH(frame, time.time())
            nvr_offer(frame)   # the NVR always gets the original, unstamped bytes
            frames += 1
            if frames % 300 == 0:
                dt = time.monotonic() - t0
                log(f"{frames} frames, {frames / dt:.1f} fps in, "
                    f"{LATEST.clients} viewer(s), {_nvr_dropped} dropped to NVR")


def main():
    # _shrink crosses the GIL boundary on every cv2 call, and each crossing can wait a full
    # switch interval for pump() — which is pure Python and holds it — to let go. The 5 ms
    # default is a lot next to a 70 ms frame; 1 ms costs more context switches than anyone
    # here will notice. Harmless when the hardware path is on, and it is the fallback that
    # runs on a bad day.
    sys.setswitchinterval(0.001)
    global PUBLISH
    # ALWAYS go through the worker, even when nothing is being resized.
    #
    # This used to branch on WIDTH at startup: with WIDTH=0 frames went straight to LATEST
    # and no worker existed, so raising WIDTH at runtime resized nothing and the control
    # looked broken. The worker decides per frame instead, which costs one condition-variable
    # hand-off (microseconds, against a 41 ms transport) and makes the knob actually work.
    PUBLISH = RAW.put
    threading.Thread(target=resizer, name="resizer", daemon=True).start()
    threading.Thread(target=h264_worker, name="h264", daemon=True).start()

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
