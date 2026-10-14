#!/usr/bin/env python3
"""
camera_source — the ONE picture the robot sends, chosen at runtime.

    camera_source.py ./videohub_jpeg_stream eth0 15 | mjpeg_server.py | gst-launch-1.0 ...

Writes a concatenated JPEG stream to stdout, exactly like videohub_jpeg_stream does, so
everything downstream (live MJPEG, the H.264 drive branch, the NVR) is unchanged and simply
carries whichever camera is selected:

    front     the robot's own camera, through the command given on the command line
              (videohub_jpeg_stream). What has always run, and the default.
    rs_color  the RealSense D435i's colour sensor, 1280x720, JPEG-encoded in hardware.
    rs_depth  the RealSense's depth, colourised: near is red, far is blue, no reading is
              black. A legend with the range in metres is drawn into the picture itself, so
              it survives every branch and every viewer without any client knowing about it.

ONLY ONE CAMERA IS EVER OPEN. Switching closes the previous one first — the videohub child is
killed, the RealSense pipeline is stopped — so the link carries one picture and the robot pays
for one capture. That is the point of the switch, not a side effect: the uplink is a shared
LTE SIM and two streams would roughly double it (see robot-splunk-docs/MEDICIONES.md).

Control: GET/POST http://127.0.0.1:$CAMERA_PORT/camera, LOCALHOST ONLY, for the same reason
mjpeg_server's /config is — the relay is the authenticated surface and calls this from the
robot itself. Nothing on the network can reach it.

TWO READ PATHS, ONE PER STREAM, measured on the Go2 2026-10-14:

    depth   librealsense, through ctypes. The D435i exposes depth as /dev/video0 in Z16, and on
            this Jetson neither OpenCV 4.2 (fails silently on open) nor GStreamer (does not
            even list the node) reads that format. librealsense 2.50 is already installed with
            ros-noetic-librealsense2, so nothing has to be installed on the robot.
            Cost: 32% of a core at 15.3 fps, nearly all of it the colourise and the JPEG.
    colour  GStreamer: v4l2src (YUY2) -> jpegenc, straight from the sensor's own format.

EVERY JPEG HERE IS 4:2:2, because the front camera's are, and that is not a detail. The
downstream decoders (nvjpegdec in the NVR encoder and in the drive branch) are long-lived and
renegotiate when the input changes; a change of SIZE they absorb, a change of CHROMA
SUBSAMPLING they do not. MEASURED on the Go2 2026-10-14 with the real front JPEG: front (4:2:2)
-> RealSense at 4:2:0 -> front again gave wrong colours on the RealSense part and a solid green
picture from then on — in the recording, until the encoder restarted. The same sequence with
the RealSense at 4:2:2: every frame right. So the colour path uses software jpegenc fed YUY2
(4:2:2 in, 4:2:2 out) and NOT the NVJPG engine, which only encodes 4:2:0; and the depth path
encodes with PIL, because OpenCV 4.2 cannot choose the subsampling at all.

AND ONE TRAP THAT IS NOT ABOUT CAMERAS: on this Jetson, `import numpy` ALONE costs 367% of a
core, forever. The system BLAS is OpenBLAS-pthread, and its worker threads spin on sched_yield
whether or not anything calls them. It looked like the cameras were expensive — three hot
threads, mostly kernel time — until the same three threads showed up in a process that only
imported numpy and slept. OPENBLAS_NUM_THREADS=1 takes it to 0%; nothing here does linear
algebra, so the pool buys nothing. run-video.sh exports it for every process it starts, and
_imaging() sets it too, for a manual run.

Standard library only for `front` and `rs_color`. numpy, cv2 and PIL are imported the first
time the depth camera is opened, so a robot without them still runs everything else.
"""
from __future__ import annotations

import ctypes
import io
import json
import os
import select
import signal
import struct
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

CAMERAS = ("front", "rs_color", "rs_depth")

PORT = int(os.environ.get("CAMERA_PORT", "8094"))
INITIAL = os.environ.get("CAMERA", "front")
# The RealSense's own rate, NOT the robot camera's MAXFPS. It used to follow MAXFPS, and the Go2
# runs MAXFPS=0 ("as fast as the videohub answers", which tops out at ~14), so the RealSense
# went to 30 fps — MEASURED 2026-10-14: the drive branch sends every frame, and depth at 30 fps
# put 4.5 Mbps on an LTE uplink with ~1 Mbps to spare. 15 matches the front camera. The sensor
# only offers 6, 15 and 30 here, so the value is rounded UP to one of those.
RS_FPS = float(os.environ.get("RS_FPS", "15") or 0)
LIBRS = os.environ.get("LIBREALSENSE",
                       "/opt/ros/noetic/lib/aarch64-linux-gnu/librealsense2.so")
RS_COLOR_SIZE = os.environ.get("RS_COLOR_SIZE", "1280x720")
# 848x480 is the D435's optimal depth resolution: the most valid pixels per frame. That is what
# is CAPTURED; what is SENT is DEPTH_OUT_SIZE, see smooth_depth().
RS_DEPTH_SIZE = os.environ.get("RS_DEPTH_SIZE", "848x480")
DEPTH_OUT_SIZE = os.environ.get("DEPTH_OUT_SIZE", "424x240")
RS_QUALITY = int(os.environ.get("RS_JPEG_QUALITY", "85"))
# OpenCV's thread pool is sized for the machine, not for this process's share of it. Same rule
# and same default as mjpeg_server's MJPEG_CV_THREADS. Used by the depth colouriser only.
CV_THREADS = int(os.environ.get("CAMERA_CV_THREADS", "1"))
# The colour scale, in millimetres. Beyond ~4 m the D435's error grows with the square of the
# distance, so a wider scale spends most of the colours on noise.
DEPTH_MIN_MM = int(os.environ.get("DEPTH_MIN_MM", "300"))
DEPTH_MAX_MM = int(os.environ.get("DEPTH_MAX_MM", "4000"))
# How long a POST waits for the switch to land before answering. Opening the RealSense takes
# about a second; the answer must still fit inside the relay's and the executor's timeouts.
SWITCH_WAIT_S = float(os.environ.get("CAMERA_SWITCH_WAIT_S", "3"))
# The front camera's command exits on its own after ~8 s without frames (the robot's camera
# service is down). It is restarted here rather than ending the pipeline, so the camera
# selected at runtime survives it.
FRONT_RESTART_S = 3.0
# A JPEG stream that never shows an end-of-image marker would otherwise grow this buffer
# without bound. Several 1080p frames of headroom; past it the bytes are garbage anyway.
MAX_BUF = 8 * 1024 * 1024

SOI = b"\xff\xd8"
EOI = b"\xff\xd9"


def log(msg):
    sys.stderr.write(f"[camera_source] {msg}\n")
    sys.stderr.flush()


def parse_size(text):
    w, _, h = text.lower().partition("x")
    return int(w), int(h)


def rs_fps(maxfps):
    """The RealSense rate to ask for: the smallest supported one that is >= the cap."""
    if maxfps <= 0 or maxfps > 15:
        return 30
    return 15 if maxfps > 6 else 6


def take_frames(buf):
    """Split complete JPEGs off the front of `buf`. Returns (frames, remainder).

    Frames are found by SOI/EOI, the same way mjpeg_server.pump() finds them — that is what
    lets a switch happen BETWEEN frames instead of handing downstream half of one."""
    frames = []
    while True:
        start = buf.find(SOI)
        if start < 0:
            return frames, buf[-1:]
        end = buf.find(EOI, start + 2)
        if end < 0:
            return frames, buf[start:]
        frames.append(buf[start:end + 2])
        buf = buf[end + 2:]


class CameraError(Exception):
    pass


# --------------------------------------------------------------------------- #
# front and rs_color — a child process that prints JPEGs
# --------------------------------------------------------------------------- #
class CommandCamera:
    """A camera that is a command writing concatenated JPEGs to stdout.

    `restart`: the front camera's command exits on its own when the robot's camera service
    stops answering, and is brought back here. A RealSense pipeline that exits has lost its
    device instead, so it raises CameraError and the caller falls back to `front`.

    `first_frame_s`: wait this long for a first frame before calling the open a success, so a
    camera that cannot start is reported by the switch itself instead of as a black screen.
    """

    def __init__(self, cmd, label, restart=True, first_frame_s=0.0):
        if not cmd:
            raise CameraError(f"no command for the {label} camera")
        self._cmd = cmd
        self._label = label
        self._restart = restart
        self._proc = None
        self._buf = b""
        self._ready = []
        self._next_start = 0.0
        self._spawn()
        if first_frame_s > 0:
            deadline = time.monotonic() + first_frame_s
            while time.monotonic() < deadline:
                frame = self.read(min(0.5, max(deadline - time.monotonic(), 0.01)))
                if frame:
                    self._ready.insert(0, frame)
                    return
            self.close()
            raise CameraError(f"no frame within {first_frame_s:g}s")

    def _spawn(self):
        try:
            # stderr is inherited: its log lines land in the same journal as always.
            self._proc = subprocess.Popen(self._cmd, stdout=subprocess.PIPE)
        except OSError as exc:
            raise CameraError(f"cannot start {self._cmd[0]}: {exc}") from None
        self._buf = b""

    def read(self, timeout):
        if self._ready:
            return self._ready.pop(0)
        if self._proc is None or self._proc.poll() is not None:
            if self._proc is not None:
                rc = self._proc.returncode
                if not self._restart:
                    raise CameraError(f"the {self._label} pipeline exited (rc={rc})")
                log(f"{self._label} camera exited (rc={rc}); "
                    f"restarting in {FRONT_RESTART_S:.0f}s")
                self._proc = None
                self._next_start = time.monotonic() + FRONT_RESTART_S
            if time.monotonic() < self._next_start:
                time.sleep(min(timeout, 0.2))
                return None
            self._spawn()
        fd = self._proc.stdout.fileno()
        if not select.select([fd], [], [], timeout)[0]:
            return None
        chunk = os.read(fd, 65536)
        if not chunk:
            return None             # EOF: poll() notices the exit on the next call
        self._buf += chunk
        # SOI/EOI framing also discards anything that is not a JPEG — gst-launch on L4T can
        # print its NvMM banner lines to stdout.
        frames, self._buf = take_frames(self._buf)
        if len(self._buf) > MAX_BUF:
            log(f"{self._label} camera: {len(self._buf)} bytes without a complete JPEG; dropped")
            self._buf = b""
        if not frames:
            return None
        # Every frame is kept, in order: the source is already capped by its own max_fps,
        # and dropping here would only hide frames from the gates downstream that count them.
        self._ready = frames[1:]
        return frames[0]

    def close(self):
        if self._proc is None:
            return
        if self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._proc.kill()
                self._proc.wait()
        self._proc.stdout.close()
        self._proc = None


# --------------------------------------------------------------------------- #
# rs_depth — librealsense's C API through ctypes
# --------------------------------------------------------------------------- #
RS2_STREAM_DEPTH = 1
RS2_FORMAT_Z16 = 1
_P = ctypes.c_void_p
_lib = None


def _librs():
    """Load librealsense once, declaring the signatures that are used. CameraError if absent."""
    global _lib
    if _lib is not None:
        return _lib
    try:
        lib = ctypes.CDLL(LIBRS)
    except OSError as exc:
        raise CameraError(f"librealsense not available ({LIBRS}): {exc}") from None
    sigs = {
        # name: (restype, argtypes) — every call takes rs2_error** last
        "rs2_get_api_version": (ctypes.c_int, []),
        "rs2_create_context": (_P, [ctypes.c_int]),
        "rs2_query_devices": (_P, [_P]),
        "rs2_get_device_count": (ctypes.c_int, [_P]),
        "rs2_create_pipeline": (_P, [_P]),
        "rs2_create_config": (_P, []),
        "rs2_config_enable_stream": (None, [_P, ctypes.c_int, ctypes.c_int, ctypes.c_int,
                                            ctypes.c_int, ctypes.c_int, ctypes.c_int]),
        "rs2_pipeline_start_with_config": (_P, [_P, _P]),
        "rs2_pipeline_try_wait_for_frames": (ctypes.c_int, [_P, ctypes.POINTER(_P),
                                                            ctypes.c_uint]),
        "rs2_extract_frame": (_P, [_P, ctypes.c_int]),
        "rs2_get_frame_data": (_P, [_P]),
        "rs2_get_frame_width": (ctypes.c_int, [_P]),
        "rs2_get_frame_height": (ctypes.c_int, [_P]),
        "rs2_get_frame_stride_in_bytes": (ctypes.c_int, [_P]),
        "rs2_pipeline_stop": (None, [_P]),
    }
    for name, (res, args) in sigs.items():
        fn = getattr(lib, name)
        fn.restype = res
        fn.argtypes = [*args, ctypes.POINTER(_P)]
    # No rs2_error** on these.
    for name in ("rs2_release_frame", "rs2_delete_pipeline_profile", "rs2_delete_config",
                 "rs2_delete_pipeline", "rs2_delete_device_list", "rs2_delete_context",
                 "rs2_free_error"):
        getattr(lib, name).argtypes = [_P]
        getattr(lib, name).restype = None
    lib.rs2_get_error_message.argtypes = [_P]
    lib.rs2_get_error_message.restype = ctypes.c_char_p
    _lib = lib
    return lib


def _call(fn, *args):
    """Call a librealsense function and turn its rs2_error into a CameraError."""
    err = _P()
    out = fn(*args, ctypes.byref(err))
    if err.value:
        msg = _lib.rs2_get_error_message(err).decode("utf-8", "replace")
        _lib.rs2_free_error(err)
        raise CameraError(f"librealsense {fn.__name__}: {msg}")
    return out


_np = None
_cv2 = None
_pil = None


def _imaging():
    global _np, _cv2, _pil
    if _cv2 is None:
        # BEFORE the import: OpenBLAS reads it once, when numpy loads it. See the module
        # docstring — without it this process burns ~3.7 cores doing nothing.
        os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
        try:
            import cv2
            import numpy
            from PIL import Image
        except ImportError as exc:
            raise CameraError(f"the depth camera needs numpy, cv2 and PIL: {exc}") from None
        cv2.setNumThreads(CV_THREADS)
        _np, _cv2, _pil = numpy, cv2, Image
    return _np, _cv2


def encode_jpeg_422(bgr):
    """BGR picture -> JPEG with 4:2:2 chroma, the front camera's. See the module docstring for
    why the subsampling must match; cv2.imencode in OpenCV 4.2 always writes 4:2:0."""
    _imaging()
    buf = io.BytesIO()
    _pil.fromarray(bgr[:, :, ::-1]).save(buf, "JPEG", quality=RS_QUALITY, subsampling=1)
    return buf.getvalue()


def jpeg_luma_sampling(jpeg):
    """(h, v) sampling factors of the first component: (2, 1) is 4:2:2, (2, 2) is 4:2:0.
    None if no SOF is found. Used by the tests to hold the contract above."""
    i = 2
    while i + 4 <= len(jpeg) and jpeg[i] == 0xFF:
        marker = jpeg[i + 1]
        if marker in (0xC0, 0xC1, 0xC2):
            return jpeg[i + 11] >> 4, jpeg[i + 11] & 15
        i += 2 + struct.unpack(">H", jpeg[i + 2:i + 4])[0]
    return None


def smooth_depth(z, size=None):
    """Raw depth -> a picture that is cheap to send: smaller, holes filled, speckle removed.

    The raw D435 frame is peppered with single pixels and small blobs of "no reading", and those
    black specks next to saturated colours are the most expensive thing an intra encoder can be
    given. MEASURED on the Go2 2026-10-14, real frames through the real drive encoder (640x360,
    QP 38): raw 17.7 KB a frame, this 7.1 KB — 2.12 -> 0.86 Mbps at 15 fps, the same as the front
    camera — and 26 -> 9.6 ms of CPU a frame, because the colourise and the JPEG get smaller too.

      * NEAREST downscale, never an average: averaging a 0.5 m edge with a 3 m wall invents a
        1.75 m surface that is not there.
      * a hole takes its neighbours' value (dilate, twice: up to ~4 px). Larger holes stay black,
        so a real "cannot see" — glass, black cloth, too close — still shows as one.
      * median 5 removes what is left of the speckle without moving edges.
    """
    _np, cv2 = _imaging()
    w, h = parse_size(size or DEPTH_OUT_SIZE)
    z = cv2.resize(z, (w, h), interpolation=cv2.INTER_NEAREST)
    kernel = _np.ones((5, 5), _np.uint8)
    for _ in range(2):
        z = _np.where(z == 0, cv2.dilate(z, kernel), z)
    return cv2.medianBlur(z, 5)


def colorize_depth(z, lo_mm=DEPTH_MIN_MM, hi_mm=DEPTH_MAX_MM):
    """uint16 millimetres -> BGR picture with a legend. 0 (no reading) stays black.

    Near is RED on purpose: this is a picture you steer by, and the thing about to be hit
    should be the loudest colour on the screen. The D400 depth unit is 1 mm by default.
    """
    np, cv2 = _imaging()
    span = max(hi_mm - lo_mm, 1)
    scaled = (np.clip(z.astype(np.float32), lo_mm, hi_mm) - lo_mm) * (255.0 / span)
    img = cv2.applyColorMap((255 - scaled).astype(np.uint8), cv2.COLORMAP_JET)
    img[z == 0] = 0
    _legend(img, lo_mm, hi_mm)
    return img


def _legend(img, lo_mm, hi_mm):
    np, cv2 = _imaging()
    h, w = img.shape[:2]
    bw, bh = w // 3, max(h // 40, 8)
    x0, y0 = w - bw - 12, h - bh - 24
    bar = np.linspace(255, 0, bw).astype(np.uint8)[None, :].repeat(bh, axis=0)
    img[y0:y0 + bh, x0:x0 + bw] = cv2.applyColorMap(bar, cv2.COLORMAP_JET)
    font, scale = cv2.FONT_HERSHEY_SIMPLEX, max(h / 1000, 0.4)
    for text, x in ((f"{lo_mm / 1000:g} m", x0), (f"{hi_mm / 1000:g} m", x0 + bw - 40)):
        cv2.putText(img, text, (x, y0 + bh + 16), font, scale, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, text, (x, y0 + bh + 16), font, scale, (255, 255, 255), 1, cv2.LINE_AA)


class RealSenseDepth:
    def __init__(self):
        lib = _librs()
        _imaging()
        self._ctx = self._pipe = self._cfg = self._profile = None
        try:
            self._ctx = _call(lib.rs2_create_context, _call(lib.rs2_get_api_version))
            devices = _call(lib.rs2_query_devices, self._ctx)
            count = _call(lib.rs2_get_device_count, devices)
            lib.rs2_delete_device_list(devices)
            if count == 0:
                raise CameraError("no RealSense camera connected")
            self._pipe = _call(lib.rs2_create_pipeline, self._ctx)
            self._cfg = _call(lib.rs2_create_config)
            w, h = parse_size(RS_DEPTH_SIZE)
            _call(lib.rs2_config_enable_stream, self._cfg, RS2_STREAM_DEPTH, -1, w, h,
                  RS2_FORMAT_Z16, rs_fps(RS_FPS))
            self._profile = _call(lib.rs2_pipeline_start_with_config, self._pipe, self._cfg)
        except CameraError:
            self.close()
            raise

    def read(self, timeout):
        lib = _lib
        np, _cv2 = _imaging()
        frames = _P()
        if not _call(lib.rs2_pipeline_try_wait_for_frames, self._pipe, ctypes.byref(frames),
                     int(timeout * 1000)):
            return None
        try:
            f = _call(lib.rs2_extract_frame, frames, 0)
            try:
                w = _call(lib.rs2_get_frame_width, f)
                h = _call(lib.rs2_get_frame_height, f)
                stride = _call(lib.rs2_get_frame_stride_in_bytes, f)
                data = ctypes.cast(_call(lib.rs2_get_frame_data, f),
                                   ctypes.POINTER(ctypes.c_uint8))
                # .copy(): the buffer belongs to librealsense and is reused once released.
                raw = np.ctypeslib.as_array(data, (h * stride,)).copy()
            finally:
                lib.rs2_release_frame(f)
        finally:
            lib.rs2_release_frame(frames)
        z = raw.view(np.uint16).reshape(h, stride // 2)[:, :w]
        return encode_jpeg_422(colorize_depth(smooth_depth(z)))

    def close(self):
        lib = _lib
        if self._profile:
            try:
                _call(lib.rs2_pipeline_stop, self._pipe)
            except CameraError as exc:
                log(f"stopping the RealSense: {exc}")
            lib.rs2_delete_pipeline_profile(self._profile)
        if self._cfg:
            lib.rs2_delete_config(self._cfg)
        if self._pipe:
            lib.rs2_delete_pipeline(self._pipe)
        if self._ctx:
            lib.rs2_delete_context(self._ctx)
        self._ctx = self._pipe = self._cfg = self._profile = None


VIDIOC_ENUM_FMT = 0xC0405602            # _IOWR('V', 2, struct v4l2_fmtdesc), 64 bytes
V4L2_BUF_TYPE_VIDEO_CAPTURE = 1


def _node_formats(path):
    """FourCCs a V4L2 node offers, e.g. ['YUYV'] — [] if it cannot be opened."""
    import fcntl
    import struct
    try:
        fd = os.open(path, os.O_RDWR)
    except OSError:
        return []
    out = []
    try:
        for i in range(32):
            desc = bytearray(64)
            struct.pack_into("II", desc, 0, i, V4L2_BUF_TYPE_VIDEO_CAPTURE)
            try:
                fcntl.ioctl(fd, VIDIOC_ENUM_FMT, desc)
            except OSError:
                break
            out.append(bytes(desc[44:48]).decode("ascii", "replace"))
    finally:
        os.close(fd)
    return out


def find_realsense_color_node(sysfs="/sys/class/video4linux"):
    """The /dev/videoN index of the RealSense's colour sensor, or CameraError.

    By what it offers, not by number: the D435i registers six nodes and only the colour one
    offers YUYV (depth is Z16, infrared is GREY/Y8I, the odd ones are metadata). The numbers
    move with plug order and with whatever else is on USB."""
    try:
        names = sorted(os.listdir(sysfs))
    except OSError:
        names = []
    for node in names:
        try:
            with open(f"{sysfs}/{node}/name") as fh:
                name = fh.read()
        except OSError:
            continue
        if "RealSense" in name and "YUYV" in _node_formats(f"/dev/{node}"):
            return int(node[len("video"):])
    raise CameraError("no RealSense colour node (YUYV) under /dev/video*")


def realsense_color_cmd(index):
    """The RealSense colour sensor as a 4:2:2 JPEG stream, captured and encoded by GStreamer.

    jpegenc fed YUY2 keeps the sensor's 4:2:2 as it is. NOT nvjpegenc: the NVJPG engine only
    takes I420/NV12, i.e. writes 4:2:0, and that is what broke the recording (module docstring)."""
    w, h = parse_size(RS_COLOR_SIZE)
    return ["gst-launch-1.0", "-q",
            "v4l2src", f"device=/dev/video{index}", "!",
            f"video/x-raw,format=YUY2,width={w},height={h},framerate={rs_fps(RS_FPS)}/1", "!",
            "jpegenc", f"quality={RS_QUALITY}", "!", "fdsink", "fd=1"]


# A colour pipeline that does not produce a frame in this long is not going to: the device is
# busy (the G1's videohub owns it) or gone. Measured first frame on the Go2: 0.9 s.
RS_COLOR_FIRST_FRAME_S = 3.0


# --------------------------------------------------------------------------- #
# The switch
# --------------------------------------------------------------------------- #
class Selector:
    """What is wanted, what is running, and why the last switch failed — shared by the HTTP
    thread (which asks) and the main loop (which is the only one that opens cameras)."""

    def __init__(self, initial):
        self._cv = threading.Condition()
        self.want = initial
        self.current = None
        self.error = None
        self.attempted = None
        self.switches = 0

    def request(self, camera, wait_s):
        """Ask for `camera` and wait until the main loop has acted on THIS request.

        Waiting for "any switch" is not enough: the answer to an earlier one (the initial
        open, or a request still in flight) would be reported as the answer to this one."""
        with self._cv:
            self.want = camera
            seen = self.switches
            self._cv.wait_for(
                lambda: (self.switches != seen and self.attempted == camera)
                or (self.current == camera and self.want == camera),
                wait_s)
            return self.status()

    def done(self, attempted, current, error):
        with self._cv:
            self.current, self.error, self.attempted = current, error, attempted
            # A failed switch falls back, so `want` must follow, or the loop would retry
            # the broken camera on every iteration. Only if nobody asked for something else
            # in the meantime — that request must not be overwritten by this answer.
            if self.want == attempted:
                self.want = current
            self.switches += 1
            self._cv.notify_all()

    def status(self):
        return {"ok": True, "camera": self.current, "choices": list(CAMERAS),
                "error": self.error}


SELECTOR = Selector(INITIAL if INITIAL in CAMERAS else "front")


def open_camera(name, front_cmd):
    if name == "front":
        return CommandCamera(front_cmd, "front")
    if name == "rs_depth":
        return RealSenseDepth()
    index = int(os.environ.get("RS_COLOR_DEVICE", "-1"))
    if index < 0:
        index = find_realsense_color_node()
    return CommandCamera(realsense_color_cmd(index), "rs_color", restart=False,
                         first_frame_s=RS_COLOR_FIRST_FRAME_S)


def run(front_cmd, out):
    """Main loop: switch when asked, otherwise copy the selected camera's frames to `out`."""
    cam, current = None, None
    try:
        while True:
            want = SELECTOR.want
            if want != current:
                if cam is not None:
                    cam.close()
                    cam = None
                error = None
                try:
                    cam = open_camera(want, front_cmd)
                    current = want
                except CameraError as exc:
                    error = f"{want}: {exc}"
                    log(f"cannot open {error}; falling back to front")
                    if want == "front":
                        raise   # nothing to fall back to
                    cam, current = open_camera("front", front_cmd), "front"
                log(f"camera: {current}")
                SELECTOR.done(want, current, error)
            try:
                frame = cam.read(0.5)
            except CameraError as exc:
                # A RealSense unplugged mid-stream lands here. Same rule as a failed open.
                if current == "front":
                    raise   # nothing to fall back to
                failed = current
                log(f"{failed} failed while streaming: {exc}; falling back to front")
                cam.close()
                cam, current = open_camera("front", front_cmd), "front"
                SELECTOR.done(failed, current, f"{failed}: {exc}")
                continue
            if frame:
                out.write(frame)
                out.flush()
    except BrokenPipeError:
        log("downstream closed, exiting")
    finally:
        if cam is not None:
            cam.close()


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "robot-camera-source"

    def log_message(self, *a):
        pass

    def _json(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _local(self):
        if self.client_address[0] in ("127.0.0.1", "::1"):
            return True
        log(f"rejected {self.command} from {self.client_address[0]} (localhost only)")
        self._json(403, {"ok": False, "error": "localhost only"})
        return False

    def do_GET(self):
        if self.path.split("?")[0].rstrip("/") != "/camera":
            return self.send_error(404)
        if self._local():
            self._json(200, SELECTOR.status())

    def do_POST(self):
        """POST /camera {"camera": "front"|"rs_color"|"rs_depth"} — answers once it took."""
        if self.path.split("?")[0].rstrip("/") != "/camera":
            return self.send_error(404)
        if not self._local():
            return None
        try:
            n = int(self.headers.get("Content-Length", "0"))
            camera = json.loads(self.rfile.read(n) or b"{}").get("camera")
        except (ValueError, AttributeError):
            return self._json(400, {"ok": False, "error": "bad json"})
        if camera not in CAMERAS:
            return self._json(400, {"ok": False,
                                    "error": f"camera must be one of {list(CAMERAS)}"})
        status = SELECTOR.request(camera, SWITCH_WAIT_S)
        if status["camera"] != camera and status["error"] is None:
            # Not refused, just not there yet: say so instead of claiming it took.
            return self._json(202, {**status, "pending": camera})
        log(f"switch requested: {camera} -> {status['camera']}")
        return self._json(200, status)


def main():
    front_cmd = sys.argv[1:]
    if INITIAL not in CAMERAS:
        log(f"CAMERA={INITIAL!r} is not one of {list(CAMERAS)}; starting on front")
    # SIGTERM from the supervisor must run the `finally` in run(), which is what stops the
    # child and releases the RealSense. The default action would skip it.
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    srv = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    log(f"control on http://127.0.0.1:{PORT}/camera, starting on {SELECTOR.want}")
    run(front_cmd, sys.stdout.buffer)


if __name__ == "__main__":
    main()
