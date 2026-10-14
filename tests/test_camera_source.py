"""camera_source — the runtime switch between the robot's camera and the RealSense.

Every test names the defect it catches. Nothing here needs a robot or a camera: the front
camera is a fake child process that prints JPEG-shaped frames, and the RealSense is either
absent (which is a real path — it is what a robot without one does) or a fake object.

The promise under test is the one the operator relies on: ONE camera at a time, a switch that
lands between frames, and a failed switch that falls back to the robot's own camera instead
of leaving the stream black.
"""
# Lazy annotations: this repo targets Python 3.8 (the robot's Jetson).
from __future__ import annotations

import json
import os
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import pytest

ROBOT = Path(__file__).resolve().parent.parent / "robot"
sys.path.insert(0, str(ROBOT))

import camera_source  # noqa: E402  (needs the sys.path line above)

SOI = b"\xff\xd8"
EOI = b"\xff\xd9"

# A front camera that prints numbered frames, like videohub_jpeg_stream does.
FAKE_FRONT = r"""
import sys, time
n = 0
limit = int(sys.argv[1]) if len(sys.argv) > 1 else 0
while not limit or n < limit:
    sys.stdout.buffer.write(b"\xff\xd8front-%d\xff\xd9" % n)
    sys.stdout.buffer.flush()
    n += 1
    time.sleep(0.02)
"""


def _free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


# --------------------------------------------------------------------------- #
# Framing
# --------------------------------------------------------------------------- #
def test_frames_split_across_reads_are_reassembled():
    """The defect: a switch, or a reader, that cuts a JPEG in half. A pipe hands over
    whatever it has, so a frame routinely arrives in two reads."""
    stream = SOI + b"one" + EOI + SOI + b"tw"
    frames, rest = camera_source.take_frames(stream)
    assert frames == [SOI + b"one" + EOI]
    frames, rest = camera_source.take_frames(rest + b"o" + EOI)
    assert frames == [SOI + b"two" + EOI]
    assert rest == b""


def test_garbage_before_a_frame_is_discarded():
    frames, _rest = camera_source.take_frames(b"noise" + SOI + b"x" + EOI)
    assert frames == [SOI + b"x" + EOI]


# --------------------------------------------------------------------------- #
# The process, end to end, with a fake front camera
# --------------------------------------------------------------------------- #
@pytest.fixture
def source(tmp_path):
    """camera_source running against the fake front camera, with NO RealSense library."""
    fake = tmp_path / "fake_front.py"
    fake.write_text(FAKE_FRONT)
    port = _free_port()
    env = {**os.environ, "CAMERA_PORT": str(port),
           "LIBREALSENSE": str(tmp_path / "no-such-librealsense.so")}
    proc = subprocess.Popen([sys.executable, str(ROBOT / "camera_source.py"),
                             sys.executable, str(fake)],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
    url = f"http://127.0.0.1:{port}/camera"
    deadline = time.monotonic() + 10
    while True:
        try:
            urllib.request.urlopen(url, timeout=1).close()
            break
        except OSError:
            if time.monotonic() > deadline:
                proc.kill()
                pytest.fail(f"camera_source never answered: {proc.stderr.read()!r}")
            time.sleep(0.05)
    yield proc, url
    proc.terminate()
    proc.wait(timeout=5)


def _read_frames(proc, n):
    buf, frames = b"", []
    while len(frames) < n:
        buf += os.read(proc.stdout.fileno(), 4096)
        got, buf = camera_source.take_frames(buf)
        frames += got
    return frames


def _post(url, body):
    req = urllib.request.Request(url, data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read())


def test_front_frames_pass_through_byte_exact(source):
    """The defect: the new hop altering what downstream has always received."""
    proc, _ = source
    frames = _read_frames(proc, 5)
    assert all(f.startswith(SOI + b"front-") and f.endswith(EOI) for f in frames)
    numbers = [int(f[len(SOI) + 6:-2]) for f in frames]
    assert numbers == list(range(numbers[0], numbers[0] + 5)), "frames lost or reordered"


def test_status_reports_the_running_camera(source):
    _, url = source
    with urllib.request.urlopen(url, timeout=2) as r:
        status = json.loads(r.read())
    assert status["camera"] == "front"
    assert status["choices"] == ["front", "rs_color", "rs_depth"]


def test_a_failed_switch_says_why_and_keeps_the_picture(source):
    """The defect: asking for the RealSense on a robot without one and getting a black
    screen, or a success that is not one. It must fall back AND say so."""
    proc, url = source
    code, status = _post(url, {"camera": "rs_depth"})
    assert code == 200
    assert status["camera"] == "front"
    assert "librealsense" in status["error"]
    assert _read_frames(proc, 3), "the stream stopped after a failed switch"


@pytest.mark.parametrize("body", [{"camera": "rear"}, {"camera": None}, {}])
def test_an_unknown_camera_is_refused(source, body):
    _, url = source
    code, status = _post(url, body)
    assert code == 400
    assert status["ok"] is False


# --------------------------------------------------------------------------- #
# The switch itself, in-process, with fake cameras
# --------------------------------------------------------------------------- #
class _FakeCam:
    open_now = 0
    max_open = 0

    def __init__(self, name, fail_open=False):
        if fail_open:
            raise camera_source.CameraError(f"{name} is not there")
        self.name = name
        _FakeCam.open_now += 1
        _FakeCam.max_open = max(_FakeCam.max_open, _FakeCam.open_now)

    def read(self, timeout):
        time.sleep(0.005)
        return SOI + self.name.encode() + EOI

    def close(self):
        _FakeCam.open_now -= 1


class _Out:
    """stdout stand-in: records frames, and ends run() after `limit` of them."""

    def __init__(self, limit):
        self.frames, self.limit = [], limit

    def write(self, frame):
        self.frames.append(frame)
        if len(self.frames) >= self.limit:
            raise BrokenPipeError

    def flush(self):
        pass


@pytest.fixture
def fresh(monkeypatch):
    _FakeCam.open_now = _FakeCam.max_open = 0
    sel = camera_source.Selector("front")
    monkeypatch.setattr(camera_source, "SELECTOR", sel)
    return sel


def test_only_one_camera_is_ever_open(monkeypatch, fresh):
    """The whole point of the switch: the link carries ONE picture. Two cameras open at once
    means two captures, and the next step from there is two streams on a shared SIM."""
    monkeypatch.setattr(camera_source, "open_camera", lambda name, _cmd: _FakeCam(name))
    out = _Out(300)
    t = threading.Thread(target=camera_source.run, args=([], out))
    t.start()
    for cam in ("rs_color", "rs_depth", "front", "rs_depth"):
        assert fresh.request(cam, 2)["camera"] == cam
    t.join(timeout=10)
    assert _FakeCam.max_open == 1
    assert _FakeCam.open_now == 0, "a camera was left open when run() ended"
    names = [f[2:-2].decode() for f in out.frames]
    assert {"front", "rs_color", "rs_depth"} <= set(names)


def test_a_camera_that_cannot_open_falls_back_to_front(monkeypatch, fresh):
    monkeypatch.setattr(camera_source, "open_camera",
                        lambda name, _cmd: _FakeCam(name, fail_open=name != "front"))
    out = _Out(100)
    t = threading.Thread(target=camera_source.run, args=([], out))
    t.start()
    status = fresh.request("rs_color", 2)
    t.join(timeout=10)
    assert status["camera"] == "front"
    assert "rs_color" in status["error"]
    assert fresh.want == "front", "the loop would retry the broken camera forever"


def test_a_request_during_a_switch_is_not_lost(fresh):
    """The defect this guards: done() resetting `want` to the camera it just opened,
    overwriting a request that arrived while the previous switch was in flight."""
    fresh.want = "rs_depth"                 # asked for while rs_color was opening
    fresh.done("rs_color", "rs_color", None)
    assert fresh.want == "rs_depth"


def test_the_front_camera_is_restarted_without_losing_the_choice(tmp_path, monkeypatch):
    """videohub_jpeg_stream exits after ~8 s without frames. That used to end the whole
    pipeline; here it must just come back, on the same camera."""
    fake = tmp_path / "fake_front.py"
    fake.write_text(FAKE_FRONT)
    monkeypatch.setattr(camera_source, "FRONT_RESTART_S", 0.1)
    cam = camera_source.CommandCamera([sys.executable, str(fake), "3"], "front")   # exits after 3
    try:
        got = []
        deadline = time.monotonic() + 10
        while len(got) < 6 and time.monotonic() < deadline:
            frame = cam.read(0.5)
            if frame:
                got.append(frame)
        assert len(got) == 6, "the front camera was not restarted after it exited"
    finally:
        cam.close()


def test_a_realsense_pipeline_that_dies_raises_instead_of_restarting(tmp_path):
    """The defect: treating a lost RealSense like the robot's camera service blinking, and
    restarting a pipeline that has no device every 3 s forever, on a black screen. It must
    raise, so the switch falls back to the robot's own camera."""
    fake = tmp_path / "fake.py"
    fake.write_text(FAKE_FRONT)
    cam = camera_source.CommandCamera([sys.executable, str(fake), "2"], "rs_color",
                                      restart=False)
    try:
        with pytest.raises(camera_source.CameraError):
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                cam.read(0.5)
    finally:
        cam.close()


def test_a_camera_that_never_delivers_fails_the_switch_itself():
    """The device is busy (the G1's videohub owns it): gst-launch starts and produces
    nothing. The open must fail within its budget so the operator is told, not shown black."""
    t = time.monotonic()
    with pytest.raises(camera_source.CameraError, match="no frame"):
        camera_source.CommandCamera([sys.executable, "-c", "import time; time.sleep(30)"],
                                    "rs_color", restart=False, first_frame_s=1.0)
    assert time.monotonic() - t < 5


def test_the_colour_node_is_found_by_what_it_offers_not_by_number(tmp_path, monkeypatch):
    """The defect: hardcoding /dev/video4. The D435i registers six nodes and their numbers
    move with plug order; only the colour one offers YUYV. Layout as measured on the Go2."""
    offers = {"video0": ["Z16 "], "video1": [], "video2": ["GREY", "Y8I "],
              "video3": [], "video4": ["YUYV"], "video5": []}
    for node in offers:
        (tmp_path / node).mkdir()
        (tmp_path / node / "name").write_text("Intel(R) RealSense(TM) Depth Ca\n")
    (tmp_path / "video6").mkdir()
    (tmp_path / "video6" / "name").write_text("USB webcam\n")      # YUYV too, but not ours
    offers["video6"] = ["YUYV"]
    monkeypatch.setattr(camera_source, "_node_formats",
                        lambda path: offers[path.rsplit("/", 1)[1]])
    assert camera_source.find_realsense_color_node(str(tmp_path)) == 4


def test_no_realsense_is_a_camera_error_not_a_crash(tmp_path):
    """CameraError is what triggers the fall-back to the robot's camera; anything else would
    take the whole video down."""
    with pytest.raises(camera_source.CameraError):
        camera_source.find_realsense_color_node(str(tmp_path))


# --------------------------------------------------------------------------- #
# The depth picture
# --------------------------------------------------------------------------- #
def test_depth_colours_near_red_far_blue_and_no_reading_black():
    """Near must be the loud colour: it is what is about to be hit. And a pixel with no
    reading must not masquerade as a distance."""
    np = pytest.importorskip("numpy")
    pytest.importorskip("cv2")
    z = np.zeros((480, 848), np.uint16)
    z[:, :200] = 400            # near
    z[:, 300:500] = 3900        # far
    img = camera_source.colorize_depth(z)
    b, g, r = (int(c) for c in img[100, 100])
    assert r > 100 and b < 60, f"near should be red, got bgr={b, g, r}"
    b, g, r = (int(c) for c in img[100, 400])
    assert b > 100 and r < 60, f"far should be blue, got bgr={b, g, r}"
    assert img[100, 700].tolist() == [0, 0, 0]


def test_the_colour_pipeline_writes_422_like_the_front_camera():
    """The defect: nvjpegenc, which can only write 4:2:0. Switching front (4:2:2) -> RealSense
    (4:2:0) -> front left the recording solid green until the encoder restarted (measured on the
    Go2 2026-10-14). jpegenc fed YUY2 keeps 4:2:2."""
    cmd = " ".join(camera_source.realsense_color_cmd(4))
    assert "format=YUY2" in cmd
    assert "! jpegenc" in cmd
    assert "nvjpegenc" not in cmd and "I420" not in cmd and "NV12" not in cmd


def test_the_depth_picture_is_encoded_422():
    """Same contract on the depth path: cv2.imencode in OpenCV 4.2 always writes 4:2:0."""
    np = pytest.importorskip("numpy")
    pytest.importorskip("cv2")
    pytest.importorskip("PIL")
    z = np.full((480, 848), 1500, np.uint16)
    jpeg = camera_source.encode_jpeg_422(camera_source.colorize_depth(z))
    assert camera_source.jpeg_luma_sampling(jpeg) == (2, 1), "not 4:2:2"


def test_the_sampling_reader_tells_422_from_420():
    sof = lambda h, v: (b"\xff\xd8" + b"\xff\xc0\x00\x11\x08\x00\x10\x00\x10\x03"  # noqa: E731
                        + bytes([1, (h << 4) | v, 0, 2, 0x11, 1, 3, 0x11, 1]))
    assert camera_source.jpeg_luma_sampling(sof(2, 1)) == (2, 1)
    assert camera_source.jpeg_luma_sampling(sof(2, 2)) == (2, 2)
    assert camera_source.jpeg_luma_sampling(b"\xff\xd8") is None


@pytest.mark.parametrize("maxfps, expect", [(0, 30), (30, 30), (15, 15), (10, 15), (5, 6)])
def test_the_realsense_rate_rounds_up_to_a_supported_one(maxfps, expect):
    """The D435 offers 6/15/30 fps. Rounding DOWN would cap the picture below the request."""
    assert camera_source.rs_fps(maxfps) == expect
