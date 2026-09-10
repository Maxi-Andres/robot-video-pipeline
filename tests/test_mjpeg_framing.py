"""JPEG framing in `mjpeg_server.pump()` — the loop that turns a byte pipe into frames.

Every test here names the defect it catches. Nothing touches DDS, a robot, a socket or a
camera: `pump()` is driven with a fake stdin, so this whole file runs with the robot powered
off. Importing `mjpeg_server` is side-effect free — the HTTP server only starts in `main()`.

Why this file exists: the same ~20-line SOI/EOI scanner is duplicated in
`unitree_ros2/robot_camera_bridge/camera_sources.py::_read_stream`, and **neither copy has a
buffer ceiling**. The duplication is deliberate (the two run on different machines and the
boundary forbids sharing a module), so the only way to keep them honest is a test on each
side. This is the robot side.

One test is marked `xfail(strict=True)`: it asserts the CORRECT behavior for a defect that is
still open. Strict means that when the defect is fixed the test starts passing and pytest
FAILS on the unexpected pass — telling you to delete the marker. Fix the code, delete the
marker; do not delete the test.
"""
# Lazy annotations: this repo targets Python 3.8 (the robot's Jetson), where `list[bytes]`
# in an evaluated annotation is a TypeError.
from __future__ import annotations

import sys
import tracemalloc
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "robot"))

import mjpeg_server

SOI = b"\xff\xd8"
EOI = b"\xff\xd9"


# --------------------------------------------------------------------------- #
# Harness
# --------------------------------------------------------------------------- #
class _FakeStdin:
    """Stands in for `sys.stdin`, serving pre-baked chunks through `.buffer.read1()`.

    `pump()` reads via `read1()` on purpose (see its comment: `read(n)` on a pipe blocks for
    all n bytes and would add a second of latency), so the fake honours the same contract:
    return whatever is "available right now", and `b""` at EOF to make `pump()` return.
    """

    def __init__(self, chunks):
        self._chunks = list(chunks)
        self.buffer = self

    def read1(self, _size):
        return self._chunks.pop(0) if self._chunks else b""


def _run_pump(monkeypatch, chunks):
    """Drive `pump()` over `chunks` and return the frames it published, in order."""
    published = []
    # PUBLISH takes (frame, t_in): pump() always passes the capture instant, which is 0.0
    # unless STAMP is on. The framing tests only care about the bytes.
    monkeypatch.setattr(mjpeg_server, "PUBLISH",
                        lambda frame, _t_in=0.0: published.append(frame))
    monkeypatch.setattr(mjpeg_server, "nvr_offer", lambda _frame: None)
    monkeypatch.setattr(mjpeg_server, "log", lambda *_a, **_k: None)
    monkeypatch.setattr(sys, "stdin", _FakeStdin(chunks))
    mjpeg_server.pump()
    return published


def _frame(payload: bytes = b"body") -> bytes:
    return SOI + payload + EOI


# --------------------------------------------------------------------------- #
# Correctness of the scanner
# --------------------------------------------------------------------------- #
def test_a_whole_frame_in_one_chunk_is_published(monkeypatch):
    assert _run_pump(monkeypatch, [_frame(b"one")]) == [_frame(b"one")]


def test_a_frame_split_across_chunks_is_reassembled(monkeypatch):
    """The defect this catches: publishing a truncated frame when a read lands mid-JPEG.

    A 200 KB frame never arrives in one 64 KB read, so this is the normal case, not the
    edge case.
    """
    whole = _frame(b"abcdefghij")
    chunks = [whole[:4], whole[4:9], whole[9:]]
    assert _run_pump(monkeypatch, chunks) == [whole]


def test_an_soi_marker_straddling_two_chunks_is_not_lost(monkeypatch):
    """The defect: dropping a frame because its 2-byte start marker was split by a read.

    This is what the `buf = buf[-1:]` tail retention is for — a one-byte carry. Feed the
    `\\xff` at the very end of one chunk and the `\\xd8` at the start of the next.
    """
    chunks = [b"junk\xff", b"\xd8payload" + EOI]
    assert _run_pump(monkeypatch, chunks) == [SOI + b"payload" + EOI]


def test_an_eoi_marker_straddling_two_chunks_is_not_lost(monkeypatch):
    chunks = [SOI + b"payload\xff", b"\xd9"]
    assert _run_pump(monkeypatch, chunks) == [SOI + b"payload" + EOI]


def test_garbage_before_the_soi_is_discarded(monkeypatch):
    """The defect: prepending multipart headers (or any preamble) to the JPEG bytes.

    `go2_jpeg_stream` concatenates frames raw, but Frigate and mediamtx wrap them in
    multipart boundaries — the scanner has to survive both, which is exactly why it looks
    for markers instead of parsing boundaries.
    """
    preamble = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
    assert _run_pump(monkeypatch, [preamble + _frame(b"x")]) == [_frame(b"x")]


def test_bytes_between_two_frames_are_discarded(monkeypatch):
    a, b = _frame(b"first"), _frame(b"second")
    chunks = [a + b"\r\n--frame\r\n" + b]
    assert _run_pump(monkeypatch, chunks) == [a, b]


def test_two_frames_in_one_chunk_are_both_published(monkeypatch):
    """The defect: an inner `while True` that breaks after one frame, halving the frame rate."""
    a, b = _frame(b"first"), _frame(b"second")
    assert _run_pump(monkeypatch, [a + b]) == [a, b]


def test_a_chunk_with_no_markers_at_all_publishes_nothing(monkeypatch):
    assert _run_pump(monkeypatch, [b"no markers here", b"still none"]) == []


def test_a_trailing_partial_frame_is_not_published(monkeypatch):
    """The defect: flushing whatever is in the buffer when the pipe closes.

    A JPEG with no EOI is a truncated image; handing it to a viewer is worse than dropping it.
    """
    assert _run_pump(monkeypatch, [_frame(b"good"), SOI + b"truncated"]) == [_frame(b"good")]


# --------------------------------------------------------------------------- #
# The missing buffer ceiling — open defect
# --------------------------------------------------------------------------- #
@pytest.mark.xfail(
    strict=True,
    reason="no buffer ceiling: on an SOI with no EOI, `buf = buf[start:]` accumulates every "
    "subsequent chunk without bound. On the robot the unit caps at MemoryMax=512M, so the "
    "OOM killer takes the video publisher down.",
)
def test_an_soi_with_no_eoi_does_not_grow_the_buffer_without_bound(monkeypatch):
    """The defect: unbounded memory growth on a stream that starts a frame and never ends it.

    Reachable without an attacker: a camera service that dies mid-frame leaves an SOI with no
    EOI on the wire, and the scanner then holds every byte that follows.

    Measured with `tracemalloc` rather than by reaching into `pump()`'s locals: feed 5 MB
    after an unterminated SOI and assert the peak stays small. With a ceiling the buffer is
    bounded; without one the peak tracks the whole 5 MB.
    """
    chunk = b"\x00" * 65536
    chunks = [SOI] + [chunk] * 80  # 5 MB, none of it a valid frame

    tracemalloc.start()
    try:
        _run_pump(monkeypatch, chunks)
        _current, peak = tracemalloc.get_traced_memory()
    finally:
        tracemalloc.stop()

    assert peak < 2 * 1024 * 1024, (
        f"the scanner held {peak / 1024 / 1024:.1f} MB of a 5 MB unterminated frame; it "
        "needs a ceiling that abandons the frame and resyncs on the next SOI"
    )


# --------------------------------------------------------------------------- #
# The latency stamp
#
# The measurement is only trustworthy if carrying it cannot change what is measured or
# corrupt what is carried. These name the two ways that could go wrong.
# --------------------------------------------------------------------------- #
def test_a_stamped_frame_is_still_a_valid_jpeg_envelope():
    """The defect: splicing the COM segment in the wrong place, so a decoder that is
    strict about SOI-first (or about the segment length) refuses the frame. A stamp that
    breaks the picture is worse than no stamp."""
    raw = SOI + b"payload" + EOI
    out = mjpeg_server.stamp(raw, 1.0, 2.0)
    assert out.startswith(SOI), "SOI must stay first"
    assert out.endswith(EOI), "EOI must stay last"
    assert out[2:4] == b"\xff\xfe", "the COM marker goes immediately after SOI"
    declared = (out[4] << 8) | out[5]
    assert out[4 + declared:] == raw[2:], (
        "the declared segment length must cover exactly the payload, so a decoder "
        "resumes at the original first byte after SOI")
    assert raw[2:] in out, "the original bytes must survive untouched"


def test_the_stamp_round_trips_and_is_absent_from_an_unstamped_frame():
    """The defect: read_stamp() drifting out of step with stamp(), which would silently
    report a wrong latency instead of no latency. Also pins that an UNSTAMPED frame
    reads as None rather than as garbage numbers."""
    assert mjpeg_server.read_stamp(mjpeg_server.stamp(SOI + b"x" + EOI,
                                                      1234.5, 1234.75)) == (1234.5, 1234.75)
    assert mjpeg_server.read_stamp(SOI + b"x" + EOI) is None
    assert mjpeg_server.read_stamp(b"") is None
    assert mjpeg_server.read_stamp(SOI + b"\xff\xfe\x00\x04zz" + EOI) is None
