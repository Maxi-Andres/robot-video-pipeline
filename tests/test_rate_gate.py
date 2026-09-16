"""`mjpeg_server.RateGate` — the fps cap, and the reason it used to lie.

THE BUG THESE CATCH, measured on the robot 2026-09-16: the operator set the live view's cap
to 10 fps and got **7.13**. It was not the link — the old gate compared each frame against
the arrival time of the last one it accepted, so with the camera at 14.3 fps it could only
ever deliver 14.3/k: 14.3, 7.15, 4.77, 3.58… Every cap between 7.2 and 14.2 collapsed onto
the same 7.15, and the knob silently meant something other than what it said.

The camera's real cadence is the fixture below: 14.3 fps, i.e. a frame every 69.93 ms. That
number is not decoration — the defect only appears when the source rate is not a multiple of
the cap, which is the normal case.

Nothing here touches a socket, a camera or a clock: the gate takes `now` as an argument
precisely so it can be driven with synthetic time.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "robot"))

import mjpeg_server

SOURCE_FPS = 14.3           # measured cadence of the Go2 videohub
SOURCE_PERIOD = 1.0 / SOURCE_FPS


def run(cap_fps, seconds, source_period=SOURCE_PERIOD, start=1000.0):
    """Feed a steady source through a gate and return the delivered rate."""
    gate = mjpeg_server.RateGate()
    min_gap = (1.0 / cap_fps) if cap_fps else 0.0
    sent = n = 0
    t = start
    while t < start + seconds:
        n += 1
        if gate.allows(t, min_gap):
            sent += 1
        t += source_period
    return sent / seconds


def test_a_cap_below_the_source_delivers_the_cap():
    """THE defect: a cap of 10 on a 14.3 fps source must deliver ~10, not 14.3/2 = 7.15.

    This is the one the operator hit. The old gate returns 7.15 here and the test goes red.
    """
    got = run(cap_fps=10, seconds=60)
    assert 9.5 <= got <= 10.5, f"cap 10 delivered {got:.2f} fps"


def test_every_cap_between_the_quantised_steps_is_honoured():
    """The old gate mapped 8, 10, 12 and 14 all onto 7.15 — four different settings, one
    outcome. Each must now land on its own value."""
    for cap in (8, 10, 12, 14):
        got = run(cap_fps=cap, seconds=60)
        assert abs(got - cap) <= 0.5, f"cap {cap} delivered {got:.2f} fps"


def test_a_cap_above_the_source_passes_everything():
    """Capping at more than the camera can produce must not throw anything away — the gate
    is a ceiling, not a schedule."""
    got = run(cap_fps=20, seconds=60)
    assert abs(got - SOURCE_FPS) <= 0.2, f"cap 20 delivered {got:.2f} of {SOURCE_FPS}"


def test_no_cap_passes_everything():
    got = run(cap_fps=0, seconds=60)
    assert abs(got - SOURCE_FPS) <= 0.05


def test_a_stall_does_not_release_a_burst():
    """After a gap, the deadline is in the past. Without the clamp the gate would pass frames
    back-to-back until it caught up — a burst of STALE pictures on the view being steered by,
    which is the exact failure `Latest` exists to prevent one layer up."""
    gate = mjpeg_server.RateGate()
    min_gap = 1.0 / 10
    t = 1000.0
    assert gate.allows(t, min_gap)
    t += 3.0                                    # a three-second stall
    assert gate.allows(t, min_gap)              # the frame that ends it goes through
    # The frames right behind it must NOT: they are 70 ms apart, the cap is 100 ms.
    assert not gate.allows(t + SOURCE_PERIOD, min_gap)
    assert gate.allows(t + 2 * SOURCE_PERIOD, min_gap)


def test_turning_the_cap_off_and_on_reseeds_the_phase():
    """`fps` is a LIVE control (POST /config). Going through 0 must not leave a stale deadline
    that swallows the first frames after the cap comes back."""
    gate = mjpeg_server.RateGate()
    t = 1000.0
    assert gate.allows(t, 1.0 / 10)
    assert gate.allows(t + 0.001, 0.0)          # cap off: everything passes
    assert gate.allows(t + 0.002, 1.0 / 10)     # cap back on: the next frame is not eaten


def test_the_gate_is_per_viewer():
    """Each HTTP client gets its own instance, so a second viewer connecting mid-stream does
    not inherit — or disturb — the first one's phase."""
    a, b = mjpeg_server.RateGate(), mjpeg_server.RateGate()
    min_gap = 1.0 / 10
    assert a.allows(1000.0, min_gap)
    assert not a.allows(1000.0 + SOURCE_PERIOD, min_gap)
    assert b.allows(1000.0 + SOURCE_PERIOD, min_gap)      # b is new: its first frame passes
