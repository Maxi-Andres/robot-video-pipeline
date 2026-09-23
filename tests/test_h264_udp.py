"""`mjpeg_server.udp_packets` / `udp_lease_port` — the robot half of the drive branch over UDP.

WHY THIS BRANCH EXISTS: over TCP one lost packet freezes the whole stream until it is
retransmitted. Measured over Starlink 2026-09-23: 10 stalls of 250-916 ms in 150 s, all on the
link. See the block above `udp_packets()` in mjpeg_server.py.

WHAT BREAKS SILENTLY HERE, and why each test exists:

* The datagram layout is a CONTRACT with `h264_relay.UdpReassembler` on the HQ side, which
  cannot import this module (network boundary). `GOLDEN` below is the same byte vector the
  HQ test decodes — `unitree_ros2/robot_camera_bridge/tests/test_h264_udp.py`. Change the
  format and BOTH vectors must change, or the drive view goes black with no error at all.
* A parity that does not XOR back to the missing fragment recovers a CORRUPT frame, and a
  decoder handed a corrupt access unit refuses the stream with an error that points nowhere
  near here.
* A datagram above the path MTU is fragmented by IP, and then one lost IP fragment loses the
  whole datagram with nothing to recover it from.
* The lease port decides where this robot sends video. It must fail safe: anything odd means
  no UDP, never a guessed port.

Nothing here opens a socket.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "robot"))

import mjpeg_server as m

# session 0x01020304, frame 7, b"ABCDEFGHIJ", capture 1790001782.5, payload 4, group 2.
# Three data fragments (ABCD, EFGH, IJ), then parity of group 0 (ABCD^EFGH) and group 1 (IJ).
# SAME VECTOR as the HQ side's test — keep the two identical.
GOLDEN = [bytes.fromhex(h) for h in (
    "41560100020004000403020107000000000003000a0000000000a09d50acda4141424344",
    "41560100020004000403020107000000010003000a0000000000a09d50acda4145464748",
    "41560100020004000403020107000000020003000a0000000000a09d50acda41494a",
    "41560101020004000403020107000000000003000a0000000000a09d50acda410404040c",
    "41560101020004000403020107000000010003000a0000000000a09d50acda41494a",
)]


def test_the_datagrams_match_the_contract_vector_byte_for_byte():
    assert m.udp_packets(0x01020304, 7, b"ABCDEFGHIJ", 1790001782.5,
                         payload=4, group=2) == GOLDEN


def test_the_header_fields_sit_where_the_contract_says():
    """Decoded by OFFSET, not with UDP_HEADER — so a reordered Struct cannot pass by agreeing
    with itself."""
    d = GOLDEN[3]
    assert d[0:2] == b"AV"
    assert d[2] == 1                                        # version
    assert d[3] == m.UDP_KIND_PARITY
    assert d[4] == 2                                        # group size
    assert int.from_bytes(d[6:8], "little") == 4            # payload size
    assert int.from_bytes(d[8:12], "little") == 0x01020304  # session
    assert int.from_bytes(d[12:16], "little") == 7          # frame
    assert int.from_bytes(d[16:18], "little") == 0          # group index
    assert int.from_bytes(d[18:20], "little") == 3          # DATA fragment count
    assert int.from_bytes(d[20:24], "little") == 10         # access unit length
    assert m.UDP_HEADER.size == 32


def _split(datagrams):
    h = m.UDP_HEADER.size
    data = [d[h:] for d in datagrams if d[3] == m.UDP_KIND_DATA]
    parity = [d[h:] for d in datagrams if d[3] == m.UDP_KIND_PARITY]
    return data, parity


@pytest.mark.parametrize("size", [1, 1199, 1200, 1201, 7614, 9600, 22124])
def test_every_single_lost_fragment_is_recoverable_from_its_group_parity(size):
    """Sizes: tiny, both sides of one payload, the QP40 frame measured on 2026-09-23, an exact
    multiple of a group, and the QP36 frame from the same day."""
    au = bytes((i * 7 + 3) & 0xFF for i in range(size))
    data, parity = _split(m.udp_packets(1, 1, au, 0.0))
    assert b"".join(data) == au
    for lost in range(len(data)):
        g = lost // m.UDP_GROUP
        members = data[g * m.UDP_GROUP:(g + 1) * m.UDP_GROUP]
        others = [f for i, f in enumerate(members) if g * m.UDP_GROUP + i != lost]
        rebuilt = m._xor([*others, parity[g]], len(parity[g]))[:len(data[lost])]
        assert rebuilt == data[lost]


def test_no_datagram_exceeds_the_path_mtu():
    """1408 is the path MTU implied by the TCP MSS measured on this link (1368 + 40)."""
    for d in m.udp_packets(1, 1, b"\xab" * 50000, 0.0):
        assert len(d) + 28 <= 1408                     # + IPv4 and UDP headers


def test_an_empty_or_oversized_access_unit_sends_nothing():
    assert m.udp_packets(1, 1, b"", 0.0) == []
    assert m.udp_packets(1, 1, b"x" * (m.UDP_MAX_AU + 1), 0.0) == []


def test_session_and_frame_wrap_instead_of_crashing_the_lease():
    """struct.error on a u32 overflow would kill the lease thread mid-drive."""
    d = m.udp_packets(2**32 + 5, 2**32 + 9, b"x", 0.0)[0]
    assert int.from_bytes(d[8:12], "little") == 5
    assert int.from_bytes(d[12:16], "little") == 9


@pytest.mark.parametrize("query, port", [
    ("udp=8895", 8895),
    ("udp=1024", 1024),
    ("udp=65535", 65535),
    ("udp=1023", None),         # privileged: never a video sink
    ("udp=65536", None),
    ("udp=0", None),
    ("udp=", None),
    ("udp=-1", None),
    ("udp=88a5", None),
    ("udp=%C2%B2", None),       # "²": isdigit() is True, int() raises
    ("udp=8895&udp=8896", None),  # ambiguous: refuse rather than pick one
    ("", None),
    ("other=1", None),
])
def test_the_lease_port_fails_safe(query, port):
    assert m.udp_lease_port(query) == port
