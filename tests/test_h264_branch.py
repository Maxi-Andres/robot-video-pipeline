"""`mjpeg_server.AuReader` — framing the drive branch's H.264 access units.

WHY THIS PARSER EXISTS. The encoder's Annex-B byte stream carries no lengths, so the only way
to know an access unit ended is to see the NEXT one start — a whole frame of lookahead (~70 ms
at the camera's cadence) on the one path whose entire purpose is being fast. `multipartmux`
writes an explicit `Content-Length` per frame for ~80 bytes of overhead (measured on the robot
2026-09-21: 380857 bytes for the same 90 frames that are 373657 raw), and this reads it back.

WHAT MAKES IT WORTH TESTING: an access unit handed to a decoder with ONE byte missing or one
byte extra is not a slightly worse picture, it is a stream the decoder refuses — and the error
it gives back ("Operation is not supported") points nowhere near the framing. Every test here
checks the bytes come out EXACTLY.

Nothing here touches gst, a robot or a socket.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "robot"))

import mjpeg_server

B = b"--ThisRandomString"


def part(payload: bytes, ctype: bytes = b"video/x-h264") -> bytes:
    """One multipart part, shaped like `multipartmux` writes it."""
    return (B + b"\r\nContent-Type: " + ctype + b"\r\nContent-Length: "
            + str(len(payload)).encode() + b"\r\n\r\n" + payload + b"\r\n")


AU1 = bytes(range(256)) * 4          # 1024 B, every byte value, so any truncation shows
AU2 = b"\x00\x00\x00\x01\x67" + b"\xab" * 500


def test_a_whole_part_in_one_chunk_yields_it_exactly():
    r = mjpeg_server.AuReader()
    assert r.feed(part(AU1)) == [AU1]


def test_a_part_split_across_chunks_is_reassembled():
    """The body arrives in pieces off a pipe. Returning early would hand the decoder a
    truncated access unit, which it rejects outright."""
    r = mjpeg_server.AuReader()
    blob = part(AU1)
    assert r.feed(blob[:40]) == []
    assert r.feed(blob[40:300]) == []
    assert r.feed(blob[300:]) == [AU1]


def test_a_header_split_across_chunks_is_not_lost():
    """The Content-Length line itself can straddle a read. Parsing what is there so far
    would read a truncated number and frame everything after it wrongly."""
    r = mjpeg_server.AuReader()
    blob = part(AU1)
    cut = blob.index(b"Content-Length:") + 18      # mid-number
    assert r.feed(blob[:cut]) == []
    assert r.feed(blob[cut:]) == [AU1]


def test_two_parts_in_one_chunk_come_out_in_order():
    r = mjpeg_server.AuReader()
    assert r.feed(part(AU1) + part(AU2)) == [AU1, AU2]


def test_the_payload_is_byte_exact():
    """A single byte of drift — eating the blank line, or keeping the trailing CRLF — is
    invisible here and fatal at the decoder."""
    r = mjpeg_server.AuReader()
    (got,) = r.feed(part(AU2))
    assert got == AU2
    assert len(got) == len(AU2)
    assert not got.endswith(b"\r\n")


def test_a_payload_that_never_arrives_does_not_grow_the_buffer_without_bound():
    """A wedged or desynchronised child must not be able to eat memory. Same audit the JPEG
    scanners in this repo went through.

    The declared length here is larger than any real access unit, so honouring it would mean
    buffering 1 GB waiting for bytes that never come."""
    cap = mjpeg_server.AuReader._MAX_PART
    r = mjpeg_server.AuReader()
    assert r.feed(B + b"\r\nContent-Length: 999999999\r\n\r\n") == []
    for _ in range(40):                       # 4 MB of body that will never be complete
        assert r.feed(b"x" * 100_000) == []
    assert len(r._buf) <= cap + 100_000, f"buffer grew to {len(r._buf)} with a cap of {cap}"


def test_garbage_with_no_header_is_dropped_rather_than_buffered_forever():
    """If the stream desynchronises there is no header to find; holding the bytes for ever
    would turn a hiccup into an out-of-memory."""
    r = mjpeg_server.AuReader()
    for _ in range(60):
        r.feed(b"\x00\xff" * 100_000)
    assert len(r._buf) <= mjpeg_server.AuReader._MAX_PART
    # and it recovers: a good part after the garbage still parses
    assert r.feed(part(AU1)) == [AU1]


def test_an_oversized_declared_length_is_skipped_and_the_next_part_still_parses():
    """A corrupt length must cost ONE part, not the stream. The reader has to step over it and
    resynchronise on the next header — otherwise a single bad frame ends the drive view until
    something restarts it."""
    r = mjpeg_server.AuReader()
    bogus = B + b"\r\nContent-Length: " + str(mjpeg_server.AuReader._MAX_PART + 1).encode() \
        + b"\r\n\r\n"
    assert r.feed(bogus + part(AU1)) == [AU1]
