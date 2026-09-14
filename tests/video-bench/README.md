# video-bench — measuring both branches of the drive video

Two pictures leave the robot: raw JPEGs over HTTP (`:8093`, what the drive view actually
uses today) and H.264 over RTMP to mediamtx (what WebRTC would carry). They are tuned
separately and they fail differently, so they are measured separately here.

Everything is read-only against the robot. Nothing writes to it, nothing reconfigures it.

## Why the old probe was not enough

`tests/_latency_probe.py` carries its timestamps in a **JPEG COM segment**, which the H.264
encode destroys. It can only ever measure the MJPEG branch. Two replacements live here: a
field probe that reads that same stamp but reports what the plan's targets are written in,
and a **barcode burned into the pixels**, which survives H.264 (verified: 109/109 frames
decoded after encoding at 1.5 Mbps).

## 1. The MJPEG branch, over whatever link the robot is on

```bash
python3 field_probe.py 60
```

Reports frames/s, bitrate, latency from capture to here (needs `STAMP=1` in `video.env`),
and stalls. **Read the cadence line before the stall line**: a branch capped at 5 fps has a
200 ms nominal spacing, so an absolute "gaps > 200 ms" count is meaningless there — the
probe prints both that and the count of intervals over twice the measured cadence, which is
what a freeze actually looks like.

## 2. What the robot costs the uplink

```bash
./rtmp_bitrate.sh 30
```

Bytes on the RTMP socket, straight from the kernel. Use this and not the encoder's `bitrate`
property, which was measured overshooting by 4.7x before the rate-control fix.

## 3. The H.264 branch as a viewer sees it

The browser is the only thing that can report WebRTC freezes, jitter-buffer delay and packet
loss, so this half runs in a page.

```bash
python3 synthetic_source.py --serve-only --port 8099 --pagedir . &
firefox 'http://127.0.0.1:8099/?whep=http://127.0.0.1:8889/robot/whep&mjpeg=off&barcode=0'
grep '"rtc":' report.jsonl | python3 show.py
```

`barcode=0` because real camera frames carry none; `mjpeg=off` because the robot's `:8093`
sends no CORS header, so the page cannot read it cross-origin — that branch is `field_probe`'s
job. The page posts a window of statistics every 30 s to `report.jsonl`.

Two traps, both hit while building this:

* **`framesDecoded` from `getStats()` is the frame rate, not the `requestVideoFrameCallback`
  count.** rVFC coalesces when two decoded frames land in one compositor cycle, which read as
  11.7 fps while the browser was in fact decoding all 15. The latency percentiles stay valid;
  the rate does not.
* **Do not time the stream by decoding mediamtx's RTSP output with ffmpeg.** It reports ~272 ms
  that are ffmpeg's own buffering, and it gives itself away by reading at 17.8 fps while
  catching up on a backlog instead of the true 15.
* **Close other bench tabs.** Every open tab measures its own peer connection and posts to
  the same `report.jsonl`; mixed sessions showed up as a *negative* bitrate. Each window now
  carries a session id and `show.py` reports only the newest unless given `--all`.
* **A hidden tab invents stalls.** Firefox throttles `requestVideoFrameCallback` in
  background tabs, which produced a 2893 ms "gap" in a window where `getStats` reported
  `freezeCount: 0`. Windows now report `hidden_ms`; when the two disagree, believe
  `freezeCount` — it is computed in the decoder.

## 4. Comparing two paths without trusting any clock

`correlate.py` answers "how much fresher is path A than path B" by aligning the two streams on
CONTENT, not on timestamps. Capture both in parallel with per-frame arrival times, then:

```bash
python3 correlate.py <dir with mc/ jp/ index.txt>
```

Use it instead of a stopwatch whenever the two clocks are not the same machine's. MEASURED the
hard way 2026-09-14: a stopwatch filmed off a laptop screen gave **-710 ms** and then
**-1400 ms** — negative, so impossible, and inconsistent with each other, because that laptop's
clock is neither ours nor stable. Correlation has no such dependency.

It needs MOVEMENT in the scene. With a still room the peak came out at 1.7 sigmas and the curve
was flat across ±70 ms — useless. With a hand waving in front of the camera it was 3.2 sigmas
and unambiguous. The script prints that contrast so a weak result cannot be mistaken for a
measurement.

## 5. Isolating a pipeline stage from the robot and the link

`synthetic_source.py` renders frames that carry their creation time as a barcode and feeds
**both** branches from one render, so the difference between the two measured latencies is
exactly what H.264 + RTMP + mediamtx + WebRTC costs.

```bash
# an isolated mediamtx twin on shifted ports — never touches production or Frigate
mediamtx mediamtx-test.yml &
python3 synthetic_source.py --stdout --port 8099 --pagedir . \
  | ffmpeg -use_wallclock_as_timestamps 1 -f mjpeg -i pipe:0 \
      -c:v libx264 -preset veryfast -tune zerolatency \
      -b:v 1500k -maxrate 1500k -bufsize 300k -g 15 -pix_fmt yuv420p \
      -fps_mode passthrough -f flv rtmp://127.0.0.1:11935/lat
firefox http://127.0.0.1:8099/
```

This is what established that mediamtx + WebRTC adds ~127 ms p50 and not the seconds that
routing the live view through Frigate once cost.
