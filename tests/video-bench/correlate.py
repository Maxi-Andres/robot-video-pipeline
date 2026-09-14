#!/usr/bin/env python3
"""How much fresher is one video path than the other?

Both paths look at the SAME scene through the same camera, so the question is purely one of
timing: if a frame that arrived on path A at time t shows what a frame on path B only shows at
t+d, then A is d seconds ahead of B.

This needs no clock synchronisation and no readable stopwatch, which is why it is here: the
stopwatch photographed off a laptop screen came out an unreadable smear, and comparing an
absolute timestamp against a laptop whose clock is not ours produced a negative latency.

Each frame is reduced to a normalised grayscale thumbnail, so differences in exposure,
resolution and encoder between the two paths cancel and only the CONTENT is compared.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(sys.argv[1] if len(sys.argv) > 1 else ".")
THUMB = (64, 36)
SEARCH_MS = 2000      # look this far either way
STEP_MS = 10


def load(tag: str):
    rows = []
    for line in (ROOT / "index.txt").read_text().splitlines():
        d, name, ts = line.split()
        if d == tag:
            rows.append((float(ts), ROOT / d / name))
    rows.sort()
    times = np.array([t for t, _ in rows])
    imgs = []
    for _, path in rows:
        a = np.asarray(Image.open(path).convert("L").resize(THUMB), dtype=np.float32)
        a -= a.mean()
        s = a.std()
        imgs.append(a / s if s > 1e-6 else a)     # normalised: exposure cancels
    return times, np.stack(imgs).reshape(len(imgs), -1)


def main():
    ta, A = load("mc")      # multicast (native H.264)
    tb, B = load("jp")      # videohub (JPEG over DDS)
    print(f"multicast {len(ta)} cuadros, {1/np.median(np.diff(ta)):.1f} fps")
    print(f"videohub  {len(tb)} cuadros, {1/np.median(np.diff(tb)):.1f} fps")

    # Drop the edges: a frame near either end has no counterpart at large offsets.
    lo, hi = ta[0] + SEARCH_MS / 1000, ta[-1] - SEARCH_MS / 1000
    keep = (ta >= lo) & (ta <= hi)
    ta, A = ta[keep], A[keep]
    if len(ta) < 10:
        print("ventana demasiado corta")
        return

    best = []
    for d_ms in range(-SEARCH_MS, SEARCH_MS + 1, STEP_MS):
        d = d_ms / 1000.0
        # For each multicast frame, the videohub frame nearest in time once shifted by d.
        idx = np.searchsorted(tb, ta + d).clip(1, len(tb) - 1)
        left = np.abs(tb[idx - 1] - (ta + d)) < np.abs(tb[idx] - (ta + d))
        idx = np.where(left, idx - 1, idx)
        score = float(np.mean(np.sum(A * B[idx], axis=1)))   # correlation, higher = same scene
        best.append((score, d_ms))

    best.sort(reverse=True)
    top_score, top_d = best[0]
    print(f"\nmejor alineacion: {top_d:+d} ms   (score {top_score:.0f})")
    print("vecinos:", "  ".join(f"{d:+d}ms:{s:.0f}" for s, d in sorted(best[:7], key=lambda x: x[1])))
    if top_d > 0:
        print(f"\n=> el multicast va {top_d} ms ADELANTE del videohub")
    elif top_d < 0:
        print(f"\n=> el multicast va {-top_d} ms ATRAS del videohub")
    else:
        print("\n=> sin diferencia medible")
    # A flat curve means the scene barely changed and the result is not trustworthy.
    scores = np.array([s for s, _ in best])
    print(f"contraste del pico: {(scores.max()-scores.mean())/(scores.std()+1e-9):.1f} sigmas "
          f"({'confiable' if (scores.max()-scores.mean())/(scores.std()+1e-9) > 3 else 'DEBIL: poca accion en la escena'})")


if __name__ == "__main__":
    main()
