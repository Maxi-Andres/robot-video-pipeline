"""Does turning YOLO on delay the frames every OTHER viewer gets?

Connects a third read-only viewer to /ws/view and measures, per frame, the time from the
robot's own capture stamp (COM segment, survives the bridge's pass-through) to arrival here.
Same question the /drive machine is asking, answered with the same clock the robot uses.

Toggles the shared flag, measures both states, and RESTORES whatever it found.
"""
import json
import ssl
import statistics as st
import time
import urllib.request

import websocket  # websocket-client

VIEW = "wss://localhost:8443/ws/view"
HEALTH = "http://10.1.254.18:8093/health"
_TAG = b"AVL1 "
PHASE_S = 20.0


def clock_offset(n=9):
    offs = []
    for _ in range(n):
        t1 = time.time()
        with urllib.request.urlopen(HEALTH, timeout=5) as r:
            remote = json.load(r)["now"]
        t2 = time.time()
        offs.append(remote - (t1 + t2) / 2)
    return st.median(offs)


def stamp(jpeg):
    if not jpeg.startswith(b"\xff\xd8") or jpeg[2:4] != b"\xff\xfe":
        return None
    n = (jpeg[4] << 8) | jpeg[5]
    body = jpeg[6:4 + n]
    if not body.startswith(_TAG):
        return None
    try:
        a, _b = body[len(_TAG):].split()
        return float(a)
    except (ValueError, IndexError):
        return None


off = clock_offset()
print(f"offset del reloj del robot: {off*1000:+.0f} ms\n")

ws = websocket.create_connection(VIEW, sslopt={"cert_reqs": ssl.CERT_NONE}, timeout=15)
ws.settimeout(15)

original = None


def measure(label, seconds):
    global original
    ages, n, t0 = [], 0, time.monotonic()
    unstamped = 0
    while time.monotonic() - t0 < seconds:
        try:
            op, data = ws.recv_data()
        except Exception as e:
            print("  socket:", e)
            break
        if op == websocket.ABNF.OPCODE_TEXT:
            try:
                m = json.loads(data)
            except ValueError:
                continue
            if m.get("type") == "config" and original is None:
                original = bool(m["state"].get("enabled"))
            continue
        s = stamp(data)
        n += 1
        if s is None:
            unstamped += 1
            continue
        ages.append((time.time() + off) - s)
    if not ages:
        print(f"{label}: {n} cuadros, NINGUNO con stamp ({unstamped} sin) — no se puede medir")
        return None
    q = sorted(ages)
    print(f"{label}:  {n/seconds:5.2f} fps   latencia p50 {q[len(q)//2]*1000:6.1f} ms   "
          f"p95 {q[int(.95*len(q))]*1000:6.1f} ms   (n={len(q)})")
    return q[len(q)//2]


# Phase 1: whatever the session is in right now.
a = measure("estado actual ", PHASE_S)
print(f"  (YOLO estaba {'PRENDIDO' if original else 'apagado'})")

# Phase 2: the opposite.
ws.send(json.dumps({"enabled": not original}))
time.sleep(3)
b = measure(f"YOLO {'apagado' if original else 'PRENDIDO'}", PHASE_S)

# Restore.
ws.send(json.dumps({"enabled": original}))
time.sleep(1)
ws.close()
print(f"\nflag restaurado a {original}")
if a and b:
    d = (b - a) * 1000
    print(f"diferencia: {abs(d):.1f} ms {'MAS' if d>0 else 'MENOS'} en el segundo estado")
