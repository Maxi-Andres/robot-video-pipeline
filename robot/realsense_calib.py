#!/usr/bin/env python3
"""
realsense_calib.py — the RealSense's own factory calibration, as the JSON the PC measures with.

    python3 robot/realsense_calib.py > g1_d435_calibration.json

Run ONCE per camera, on the robot (PC2 on the G1). The PC turns a box in the colour picture and
a depth snapshot into metres (AI-VL-core/src/perception3d.py); for that it needs the two lenses
(depth and colour intrinsics) and where one sensor sits from the other (depth -> colour
extrinsics). Every D435 carries its own, written at the factory; until this has been read the
PC uses the data sheet's values and says "nominal" on every answer.

WARNING — ON THE G1 IT PROBABLY TAKES THE COLOUR CAMERA DOWN. It only queries (no stream is
started), and that was assumed to be harmless next to Unitree's videohub_pc4, which holds the
colour node. It was not checked first, and on 2026-11-03 the one run on the G1 was followed, the
same minute, by videohub_pc4 dying, being respawned, and delivering no frames until PC2 was
rebooted. librealsense takes the whole device, not one node — the reason camera_source reads the
G1's depth through V4L2 (video.g1.env.example, DEPTH_BACKEND). The calibration is per camera and
already saved (AI-VL-core/config/g1_d435_calibration.json): run this again only for a NEW camera,
with the robot not in use, and reboot PC2 afterwards.

Reads through pyrealsense2 if it is installed, otherwise through the same librealsense C library
camera_source loads (LIBREALSENSE, or the usual install paths).

The output: {"depth": {width, height, fx, fy, ppx, ppy}, "color": {...},
"depth_to_color": {"rotation": 9 floats ROW-major, "translation": 3 floats, metres}, ...}.
librealsense stores the rotation COLUMN-major; it is transposed here, once.
"""
from __future__ import annotations

import ctypes
import json
import os
import sys
import time

DEPTH_SIZE = (848, 480)
COLOR_SIZE = (1280, 720)
LIB_PATHS = ("/opt/ros/noetic/lib/aarch64-linux-gnu/librealsense2.so",
             "/usr/lib/aarch64-linux-gnu/librealsense2.so",
             "/usr/local/lib/librealsense2.so", "/usr/lib/librealsense2.so")

RS2_STREAM_DEPTH, RS2_STREAM_COLOR = 1, 2


def _row_major(col_major):
    m = list(col_major)
    return [m[c * 3 + r] for r in range(3) for c in range(3)]


def _intr(width, height, fx, fy, ppx, ppy):
    return {"width": width, "height": height, "fx": round(fx, 4), "fy": round(fy, 4),
            "ppx": round(ppx, 4), "ppy": round(ppy, 4)}


def with_pyrealsense():
    import pyrealsense2 as rs
    devices = rs.context().query_devices()
    if len(devices) == 0:
        raise RuntimeError("no RealSense camera connected")
    dev = devices[0]
    found = {}
    for sensor in dev.query_sensors():
        for prof in sensor.get_stream_profiles():
            v = prof.as_video_stream_profile() if prof.is_video_stream_profile() else None
            if v is None:
                continue
            key = {rs.stream.depth: "depth", rs.stream.color: "color"}.get(prof.stream_type())
            want = DEPTH_SIZE if key == "depth" else COLOR_SIZE
            if key and key not in found and (v.width(), v.height()) == want:
                found[key] = v
    if set(found) != {"depth", "color"}:
        raise RuntimeError(f"profiles found: {sorted(found)}; need depth {DEPTH_SIZE} and "
                           f"colour {COLOR_SIZE}")
    out = {}
    for key, v in found.items():
        i = v.get_intrinsics()
        out[key] = _intr(i.width, i.height, i.fx, i.fy, i.ppx, i.ppy)
    e = found["depth"].get_extrinsics_to(found["color"])
    out["depth_to_color"] = {"rotation": [round(x, 6) for x in _row_major(e.rotation)],
                             "translation": [round(x, 6) for x in e.translation]}
    out["serial"] = dev.get_info(rs.camera_info.serial_number)
    return out


class _Intrinsics(ctypes.Structure):
    _fields_ = [("width", ctypes.c_int), ("height", ctypes.c_int), ("ppx", ctypes.c_float),
                ("ppy", ctypes.c_float), ("fx", ctypes.c_float), ("fy", ctypes.c_float),
                ("model", ctypes.c_int), ("coeffs", ctypes.c_float * 5)]


class _Extrinsics(ctypes.Structure):
    _fields_ = [("rotation", ctypes.c_float * 9), ("translation", ctypes.c_float * 3)]


def with_librealsense():
    paths = [os.environ["LIBREALSENSE"]] if os.environ.get("LIBREALSENSE") else LIB_PATHS
    lib = None
    for p in paths:
        try:
            lib = ctypes.CDLL(p)
            break
        except OSError:
            continue
    if lib is None:
        raise RuntimeError(f"no librealsense2.so in {list(paths)} and no pyrealsense2")
    P, Int = ctypes.c_void_p, ctypes.c_int
    err = P()
    E = ctypes.byref(err)

    def call(name, restype, *args):
        fn = getattr(lib, name)
        fn.restype = restype
        out = fn(*args, E)
        if err.value:
            lib.rs2_get_error_message.restype = ctypes.c_char_p
            msg = lib.rs2_get_error_message(err).decode("utf-8", "replace")
            raise RuntimeError(f"librealsense {name}: {msg}")
        return out

    ctx = call("rs2_create_context", P, Int(call("rs2_get_api_version", Int)))
    devs = call("rs2_query_devices", P, P(ctx))
    if call("rs2_get_device_count", Int, P(devs)) == 0:
        raise RuntimeError("no RealSense camera connected")
    dev = call("rs2_create_device", P, P(devs), Int(0))
    sensors = call("rs2_query_sensors", P, P(dev))
    found = {}
    for s in range(call("rs2_get_sensors_count", Int, P(sensors))):
        sensor = call("rs2_create_sensor", P, P(sensors), Int(s))
        profiles = call("rs2_get_stream_profiles", P, P(sensor))
        for k in range(call("rs2_get_stream_profiles_count", Int, P(profiles))):
            prof = call("rs2_get_stream_profile", P, P(profiles), Int(k))
            stream, fmt, index, uid, fps = (Int(), Int(), Int(), Int(), Int())
            call("rs2_get_stream_profile_data", None, P(prof), ctypes.byref(stream),
                 ctypes.byref(fmt), ctypes.byref(index), ctypes.byref(uid), ctypes.byref(fps))
            key = {RS2_STREAM_DEPTH: "depth", RS2_STREAM_COLOR: "color"}.get(stream.value)
            if not key or key in found:
                continue
            w, h = Int(), Int()
            try:
                call("rs2_get_video_stream_resolution", None, P(prof), ctypes.byref(w),
                     ctypes.byref(h))
            except RuntimeError:
                continue                     # not a video profile (motion, pose)
            if (w.value, h.value) == (DEPTH_SIZE if key == "depth" else COLOR_SIZE):
                found[key] = prof
    if set(found) != {"depth", "color"}:
        raise RuntimeError(f"profiles found: {sorted(found)}; need depth {DEPTH_SIZE} and "
                           f"colour {COLOR_SIZE}")
    out = {}
    for key, prof in found.items():
        i = _Intrinsics()
        call("rs2_get_video_stream_intrinsics", None, P(prof), ctypes.byref(i))
        out[key] = _intr(i.width, i.height, i.fx, i.fy, i.ppx, i.ppy)
    e = _Extrinsics()
    call("rs2_get_extrinsics", None, P(found["depth"]), P(found["color"]), ctypes.byref(e))
    out["depth_to_color"] = {"rotation": [round(x, 6) for x in _row_major(e.rotation)],
                             "translation": [round(x, 6) for x in e.translation]}
    return out


def main():
    errors = []
    for how, fn in (("pyrealsense2", with_pyrealsense), ("librealsense", with_librealsense)):
        try:
            out = fn()
        except ImportError as exc:
            errors.append(f"{how}: {exc}")
            continue
        except RuntimeError as exc:
            errors.append(f"{how}: {exc}")
            continue
        out.update({"read_with": how, "host": os.uname().nodename,
                    "date": time.strftime("%Y-%m-%d")})
        print(json.dumps(out, indent=2))
        return 0
    sys.stderr.write("could not read the calibration:\n  " + "\n  ".join(errors) + "\n")
    return 1


if __name__ == "__main__":
    sys.exit(main())
