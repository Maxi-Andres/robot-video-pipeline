#!/usr/bin/env bash
# Build go2_h264_stream against the prebuilt Unitree SDK (no cmake needed).
set -euo pipefail
cd "$(dirname "$0")"

SDK="${UNITREE_SDK2_DIR:-$HOME/Desktop/unitree_sdk2}"
ARCH="$(uname -m)"   # x86_64 or aarch64

if [ ! -f "$SDK/lib/$ARCH/libunitree_sdk2.a" ]; then
  echo "error: SDK static lib not found at $SDK/lib/$ARCH/libunitree_sdk2.a" >&2
  echo "set UNITREE_SDK2_DIR to your unitree_sdk2 checkout" >&2
  exit 1
fi

INCS=(-I"$SDK/include" -I"$SDK/thirdparty/include" -I"$SDK/thirdparty/include/ddscxx")
LIBS=("$SDK/lib/$ARCH/libunitree_sdk2.a" -L"$SDK/thirdparty/lib/$ARCH" -lddscxx -lddsc
      -Wl,-rpath,"$SDK/thirdparty/lib/$ARCH" -lpthread)

# Primary path: JPEG (videohub) -> stdout. Reliable on this Go2.
g++ -O2 -std=c++17 src/go2_jpeg_stream.cpp -o go2_jpeg_stream "${INCS[@]}" "${LIBS[@]}"
echo "built ./go2_jpeg_stream"

# Experimental path: native H.264 (rt/frontvideostream). Does NOT deliver cleanly on
# this robot (see README); kept for future firmware / other units.
g++ -O2 -std=c++17 src/go2_h264_stream.cpp -o go2_h264_stream "${INCS[@]}" "${LIBS[@]}"
echo "built ./go2_h264_stream (experimental)"
