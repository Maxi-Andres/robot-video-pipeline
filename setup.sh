#!/usr/bin/env bash
# Download the binary dependencies that are NOT stored in git:
#   - mediamtx  (RTSP/HLS/WebRTC server, single binary)
#   - ffmpeg + ffprobe  (static builds, dropped in bin/)
# Run this once after cloning, before ./build.sh.
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p bin

# --- mediamtx (latest linux_amd64) ---
if [ -x ./mediamtx ]; then
  echo "[setup] mediamtx already present"
else
  echo "[setup] downloading mediamtx..."
  URL=$(curl -s https://api.github.com/repos/bluenviron/mediamtx/releases/latest \
        | grep -oE '"browser_download_url": *"[^"]*linux_amd64\.tar\.gz"' \
        | head -1 | sed -E 's/.*"(https[^"]+)"/\1/')
  [ -n "$URL" ] || { echo "could not resolve mediamtx download URL" >&2; exit 1; }
  curl -sL "$URL" -o /tmp/mediamtx.tar.gz
  tmp=$(mktemp -d)
  tar xzf /tmp/mediamtx.tar.gz -C "$tmp"
  cp "$tmp/mediamtx" ./mediamtx
  # Keep the full stock config as reference (our own mediamtx.yml is versioned).
  [ -f "$tmp/mediamtx.yml" ] && cp "$tmp/mediamtx.yml" ./mediamtx.stock.yml || true
  chmod +x ./mediamtx
  rm -rf "$tmp" /tmp/mediamtx.tar.gz
  echo "[setup] mediamtx ready: $(./mediamtx --version 2>/dev/null || echo installed)"
fi

# --- ffmpeg + ffprobe (static amd64) ---
if [ -x ./bin/ffmpeg ] && [ -x ./bin/ffprobe ]; then
  echo "[setup] ffmpeg already present"
else
  echo "[setup] downloading static ffmpeg..."
  curl -sL https://johnvansickle.com/ffmpeg/releases/ffmpeg-release-amd64-static.tar.xz \
       -o /tmp/ffmpeg.tar.xz
  d=$(tar tf /tmp/ffmpeg.tar.xz | head -1 | cut -d/ -f1)
  tar xf /tmp/ffmpeg.tar.xz -C /tmp "$d/ffmpeg" "$d/ffprobe"
  cp /tmp/"$d"/ffmpeg /tmp/"$d"/ffprobe ./bin/
  rm -rf /tmp/ffmpeg.tar.xz /tmp/"$d"
  echo "[setup] ffmpeg ready: $(./bin/ffmpeg -version 2>/dev/null | head -1)"
fi

echo "[setup] done. Next: ./build.sh, then ./start-all.sh"
