#!/usr/bin/env bash
# Health / status of the whole robot-NVR stack: service, pipeline, RTSP, Frigate,
# robot link and recordings. Read-only — changes nothing.
cd "$(dirname "$0")"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
ROBOT_IP="${ROBOT_IP:-192.168.123.161}"
CAM="${CAM:-robot}"           # Frigate camera name
FFPROBE="./bin/ffprobe"

g(){ printf '\033[32m%s\033[0m' "$1"; }   # green
r(){ printf '\033[31m%s\033[0m' "$1"; }   # red
y(){ printf '\033[33m%s\033[0m' "$1"; }   # yellow
ok(){ printf '  [%s] %s\n' "$(g OK)" "$1"; }
bad(){ printf '  [%s] %s\n' "$(r --)" "$1"; }
warn(){ printf '  [%s] %s\n' "$(y '..')" "$1"; }

human_since(){ # $1 = a date string; prints "Xd Yh Zm (since ...)"
  local start now d
  start=$(date -d "$1" +%s 2>/dev/null) || { echo "?"; return; }
  now=$(date +%s); d=$((now-start))
  printf '%dd %dh %dm  (desde %s)' $((d/86400)) $(((d%86400)/3600)) $(((d%3600)/60)) "$1"
}

echo "=================== ROBOT-NVR · STATUS ==================="
date '+ahora: %Y-%m-%d %H:%M:%S %Z'
echo

# --- 1. systemd service ---
echo "SERVICIO (pipeline robot -> RTSP)"
state=$(systemctl --user is-active robot-nvr.service 2>/dev/null)
if [ "$state" = active ]; then
  ok "robot-nvr.service: $(g activo)"
  since=$(systemctl --user show robot-nvr.service --value -p ActiveEnterTimestamp 2>/dev/null)
  printf '       uptime : %s\n' "$(human_since "$since")"
  printf '       PID    : %s   reinicios: %s\n' \
    "$(systemctl --user show robot-nvr.service --value -p MainPID)" \
    "$(systemctl --user show robot-nvr.service --value -p NRestarts)"
  systemctl --user is-enabled robot-nvr.service >/dev/null 2>&1 \
    && printf '       boot   : %s\n' "$(g 'habilitado (arranca al iniciar sesion)')" \
    || printf '       boot   : %s\n' "$(y 'no habilitado')"
else
  bad "robot-nvr.service: $(r "${state:-desconocido}")   (arranca con: ./start-all.sh)"
fi
echo

# --- 2. pipeline processes ---
echo "PROCESOS DEL PIPELINE"
pgrep -f "mediamtx.yml"     >/dev/null 2>&1 && ok "mediamtx (servidor RTSP/HLS/WebRTC)" || bad "mediamtx caido"
pgrep -f "go2_jpeg_stream"  >/dev/null 2>&1 && ok "go2_jpeg_stream (captura del robot)" || bad "captura caida"
pgrep -f "bin/ffmpeg"       >/dev/null 2>&1 && ok "ffmpeg (encode H.264)"               || bad "ffmpeg caido"
echo

# --- 3. RTSP stream ---
echo "STREAM RTSP  (rtsp://<host>:8554/$CAM)"
if ss -ltn 2>/dev/null | grep -q ":8554"; then
  ok "mediamtx escuchando en :8554"
  if [ -x "$FFPROBE" ] && info=$(timeout 7 "$FFPROBE" -v error -rtsp_transport tcp \
        -i "rtsp://127.0.0.1:8554/$CAM" -show_entries stream=codec_name,width,height \
        -of csv=p=0:s=x 2>/dev/null) && [ -n "$info" ]; then
    ok "stream EN VIVO: $info"
  else
    warn "sin video en el stream ahora mismo (robot desconectado o sin frames)"
  fi
else
  bad "mediamtx no escucha en :8554"
fi
echo

# --- 4. Frigate NVR ---
echo "FRIGATE (NVR: graba + interfaz web)"
fstat=$(docker ps --filter name=frigate --format '{{.Status}}' 2>/dev/null)
if [ -n "$fstat" ]; then
  ok "contenedor: $fstat"
  fps=$(curl -s --max-time 5 http://127.0.0.1:5000/api/stats 2>/dev/null \
        | python3 -c "import sys,json;print(json.load(sys.stdin)['cameras']['$CAM']['camera_fps'])" 2>/dev/null)
  [ -n "$fps" ] && { [ "$fps" != "0.0" ] && ok "camara '$CAM' recibiendo: ${fps} fps" || warn "camara '$CAM': 0 fps (sin frames)"; }
  printf '       UI     : http://%s:5000\n' "$(hostname -I 2>/dev/null | awk '{print $1}')"
else
  bad "Frigate no esta corriendo   (arranca con: ./start-all.sh)"
fi
echo

# --- 5. robot link ---
echo "ROBOT"
if p=$(ping -c1 -W1 "$ROBOT_IP" 2>/dev/null | grep -oE 'time=[0-9.]+ ms'); then
  ok "$ROBOT_IP online ($p)"
else
  warn "$ROBOT_IP offline (sin robot -> el stream queda vacio; el NVR se recupera solo cuando vuelva)"
fi
echo

# --- 6. recordings ---
echo "GRABACIONES"
if [ -d frigate/media/recordings ]; then
  printf '       tamano : %s\n' "$(du -sh frigate/media 2>/dev/null | cut -f1)"
  last=$(find frigate/media/recordings -name '*.mp4' -printf '%T+ %p\n' 2>/dev/null | sort | tail -1)
  [ -n "$last" ] && printf '       ultimo : %s\n' "$(echo "$last" | awk '{print $1}' | cut -d. -f1 | tr '+' ' ')" \
                  || warn "todavia no hay grabaciones"
else
  warn "sin carpeta de grabaciones aun"
fi
echo "=========================================================="

# Keep the window open when launched by double-click (a spawned terminal closes as
# soon as the script ends). Skip the pause with STATUS_NO_PAUSE=1 for scripting.
if [ -z "${STATUS_NO_PAUSE:-}" ]; then
  echo
  read -rn1 -s -p "  Presioná una tecla para cerrar..." _ 2>/dev/null || sleep 30
  echo
fi
