# robot-nvr-bridge

Puente **independiente** que toma el video de la cámara del **robot Unitree Go2** y lo
publica para que se pueda **ver en vivo y grabar en un NVR**. No toca ni depende de
`AI-VL-ecosystem`; solo se apoya en el SDK de Unitree ya instalado para leer la cámara.

> TL;DR: `./start-all.sh` y abrís `http://192.168.123.99:5000` en el navegador.

---

## 1. Qué hace, en una frase

El robot no expone la cámara como una cámara IP normal (no hay una URL de video que
puedas abrir directo). El video sale por el bus interno de Unitree (DDS). Este proyecto
**lee ese video, lo convierte a un stream estándar (RTSP/H.264) y se lo entrega a un
NVR** (Frigate) que lo muestra y lo graba. Cualquiera en la red puede mirarlo desde el
navegador.

---

## 2. Cómo funciona (la cadena completa)

```
┌─────────┐   JPEG por DDS    ┌──────────────────┐   MJPEG    ┌─────────┐  H.264   ┌──────────┐  RTSP   ┌─────────┐
│  Robot  │ ───(SDK Unitree)─▶│ go2_jpeg_stream  │ ──(pipe)──▶│ ffmpeg  │ ───────▶ │ mediamtx │ ──────▶ │ Frigate │
│  Go2    │   videohub 1001   │  (C++, propio)   │   stdout   │ (encode)│          │ (server) │  :8554  │  (NVR)  │
└─────────┘                   └──────────────────┘            └─────────┘          └──────────┘         └─────────┘
                                                                                        │ también            │
                                                                                        ▼ HLS/WebRTC         ▼ web UI
                                                                                   navegador            navegador
                                                                                   :8888 / :8889        :5000
```

Paso a paso:

1. **`go2_jpeg_stream`** (programa propio en C++) le pide fotos a la cámara del robot
   usando el SDK de Unitree (función `GetImageSample`, servicio "videohub", API 1001).
   El robot devuelve un **JPEG** por cada pedido (~180 KB, 1920×1080). El programa
   escribe esos JPEG uno atrás de otro por su salida estándar (un stream MJPEG).
2. **`ffmpeg`** toma ese MJPEG, lo **codifica a H.264** (ajustes de baja latencia:
   `ultrafast` + `zerolatency`) y lo **empuja por RTSP**.
3. **`mediamtx`** es un servidor de streaming: recibe ese RTSP y lo re-publica para que
   lo consuma quien quiera — por **RTSP** (`:8554`, lo que usan los NVR), por **HLS**
   (`:8888`) y por **WebRTC** (`:8889`, lo más rápido para navegador).
4. **Frigate** (el NVR) se conecta al RTSP de mediamtx, **muestra el video en vivo en
   una página web y lo graba a disco**. También corre detección de objetos por CPU.

Por qué hay dos "servidores" (mediamtx y Frigate) y no uno solo: mediamtx desacopla la
fuente del consumidor. Así podés ver el video en VLC/navegador **aunque Frigate esté
apagado**, y podés enchufar otro NVR distinto sin cambiar nada de la captura.

---

## 3. Tecnologías que usa y por qué

| Componente | Qué es | Por qué está acá |
|------------|--------|------------------|
| **Unitree SDK2** (C++) | SDK oficial del robot | Es la única forma de leer la cámara: el video va por DDS, no por una URL |
| **DDS / CycloneDDS** | Bus de mensajería en tiempo real | Es el transporte interno del robot; el SDK habla DDS por debajo |
| **`go2_jpeg_stream`** | Programa propio (C++) | Pide los JPEG de la cámara y los vuelca como stream. Es el "adaptador" robot→ffmpeg |
| **FFmpeg** | Suite de video | Codifica JPEG→H.264 y lo empuja por RTSP. Va como binario estático en `bin/` (no ensucia el sistema) |
| **MediaMTX** | Servidor RTSP/HLS/WebRTC | Recibe el stream y lo reparte en protocolos estándar. Un solo binario, sin instalar |
| **Frigate** | NVR open-source | Muestra en vivo, **graba**, tiene línea de tiempo y detección de objetos. Corre en Docker |
| **Docker / Compose** | Contenedores | Frigate se levanta con un `docker compose up -d`, sin instalar dependencias a mano |
| **H.264 / RTSP** | Códec / protocolo | El "idioma" que entienden todos los NVR y reproductores |

---

## 4. Cómo se usa

### Requisito previo (una sola vez)
```bash
cd ~/Desktop/robot-nvr-bridge
./setup.sh          # descarga mediamtx + ffmpeg (no se versionan en git)
./build.sh          # compila go2_jpeg_stream contra el SDK (necesita g++)
```

### Prender todo
```bash
./start-all.sh      # levanta el pipeline (como servicio) y el NVR Frigate
```
La primera vez, `start-all.sh` instala el pipeline como **servicio systemd de usuario**
(`robot-nvr.service`) con `Restart=always`, así queda **siempre prendido**: se
reinicia solo si falla y **se recupera solo cuando el robot se cae y vuelve** (no hay
que relanzar nada a mano). Frigate ya se reinicia solo vía Docker (`restart:
unless-stopped`).

### Que arranque al bootear la PC (una vez, con sudo)
El servicio de usuario arranca al iniciar sesión. Para que además levante **al prender
la computadora sin que nadie loguee**, hay que habilitar *lingering* una sola vez:
```bash
sudo loginctl enable-linger $USER
```

### Ver / diagnosticar el servicio
```bash
systemctl --user status robot-nvr.service       # estado
journalctl --user -u robot-nvr.service -f        # logs en vivo
```

### Ver la cámara
- **NVR / monitoreo + grabaciones (recomendado):** `http://192.168.123.99:5000`
  (interfaz de Frigate, sin login)
- **En vivo directo en el navegador (mínima latencia):** `http://192.168.123.99:8889/robot` (WebRTC)
- **En VLC u otro NVR:** `rtsp://192.168.123.99:8554/robot`

> Desde **esta misma PC** podés usar `localhost` en vez de la IP.

### Apagar todo
```bash
./stop-all.sh
```

### Ver logs / diagnosticar
```bash
tail -f /tmp/robot-nvr-run.log         # el pipeline robot→RTSP
cd frigate && docker compose logs -f   # el NVR
```

---

## 5. Dónde se guarda el video

Frigate graba a disco dentro de la carpeta del proyecto:

```
frigate/media/
├── recordings/           ← grabación CONTINUA (esto es lo importante)
│   └── 2026-07-23/           por día
│       └── 16/                  por hora
│           └── robot/               por cámara
│               ├── 31.01.mp4           segmentos de video (minuto.segundo)
│               └── 40.21.mp4
├── clips/                ← miniaturas, previews y clips de eventos
└── exports/              ← exportaciones manuales que hagas desde la UI
```

- **Formato:** archivos `.mp4` (H.264), cortados en segmentos y ordenados
  `fecha/hora/cámara`.
- **Retención:** **3 días** de grabación continua (configurable). Después se borra solo
  lo más viejo.
- **Tamaño:** ~1–2,5 GB por hora a 1080p (o sea ~50–150 GB para 3 días). Hay 800 GB
  libres en el disco, pero si querés que ocupe menos, mirá la sección de configuración.
- La base de datos de eventos/metadata de Frigate también vive en `frigate/config/`.

---

## 6. Configuración (qué podés tocar)

### Cámara / grabación → `frigate/config/config.yml`
```yaml
record:
  enabled: true
  retain:
    days: 3          # ← cuántos días guardar
    mode: all        # all = grabación continua; motion = solo cuando hay movimiento (ocupa mucho menos)
detect:
  fps: 5             # ← cuadros por segundo que analiza la detección (más = más CPU)
```
Para que ocupe **mucho menos disco**, cambiá `mode: all` por `mode: motion`, o bajá
`days`. Después: `cd frigate && docker compose restart`.

### Pipeline → variables de entorno de `run.sh` / `start-all.sh`
| Variable | Default | Para qué |
|----------|---------|----------|
| `NIC` | `enp4s0` | Interfaz de red conectada al robot |
| `MAXFPS` | `0` (lo más rápido posible) | Limitar los cuadros por segundo de la captura |
| `ROBOT_IP` | `192.168.123.161` | IP del robot (solo para el chequeo de ping) |

### Puertos que se usan
| Puerto | Servicio |
|--------|----------|
| `5000` | Frigate — interfaz web (sin login) |
| `8971` | Frigate — interfaz web con usuario/contraseña |
| `8554` | mediamtx — RTSP (lo que consume un NVR) |
| `8888` | mediamtx — HLS (navegador) |
| `8889` | mediamtx — WebRTC (navegador, baja latencia) |

---

## 7. Estructura del proyecto

```
robot-nvr-bridge/
├── src/
│   ├── go2_jpeg_stream.cpp   ← captura JPEG del robot (camino que se usa)
│   └── go2_h264_stream.cpp   ← intento de H.264 nativo (NO funciona en este robot, ver §8)
├── setup.sh                  ← descarga mediamtx + ffmpeg (no versionados)
├── build.sh                  ← compila los programas C++
├── run.sh                    ← supervisor: mediamtx + captura + ffmpeg (auto-reinicio)
├── install-service.sh        ← instala el pipeline como servicio systemd (siempre prendido)
├── systemd/robot-nvr.service ← definición del servicio
├── start-all.sh / stop-all.sh← prende / apaga TODO (servicio + NVR)
├── mediamtx  + mediamtx.yml  ← servidor de streaming + su config
├── mediamtx.stock.yml        ← config completa de referencia de mediamtx
├── bin/ffmpeg  bin/ffprobe   ← binarios estáticos (no se instalan en el sistema)
└── frigate/
    ├── docker-compose.yml     ← cómo se levanta Frigate
    ├── config/config.yml      ← configuración del NVR (cámara, grabación, detección)
    └── media/                 ← acá se guardan las grabaciones
```

---

## 8. Una decisión técnica importante: por qué JPEG y no H.264 nativo

El Go2 **sí** genera H.264 por dentro y lo publica en un topic DDS
(`rt/frontvideostream`). En teoría eso permitiría "reempaquetar" ese H.264 directo a
RTSP **sin recodificar** — latencia mínima. Se probó y **en este robot no se entrega
bien**: por el puente ROS2/cyclonedds los datos llegan corruptos, y leyéndolo con el SDK
nativo el programa **empareja con el emisor pero nunca recibe un cuadro completo** (los
mensajes grandes no se reensamblan). Es la misma razón por la que el equipo de AI-VL
también lo evitó.

Por eso se usa el camino **JPEG** (`GetImageSample`): son mensajes chicos y confiables,
funcionan siempre. El costo es una recodificación JPEG→H.264 (con ajustes de baja
latencia igual queda fluido). El programa `go2_h264_stream` queda en el repo por si un
firmware futuro u otro modelo de robot lo arregla.

---

## 9. Requisitos y red

- **El robot tiene que estar online** (`ping 192.168.123.161`). Acá la conexión es
  intermitente: si el robot se cae, la cámara deja de responder y el stream queda vacío.
- **Red hacia quien mira:** hoy esta PC está **solo** en la red del robot
  (`192.168.123.x`, interfaz `enp4s0`). Quien quiera ver — NVR, celular, otra PC —
  tiene que estar en esa misma red, **o** hay que darle a esta PC una segunda interfaz
  (LAN/Wi-Fi de la oficina) y usar esa IP en las URLs. mediamtx y Frigate ya escuchan en
  todas las interfaces.
- **DDS:** `run.sh` exporta `CYCLONEDDS_URI` apuntando a `enp4s0`. Sin eso, el SDK no
  recibe nada.
- **Solo Go2:** usa la API de video del Go2. Un G1 necesitaría otra fuente de imagen.

---

## 10. Problemas comunes

| Síntoma | Causa probable / solución |
|---------|---------------------------|
| El video está negro / "no frames" | El robot está apagado o fuera de red → `ping 192.168.123.161` |
| No abre `:5000` desde otra PC | Esa PC no está en la red `192.168.123.x` (ver §9) |
| ffmpeg "Broken pipe" al arrancar | Quedó un mediamtx viejo ocupando el puerto → `./stop-all.sh` y volver a `./start-all.sh` |
| El disco se llena | Bajá `retain.days` o poné `mode: motion` en `frigate/config/config.yml` |
| Frigate no levanta la cámara | `cd frigate && docker compose logs -f` para ver el error de config |
