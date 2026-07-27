# Arquitectura y flujo del robot-nvr-bridge

Documento completo de **cómo funciona todo**: de dónde sale el video, por dónde
pasa, dónde se procesa, dónde se guarda, qué tecnología usa cada parte y por qué.

---

## 1. Idea general en una frase

El robot Unitree **no es una cámara IP** (no tenés una URL de video para abrir). El
video sale por el bus interno de Unitree (**DDS**). Este proyecto **lee ese video, lo
convierte a un stream estándar (RTSP/H.264) y se lo entrega a un NVR (Frigate)** que lo
muestra en vivo y lo graba. Todo corre en la PC (`192.168.123.99`), aparte del stack
AI-VL — no lo toca.

---

## 2. El flujo completo, de punta a punta

```
┌───────────┐   JPEG por DDS    ┌──────────────────┐  MJPEG   ┌─────────┐  H.264   ┌──────────┐  RTSP   ┌─────────┐
│  ROBOT    │ ───(videohub)───▶ │ go2_jpeg_stream  │ ─(pipe)─▶│ ffmpeg  │ ───────▶ │ mediamtx │ ──────▶ │ Frigate │
│ Go2 / G1  │  GetImageSample   │  (C++, propio)   │  stdout  │ (encode)│          │ (server) │  :8554  │  (NVR)  │
└───────────┘  api_id 1001      └──────────────────┘          └─────────┘          └──────────┘         └─────────┘
  192.168.123.x                  usa unitree_sdk2              -vsync cfr -r 15      :8554 RTSP           graba + web
  (DDS dominio 0)                                                                    :8888 HLS            UI :5000
                                                                                     :8889 WebRTC
```

### Paso a paso

1. **El robot** (Go2 o G1) tiene un servicio DDS llamado **`videohub`**. Cuando le
   mandás el pedido `GetImageSample` (api id **1001**), te devuelve **una foto JPEG**
   de su cámara (~180–240 KB, 1920×1080). Es request/response: pedís una foto, te la
   da. Los dos robots responden a este mismo servicio.

2. **`go2_jpeg_stream`** (programa propio en C++, `src/go2_jpeg_stream.cpp`) hace ese
   pedido en loop, lo más rápido que el robot contesta. Cada JPEG que llega lo escribe
   por su **salida estándar** (stdout), uno atrás de otro → un stream **MJPEG**.
   - Descarta frames byte-idénticos consecutivos (si el robot devuelve la misma foto
     cacheada) para no alimentar duplicados.
   - Si pasan ~8 s sin frames (robot caído), **sale con error** a propósito para que el
     supervisor reinicie la cadena (auto-sanación).

3. **`ffmpeg`** toma ese MJPEG por un **pipe**, lo **codifica a H.264** y lo **empuja
   por RTSP** a mediamtx. Flags clave:
   - `-c:v libx264 -preset ultrafast -tune zerolatency` → baja latencia.
   - `-vsync cfr -r 15` → **fuerza 15 fps constantes de salida**. Esto es
     importante: los JPEG llegan con un ritmo irregular (sobre todo el G1) y sin esto
     ffmpeg se traba y mediamtx lo echa por timeout (ver §7).
   - `-g 15` → un keyframe por segundo, para que el que se conecta empiece a ver rápido.

4. **`mediamtx`** es un **servidor de streaming** (un solo binario). Recibe el RTSP de
   ffmpeg en el path `robot` y lo re-publica en varios protocolos para quien lo quiera
   consumir:
   - **RTSP** en `:8554` → lo que usan los NVR.
   - **HLS** en `:8888` → para ver en el navegador.
   - **WebRTC** en `:8889` → ver en el navegador con mínima latencia.

5. **Frigate** (el NVR) se conecta al RTSP de mediamtx (`rtsp://127.0.0.1:8554/robot`),
   **muestra el video en vivo en una web y lo graba a disco**. Corre en Docker. También
   hace detección de objetos por CPU.

**¿Por qué dos servidores (mediamtx + Frigate)?** mediamtx desacopla la fuente del
consumidor: podés ver el video en VLC o el navegador **aunque Frigate esté apagado**, y
podés enchufar otro NVR distinto sin tocar la captura.

---

## 3. Tecnologías y por qué se usa cada una

| Componente | Qué es | Por qué está |
|------------|--------|--------------|
| **Unitree SDK2** (C++) | SDK oficial del robot | Única forma de leer la cámara: el video va por DDS, no por URL. Nuestro programa se **compila contra** su lib (`libunitree_sdk2.a`) |
| **DDS / CycloneDDS** | Bus de mensajería en tiempo real | Es el transporte interno del robot; el SDK habla DDS por debajo |
| **`go2_jpeg_stream`** | Programa propio (C++) | El "adaptador" robot→ffmpeg: pide los JPEG y los vuelca como stream |
| **FFmpeg** | Suite de video | Codifica JPEG→H.264 y publica por RTSP. Va como binario **estático** en `bin/` (no ensucia el sistema) |
| **MediaMTX** | Servidor RTSP/HLS/WebRTC | Recibe el stream y lo reparte en protocolos estándar. Un solo binario, sin instalar |
| **Frigate** | NVR open-source | Muestra en vivo, **graba**, línea de tiempo, detección. Corre en Docker |
| **Docker / Compose** | Contenedores | Frigate se levanta con `docker compose up -d`, sin instalar dependencias a mano |
| **systemd (user)** | Supervisor de servicios de Linux | Mantiene el pipeline **siempre prendido** (`Restart=always`) y lo arranca al iniciar sesión |
| **H.264 / RTSP** | Códec / protocolo | El "idioma" que entienden todos los NVR y reproductores |

---

## 4. La parte de video en el SDK (dónde vive exactamente)

Dentro del repo `unitree_sdk2`, la capacidad de video es una sola carpeta:

```
unitree_sdk2/include/unitree/robot/go2/video/
├── video_client.hpp   ← clase VideoClient, método GetImageSample(vector<uint8_t>&)
├── video_api.hpp       ← servicio "videohub", api id 1001
└── video_error.hpp
```

Nuestro código la usa en `src/go2_jpeg_stream.cpp`:
```cpp
#include <unitree/robot/go2/video/video_client.hpp>   // trae la parte de video
ChannelFactory::Instance()->Init(0, nic);             // conecta al DDS del robot (dominio 0, enp4s0)
go2::VideoClient vc;  vc.Init();                       // cliente de video
int r = vc.GetImageSample(img);                        // <-- ACA se saca cada frame (JPEG)
```

> Nota: aunque el nombre dice "go2", **el G1 también responde a `videohub`/`GetImageSample`**,
> así que el mismo programa sirve para los dos robots (uno a la vez — ver el doc
> `DOS-ROBOTS.md`).

---

## 5. Dónde se guarda el video

Frigate graba a disco dentro del proyecto:

```
frigate/media/
├── recordings/           ← grabación CONTINUA (lo importante)
│   └── 2026-07-27/           por día
│       └── 16/                  por hora
│           └── robot/               por cámara
│               └── 34.21.mp4          segmentos (minuto.segundo)
├── clips/                ← miniaturas, previews, clips de eventos
└── exports/              ← exportaciones manuales desde la UI
```

- **Formato:** `.mp4` (H.264), en segmentos ordenados `fecha/hora/cámara`.
- **Retención:** 3 días de grabación continua (config en `frigate/config/config.yml`).
- **Tamaño:** ~1–2,5 GB por hora a 1080p. Para ocupar menos: bajar `retain.days` o poner
  `mode: motion` (graba solo con movimiento).

---

## 6. Siempre prendido (el servicio systemd)

El pipeline corre como **servicio systemd de usuario** `robot-nvr.service`
(`systemd/robot-nvr.service`, se instala con `install-service.sh`):

- `Restart=always` → si algo falla, systemd lo revive.
- `run.sh` es un **supervisor**: mantiene `mediamtx` fijo y **reinicia sola** la cadena
  de captura+encode cuando se corta (robot que se cae y vuelve).
- Arranca al iniciar sesión. Para que arranque **al bootear la PC sin login**:
  `sudo loginctl enable-linger $USER` (una vez).
- Frigate se reinicia solo por Docker (`restart: unless-stopped`).

Gestión:
```
./start-all.sh     # prende todo (servicio + Frigate)
./stop-all.sh      # apaga todo
./status.sh        # estado / health (uptime, procesos, stream, robot, grabaciones)
systemctl --user status robot-nvr.service
journalctl --user -u robot-nvr.service -f
```

---

## 7. Detalle técnico importante: el fix de `-vsync cfr`

Sin `-vsync cfr -r 15`, ffmpeg usa los timestamps de llegada (wallclock) de cada JPEG.
El G1 entrega los frames con un **ritmo irregular** (frames grandes, timing dispar), y
ffmpeg **descarta frames** y **deja de emitir** hacia mediamtx. mediamtx entonces echa
al publisher con **`i/o timeout`** a los ~10 s → el stream se cae aunque la captura
siga. El Go2 zafaba por tener un ritmo más parejo. Forzar **tasa constante de salida**
(`-vsync cfr -r 15`) normaliza la cadencia y mantiene viva la publicación.

---

## 8. Puertos

| Puerto | Servicio |
|--------|----------|
| `5000` | Frigate — interfaz web (sin login) |
| `8971` | Frigate — interfaz web con usuario/contraseña |
| `8554` | mediamtx — **RTSP** (lo que consume un NVR / VLC) |
| `8888` | mediamtx — HLS (navegador) |
| `8889` | mediamtx — WebRTC (navegador, baja latencia) |
| `8000/8001` | mediamtx — RTP/RTCP (UDP, interno de RTSP) |

Cómo ver el video:
- **NVR / monitoreo + grabaciones:** `http://192.168.123.99:5000`
- **Navegador en vivo (baja latencia):** `http://192.168.123.99:8889/robot`
- **VLC u otro NVR:** `rtsp://192.168.123.99:8554/robot`

> Quien mire tiene que estar en la red `192.168.123.x` (hoy la PC solo tiene esa
> interfaz). Desde la misma PC, usá `localhost`.

---

## 9. Red y DDS

- Robot(s) en `192.168.123.x`; la PC en `192.168.123.99` (interfaz `enp4s0`).
- **DDS dominio 0** para todo. `run.sh` exporta `CYCLONEDDS_URI` fijando la interfaz
  `enp4s0` — **sin eso el SDK no recibe nada** (fue un detalle clave para que anduviera).
- Go2 = `.161`, G1 = `.164`.

---

## 10. Problemas conocidos (que NO son del NVR)

| Síntoma | Causa real |
|---------|-----------|
| El G1 corta el video a los minutos (también en la app de Unitree) | **Sobrecalentamiento del robot**: el Jetson del G1 llega a temperatura crítica (~104 °C) y **se apaga solo**. Es hardware/térmico del robot — revisar ventilador/rejillas, enfriar. Nada del NVR lo arregla; el pipeline se recupera solo cuando el video vuelve |
| Con los dos robots conectados, se mezclan los frames (perro y humano) | Los dos publican en el **mismo dominio DDS con los mismos topics** → DDS los fusiona. Ver el documento `DOS-ROBOTS.md` |
| Video negro / "no frames" | Robot apagado o fuera de red |
| No abre `:5000` desde otra PC | Esa PC no está en la red `192.168.123.x` |

---

## 11. Estructura del proyecto

```
robot-nvr-bridge/
├── src/go2_jpeg_stream.cpp   ← captura JPEG del robot (camino que se usa)
├── src/go2_h264_stream.cpp   ← intento de H.264 nativo (no funciona en este robot)
├── setup.sh                  ← descarga mediamtx + ffmpeg (no versionados en git)
├── build.sh                  ← compila los programas C++
├── run.sh                    ← supervisor: mediamtx + captura + ffmpeg (auto-reinicio)
├── install-service.sh        ← instala el servicio systemd (siempre prendido)
├── systemd/robot-nvr.service ← definición del servicio
├── start-all.sh / stop-all.sh / status.sh
├── mediamtx + mediamtx.yml   ← servidor de streaming + su config
├── bin/ffmpeg  bin/ffprobe   ← binarios estáticos
├── frigate/
│   ├── docker-compose.yml     ← cómo se levanta Frigate
│   ├── config/config.yml      ← config del NVR (cámara, grabación, detección)
│   └── media/                 ← acá se guardan las grabaciones
└── docs/
    ├── ARQUITECTURA.md        ← este documento
    └── DOS-ROBOTS.md          ← cómo separar los dos robots
```
