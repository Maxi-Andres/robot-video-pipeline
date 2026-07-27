# Cómo tener los dos robots a la vez (Go2 + G1) como dos cámaras separadas

Documento de **investigación**: por qué hoy con los dos robots conectados se mezclan
en una sola cámara, qué opciones hay para separarlos, y qué implica cada una. El
objetivo es que en el NVR aparezcan **dos cámaras distintas** (una por robot), con
**historiales de grabación separados**.

---

## 1. El objetivo

```
Go2 (perro)  ─┐
              ├─▶  NVR  ─▶  camara "go2"  (grabación propia)
G1 (humano)  ─┘             camara "g1"   (grabación propia)
```

Hoy hay **una sola cámara** llamada `robot`. Queremos **dos**.

---

## 2. El problema (por qué se mezclan)

Los dos robots:
- Están en el **mismo dominio DDS (0)**.
- Publican con los **mismos nombres de topic** (`/api/videohub/response`,
  `/frontvideostream`, etc.).
- Los dos responden al **mismo servicio `videohub`** (`GetImageSample`, api 1001).

En DDS, cuando dos equipos publican el **mismo topic en el mismo dominio, se fusionan**:
un suscriptor (nuestro capturador) recibe los mensajes de **ambos mezclados**, sin
forma de saber cuál vino de cuál. Por eso:

- **Con uno conectado** anda perfecto (solo uno publica).
- **Con los dos**, los frames se intercalan → ves "unos frames del perro y unos del
  humano" en la misma cámara.

Es el mismo problema del lado de los **comandos**: el executor de AI-VL le publica al
Go2 y al G1 en topics que se pisan (`/api/sport/request`), así que también hay cross-talk
de control con los dos en el mismo dominio.

---

## 3. Evidencia que recolectamos

Con los dos robots conectados, medimos:

- **`/api/videohub/response` → Publisher count: 2.** Los dos robots publican video en
  el mismo topic.
- **`/frontvideostream` → Publisher count: 2.** Ídem el stream H.264.
- Cada robot es un **participante DDS distinto** (prefijos de GUID `75.9e…` vs `f6.17…`).
  → *técnicamente* son distinguibles a nivel DDS.
- **Los dos devuelven la MISMA resolución: 1920×1080.** (Medimos 50 frames; todos
  1080p.) → **no** se pueden separar por resolución.
- El mensaje de video (`unitree_api::msg::Response`) **no trae ningún identificador de
  robot** (ni serial, ni MAC, ni source id). El único campo de identidad es un número
  de correlación que ponemos nosotros en el pedido. → No hay forma de separar mirando
  el contenido del mensaje.
- El robot es Go2 = `192.168.123.161`, G1 = `192.168.123.164`, PC = `.99` en `enp4s0`.

**Conclusión:** el único discriminador confiable "en banda" es la **identidad del
participante DDS** (el GUID del que publica cada frame). Fuera de eso, hay que
**separarlos a nivel de transporte** (dominios distintos).

---

## 4. Opciones analizadas

### Opción A — Dominio DDS por robot (la más limpia a nivel red)

Poner cada robot en su **propio dominio DDS** (ej. G1 en 0, Go2 en 1). Ahí se corre
**una captura por dominio** → dos paths en mediamtx (`/go2`, `/g1`) → dos cámaras en
Frigate. Separación perfecta, cero mezcla, sin parsear frames.

**Lo crítico (dos cosas que hay que saber):**

1. **El dominio se cambia EN EL ROBOT, no en la PC.** Poner `ROS_DOMAIN_ID` /
   `CYCLONEDDS_URI` en la compu solo cambia **qué escucho yo**; el robot sigue
   publicando en 0 salvo que le cambies el dominio **a su computadora onboard**. En
   ninguno de los repos (unitree_ros2, unitree_sdk2, AI-VL) hay documentado cómo fijar
   el dominio propio del robot — es config del onboard de Unitree. **Si el firmware no
   lo permite, esta opción no sirve.** (Es lo primero a validar.)

2. **Rompe AI-VL para el robot que muevas.** Todo AI-VL (executor `:8090`, bridge de
   cámara `:8091`, backend) corre en **un solo dominio (0)** heredado de `setup.sh`, y
   `rclpy` es **un-dominio-por-proceso**. Con el perro en domain 1:
   - El executor de AI-VL no le manda comandos (los publica en domain 0).
   - El bridge de cámara de AI-VL no lo ve.
   - Para que AI-VL vuelva a manejarlo habría que correr **un segundo executor y un
     segundo bridge** en domain 1 y que el backend rutee por robot — un **refactor
     real** de AI-VL, no un toque de config.

**Costo:** cambio en el robot (si se puede) + potencial refactor de AI-VL.
**Beneficio:** la separación más simple y 100% confiable **del lado del NVR**.

### Opción B — Separar por resolución del frame — ❌ DESCARTADA

La idea era: una sola captura y separar los frames por tamaño de imagen (ej. Go2 1080p
vs G1 640p). **No sirve: los dos devuelven 1920×1080.** Tampoco por tamaño de bytes (se
superponen). Descartada por evidencia.

### Opción C — Demux por identidad del participante DDS (sin tocar los robots)

Cada robot es un participante DDS distinto. Se puede **bajar por debajo del SDK** y leer
el topic crudo `rt/api/videohub/response` con la **API C de CycloneDDS**, que por cada
muestra expone `dds_sample_info_t.publication_handle` →
`dds_get_matched_publication_data()` → **`participant_key` (GUID del robot)**. Con eso se
rutea cada frame al robot correcto → dos streams → dos cámaras. Todo esto **sin cambiar
el dominio ni tocar AI-VL** (todo sigue en domain 0).

**A favor:** no toca los robots ni AI-VL; separación determinística.
**En contra:**
- Es lo **más complejo de programar**: hay que reemplazar el `VideoClient` (que abstrae
  todo esto) por un suscriptor CycloneDDS crudo en C++, con manejo de `dds_take` +
  sample info + resolución del GUID del participante.
- Hay que **mapear qué GUID es el Go2 y cuál el G1** (los GUIDs son opacos). Se puede:
  fijar por primera vez y recordar, o correlacionar con quién publica `/frontvideostream`
  (Go2), o dejar que el usuario etiquete cuál es cuál la primera vez.
- Comparte el **ritmo de captura**: como es un solo `videohub` para los dos, cada cámara
  recibe ~la mitad de los frames.

Todas las piezas de la API existen en el CycloneDDS que trae el SDK
(`unitree_sdk2/thirdparty/include/dds/dds.h`), así que es factible — es cuestión de
esfuerzo de implementación.

### Opción D — Particiones / namespaces DDS (mención)

DDS soporta **partitions** o topics con namespace por robot, que evitarían el choque sin
cambiar de dominio. Pero **nada en estos repos lo implementa hoy**, y las particiones las
tendría que aplicar también el firmware del robot (que no expone esa config acá). Queda
como idea teórica, no práctica con lo que hay.

---

## 5. Comparación

| Opción | Toca el robot | Toca AI-VL | Complejidad | Confiabilidad | Estado |
|--------|:---:|:---:|:---:|:---:|--------|
| **A** Dominio por robot | **Sí** (si se puede) | **Sí** (refactor) | Baja (del lado NVR) | Alta | Depende de poder cambiar el dominio del robot |
| **B** Por resolución | No | No | Baja | — | ❌ Descartada (ambos 1080p) |
| **C** Demux por GUID DDS | No | No | **Alta** (C++/DDS crudo) | Alta | Factible, no implementado |
| **D** Particiones DDS | Sí (firmware) | Depende | Media | Alta | No soportado por lo que hay |

---

## 6. Cómo quedaría en el NVR (vale para A o C)

En cualquiera de los dos caminos viables, el NVR queda igual:

- **mediamtx:** dos paths, `robot` → `go2` y `g1`.
- **go2_jpeg_stream:** dos instancias (o una que demultiplexa, en la Opción C).
  - Opción A: una por dominio (`DDS_DOMAIN=0` → g1, `DDS_DOMAIN=1` → go2).
  - Opción C: una sola que rutea por GUID a `/go2` o `/g1`.
- **`run.sh`:** el supervisor levanta las dos cadenas de captura→ffmpeg.
- **Frigate `config.yml`:** dos cámaras:
  ```yaml
  cameras:
    go2:
      ffmpeg: { inputs: [{ path: rtsp://127.0.0.1:8554/go2, roles: [detect, record] }] }
      ...
    g1:
      ffmpeg: { inputs: [{ path: rtsp://127.0.0.1:8554/g1, roles: [detect, record] }] }
      ...
  ```

Frigate ya separa todo solo: **dos vistas en vivo** y **dos carpetas de grabación**
(`recordings/…/go2/` y `recordings/…/g1/`). Ese "historial separado" sale gratis una vez
que hay dos streams distintos.

---

## 7. Recomendación

Depende de dos preguntas que hay que responder primero:

1. **¿Se puede fijar el dominio DDS en el onboard del robot?**
   - **Sí** → **Opción A** es la más prolija, *si además* aceptás el impacto en AI-VL
     (o el robot movido lo usás solo para el NVR, no para control por AI-VL).
   - **No / no querés tocar el robot ni AI-VL** → **Opción C** (demux por GUID): más
     trabajo de programación, pero cero riesgo externo.

2. **¿El robot que muevas necesita seguir siendo controlado por AI-VL?**
   - **No** (solo verlo/grabarlo) → Opción A directa.
   - **Sí** → Opción A obliga a duplicar executor + bridge de cámara en AI-VL. En ese
     caso, quizá convenga la **Opción C** para no romper AI-VL.

**Camino sugerido para decidir rápido:** primero **probar si el firmware del robot deja
cambiar el dominio** (un test chico: cambiarlo en el robot y ver si publica en domain 1).
Si se puede y no te importa el impacto en AI-VL → Opción A. Si no → Opción C.

---

## 8. Pendientes / cosas a tener en cuenta

- **El sobrecalentamiento del G1 es un tema aparte** (ver `ARQUITECTURA.md` §10): el
  robot se apaga solo a ~104 °C. Con los dos robots grabando, sumás carga (y calor) a la
  fuente de video; conviene resolver la refrigeración antes de exigirle streaming
  sostenido, e incluso limitar los fps de captura (`MAXFPS`) para descargar al robot.
- **Opción A + AI-VL:** si algún día se hace, es un refactor multi-instancia (dos
  executors, dos bridges, ruteo por robot en el backend, dos WS de cámara). Documentarlo
  aparte cuando se encare.
- **Opción C:** requiere identificar y persistir el mapeo GUID→robot; pensar qué pasa si
  un robot se reinicia (nuevo GUID de participante).
- Nada de esto está implementado todavía: este documento es la **investigación previa**
  para elegir el camino.
