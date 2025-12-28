<div align="center">

# tirgo_bringup
**Bringup y orquestación de lanzadores (ROS 1 Noetic) para TirGoPharma**

Este paquete agrupa los **launch files** necesarios para levantar el sistema TirGoPharma en modo:
- **DEV** (rápido para iterar)
- **DEPLOY** (con configuración por variables de entorno)
- **ALL** (modular por grupos: core / web / hri / move)

</div>

---

## Índice
- [1. Qué hace este paquete](#1-qué-hace-este-paquete)
- [2. Arquitectura a alto nivel](#2-arquitectura-a-alto-nivel)
- [3. Launch files disponibles](#3-launch-files-disponibles)
- [4. Quickstart](#4-quickstart)
  - [4.1 DEV rápido (PC / Docker)](#41-dev-rápido-pc--docker)
  - [4.2 DEPLOY (variables de entorno)](#42-deploy-variables-de-entorno)
  - [4.3 ALL modular (activar/desactivar grupos)](#43-all-modular-activar-desactivar-grupos)
- [5. Modo sin hardware](#5-modo-sin-hardware)
- [6. Variables y parámetros](#6-variables-y-parámetros)
- [7. Troubleshooting](#7-troubleshooting)

---

## 1. Qué hace este paquete

`tirgo_bringup` contiene los `*.launch` que agrupan los nodos de:

- **Núcleo de misión**: `tirgo_mission_server` (ActionServer `/tirgo/mission`)
- **Web UI**: `tirgo_ui` (Flask + MongoDB + puente ROS)
- **HRI (voz)**: `stt_vosk` (STT offline) + `tiago_speech_node` (TTS TIAGo)
- **Movimiento (move)**: opcional

> Nota: el **dispensador físico** (`servo_dispenser`) NO se lanza aquí porque se asume en **Raspberry Pi 3B** (ROS_MASTER_URI apuntando al PC/robot).

---

## 2. Arquitectura a alto nivel

### 2.1 Componentes principales

```mermaid
flowchart LR
  UI["tirgo_ui (Flask)"]
  AS["/tirgo/mission (ActionServer)\n(tirgo_mission_server)"]
  MOVE["move (navegación)\ncheckpointfollower + communication_move"]
  DISP["servo_dispenser (RPi 3B)\nservos + pigpio"]
  ARM["brazo_robot\npick / deliver"]
  TTS["tiago_speech_node\n/tts (pal_interaction_msgs)"]
  STT["stt_vosk\n/stt/text"]

  UI -->|"Action goal"| AS

  AS -->|"topic: /tirgo/mission/start"| MOVE

  AS -->|"topic: /tirgo/dispense/request (Int32)"| DISP
  DISP -->|"topic: /tirgo/dispense/ready (Bool)"| AS

  AS -->|"flags (Bool)"| ARM
  ARM -->|"topics: /tirgo/tiago/picked, /tirgo/tiago/delivered"| AS

  AS -->|"topic: /tirgo/say (String)"| TTS

  STT --> UI

````

### 2.2 Secuencia “feliz” de la misión

```mermaid
sequenceDiagram
  participant Web as tirgo_ui
  participant MS as tirgo_mission_server (/tirgo/mission)
  participant Move as move
  participant Disp as servo_dispenser (RPi)
  participant Arm as brazo_robot
  participant TTS as tiago_speech_node

  Web->>MS: Goal(patient_id, med_id)
  MS->>Move: /tirgo/mission/start="start"
  Move-->>MS: /tirgo/tiago/arrived=True
  MS->>Disp: /tirgo/dispense/request=med_id
  Disp-->>MS: /tirgo/dispense/ready=True
  Arm-->>MS: /tirgo/tiago/picked=True
  Move-->>MS: /tirgo/tiago/at_patient=True
  Arm-->>MS: /tirgo/tiago/delivered=True
  TTS-->>MS: /tirgo/tiago/farewell_done=True
  MS-->>Web: Result=SUCCESS
```

---

## 3. Launch files disponibles

> Ubicación: `tirgo_bringup/launch/`

| Launch                    | Para qué sirve                                                                       | Recomendado para          |
| ------------------------- | ------------------------------------------------------------------------------------ | ------------------------- |
| `tirgo_all_dev.launch`    | Stack completo “DEV”: STT + voz TIAGo + MissionServer + Web                          | Iterar rápido             |
| `tirgo_all_deploy.launch` | Stack “DEPLOY”: igual que DEV, pero con defaults por **variables de entorno**        | Demo estable / despliegue |
| `tirgo_all.launch`        | Stack “ALL modular”: incluye `tirgo_core.launch` + grupos `use_web/use_hri/use_move` | Encender/apagar piezas    |
| `tirgo_core.launch`       | Solo el núcleo (`tirgo_mission_server`) + parámetros de timeouts                     | Testing / integración     |
| `tirgo_web.launch`        | Solo `tirgo_ui` con sus args (puerto, mongo_uri, vídeo…)                             | Web standalone            |
| `tirgo_hri.launch`        | Solo voz: `stt_vosk` + `tiago_speech.launch`                                         | HRI standalone            |

---

## 4. Quickstart

### 4.1 DEV rápido (PC / Docker)

**Opción A: usando tus scripts del repo (recomendado)**

```bash
./tirgo_stack.sh
```

Esto levanta:

* MongoDB (stack infra)
* Contenedor `ros1_rob_tirgo`
* Y dentro, lo típico es arrancar bringup con tu script dev.

**Opción B: manual**

```bash
# En el contenedor / PC con ROS Noetic
source /opt/ros/noetic/setup.bash
source ~/carpeta_compartida/ros_ws/devel/setup.bash

roslaunch tirgo_bringup tirgo_all_dev.launch
```

**Web**: por defecto en `http://localhost:9001`

---

### 4.2 DEPLOY (variables de entorno)

`tirgo_all_deploy.launch` lee defaults desde entorno (ideal para `.env` o para docker compose).

```bash
export TIRGO_WEB_PORT=9001
export FLASK_SECRET_KEY="cambia-esta-clave"
export TIRGO_PEPPER="cambia-este-pepper-largo"
export TIRGO_HOTWORD="hola tirgo"
export MONGO_URI="mongodb://tirgo_user:CAMBIA_PASS@tirgo_mongo:27017/tirgo?authSource=tirgo"
export TIRGO_USE_VIDEO=true
export TIRGO_VIDEO_PORT=8080

roslaunch tirgo_bringup tirgo_all_deploy.launch
```

**Importante**: asegúrate de que `MONGO_URI` coincide con el usuario/clave creados en `infra/tirgo_db_stack`.

* En algunos sitios del repo aparece `tirgo_app:tirgo@127.0.0.1...`
* En deploy, el default apunta a `tirgo_user:CAMBIA_PASS@tirgo_mongo...`


---

### 4.3 ALL modular (activar/desactivar grupos)

`tirgo_all.launch` permite encender por bloques:

```bash
roslaunch tirgo_bringup tirgo_all.launch \
  use_web:=true \
  use_hri:=true \
  use_move:=false
```

Argumentos principales:

* `use_web` (default: `true`)
* `use_hri` (default: `true`)
* `use_move` (default: `true`)
* Config web: `web_port`, `mongo_uri`, `flask_debug`, etc.

---

## 5. Modo sin hardware

Si quieres demo “sin robot / sin RPi”, puedes simular la misión publicando flags.

### 5.1 Levantar solo misión + web

```bash
roslaunch tirgo_bringup tirgo_all_dev.launch
# (o tirgo_all.launch use_move:=false use_hri:=false si quieres ultra-minimal)
```

### 5.2 Simular los flags en orden

En otra terminal:

```bash
# 1) simular llegada al dispensador
rostopic pub -1 /tirgo/tiago/arrived std_msgs/Bool "data: true"

# 2) simular que el dispensador está listo
rostopic pub -1 /tirgo/dispense/ready std_msgs/Bool "data: true"

# 3) simular recogida con brazo
rostopic pub -1 /tirgo/tiago/picked std_msgs/Bool "data: true"

# 4) simular llegada al paciente
rostopic pub -1 /tirgo/tiago/at_patient std_msgs/Bool "data: true"

# 5) simular entrega
rostopic pub -1 /tirgo/tiago/delivered std_msgs/Bool "data: true"

# 6) simular despedida completada
rostopic pub -1 /tirgo/tiago/farewell_done std_msgs/Bool "data: true"
```

Con esto, el ActionServer debería terminar la misión como **SUCCESS** (si has lanzado la misión desde la web).

---

## 6. Variables y parámetros

### 6.1 Args típicos de la web (`tirgo_ui/web.launch`)

Estos args se pasan desde `tirgo_web.launch` y desde los `all_*`:

* `port` (default: `9001`)
* `flask_secret_key` (default: `dev-secret`)
* `flask_debug` (default: `1`)
* `mongo_uri`
* `hotword` (default: `hola tirgo`)
* `stt_topic` (default: `/stt/text`)
* `pepper` (default: `cambia-esta-cadena-larga`)
* `use_video_server` (default: `true`)
* `video_port` (default: `8080`)

### 6.2 Parámetros del MissionServer (timeouts)

En `tirgo_core.launch` se definen parámetros como:

* `mission/timeout_arrive`
* `mission/timeout_ready`
* `mission/timeout_pick`
* `mission/timeout_patient`
* `mission/timeout_deliver`
* `mission/timeout_farewell`

Si estás en modo sin hardware, puedes subirlos o bajarlos según la demo.

---

## 7. Troubleshooting

### 7.1 “La web no conecta a Mongo” / auth fail

* Revisa `mongo_uri` (usuario/clave/host/authSource).
* Si estás en docker, probablemente el host sea `tirgo_mongo`.
* Si estás en local, probablemente `127.0.0.1`.

Consejo: usa `tirgo_all_deploy.launch` y fija `MONGO_URI` en tu `.env`.

---

### 7.2 “No hay STT / no pilla micro”

`stt_vosk` depende del dispositivo de audio correcto.

* En distintos PCs, el `device_index` puede cambiar.
* En docker, audio puede ser “la fiesta del caos”.

Mira el README de `stt_vosk` y ajusta el launch de ese paquete.

---

### 7.3 “use_move=true rompe al lanzar”

Ahora mismo falta el launch `tirgo_move.launch` en `tirgo_bringup`.

Workaround manual (dentro de ROS Noetic con tu ws):

```bash
# (opcional) RViz y mapa si lo usas
roslaunch move rviz.launch

# nodos de move (orden típico)
rosrun move publish_initial_pose.py
rosrun move checkpointfollower.py
rosrun move comunication_move.py
```

---

### 7.4 “La misión se queda esperando para siempre”

El MissionServer está esperando alguno de estos flags (Bool):

* `/tirgo/tiago/arrived`
* `/tirgo/dispense/ready`
* `/tirgo/tiago/picked`
* `/tirgo/tiago/at_patient`
* `/tirgo/tiago/delivered`
* `/tirgo/tiago/farewell_done`

Usa `rostopic echo` para ver si se publican, o simúlalos (modo sin hardware).
