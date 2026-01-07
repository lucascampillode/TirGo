<div align="center">

# tirgo_bringup
**Bringup y orquestación de launch files (ROS 1 Noetic) para TirGoPharma**

Este paquete agrupa los **launch files ROS** necesarios para levantar
subconjuntos coherentes del sistema **TirGoPharma** en distintos modos
(de desarrollo, despliegue o pruebas).

 **Este paquete NO es el punto de entrada oficial de la demo final.**  
Para la ejecución integrada y reproducible del sistema completo se utiliza
el script de raíz **`tirgo_ALL.sh`**.

</div>


---

## 1. Rol del paquete dentro del proyecto

`tirgo_bringup` ocupa un **nivel intermedio** dentro de la jerarquía de arranque
del proyecto TirGoPharma.

### Jerarquía real de arranque

| Nivel | Elemento                     | Rol |
|------:|------------------------------|-----|
| 1 | `tirgo_ALL.sh` | **Punto de entrada oficial de la demo final** |
| 2 | Docker / docker-compose | Reproducibilidad del entorno |
| 3 | **tirgo_bringup** | Orquestación de launch files ROS |
| 4 | Launch files por paquete | Arranque fino y específico |

Este paquete **no sustituye** a los scripts de raíz ni a Docker.
Su objetivo es facilitar **desarrollo, pruebas e integración parcial** del sistema ROS.

---

## 2. Qué hace este paquete

### Qué hace

- Agrupa nodos ROS en **launch files coherentes**
- Permite levantar el sistema:
  - en modo **DEV**
  - en modo **DEPLOY**
  - de forma **modular** (activar/desactivar subsistemas)
- Centraliza parámetros comunes (timeouts, puertos, flags)

### Qué NO hace

- No lanza el **dispensador físico** (`servo_dispenser`)
- No levanta **MongoDB** ni infraestructura
- No reemplaza scripts de arranque del repositorio
- No es obligatorio para la demo final

---

## 3. Arquitectura a alto nivel

### Componentes principales

```mermaid
flowchart LR
  UI["tirgo_ui (Flask)"]
  AS["/tirgo/mission (ActionServer)\n(tirgo_mission_server)"]
  MOVE["move (navegación)"]
  DISP["servo_dispenser (RPi 3B)"]
  ARM["tirgo_tiago_arm_seq"]
  TTS["tiago_speech_node"]
  STT["stt_vosk"]

  UI --> AS
  AS --> MOVE
  AS --> DISP
  DISP --> AS
  AS --> ARM
  ARM --> AS
  AS --> TTS
  STT --> UI
````

---

## 4. Launch files disponibles

> Ubicación: `tirgo_bringup/launch/`

| Launch                    | Qué levanta                                              | Cuándo usar         |
| ------------------------- | -------------------------------------------------------- | ------------------- |
| `tirgo_all_dev.launch`    | MissionServer + Web + HRI                                | Desarrollo rápido   |
| `tirgo_all_deploy.launch` | Igual que DEV, con configuración por entorno             | Demo estable        |
| `tirgo_all.launch`        | Núcleo + módulos opcionales (`use_web/use_hri/use_move`) | Integración modular |
| `tirgo_core.launch`       | Solo `tirgo_mission_server`                              | Testing / debug     |
| `tirgo_web.launch`        | Solo `tirgo_ui`                                          | Web standalone      |
| `tirgo_hri.launch`        | STT + TTS                                                | HRI standalone      |

---

## 5. Quickstart

### 5.1 DEV rápido (PC / Docker)

**Opción recomendada (scripts del repositorio):**

```bash
./tirgo_stack.sh
```

**Opción manual (ROS ya activo):**

```bash
source /opt/ros/noetic/setup.bash
source ~/carpeta_compartida/ros_ws/devel/setup.bash

roslaunch tirgo_bringup tirgo_all_dev.launch
```

La interfaz web estará disponible, por defecto, en `http://localhost:9001`.

---

### 5.2 DEPLOY (variables de entorno)

`tirgo_all_deploy.launch` toma valores por defecto desde el entorno:

```bash
export TIRGO_WEB_PORT=9001
export FLASK_SECRET_KEY="cambia-esta-clave"
export TIRGO_PEPPER="cambia-este-pepper-largo"
export MONGO_URI="mongodb://tirgo_user:PASS@tirgo_mongo:27017/tirgo?authSource=tirgo"

roslaunch tirgo_bringup tirgo_all_deploy.launch
```

Asegúrate de que `MONGO_URI` coincide con las credenciales creadas en
`infra/tirgo_db_stack`.

---

### 5.3 ALL modular (activar/desactivar grupos)

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

---

## 6. Modo sin hardware

Este modo permite validar la lógica de misión **sin robot ni Raspberry Pi**.

1. Lanza el sistema (por ejemplo, `tirgo_all_dev.launch`).
2. Inicia una misión desde la web.
3. Simula los eventos publicando flags manualmente:

```bash
rostopic pub -1 /tirgo/tiago/arrived std_msgs/Bool "data: true"
rostopic pub -1 /tirgo/dispense/ready std_msgs/Bool "data: true"
rostopic pub -1 /tirgo/tiago/picked std_msgs/Bool "data: true"
rostopic pub -1 /tirgo/tiago/at_patient std_msgs/Bool "data: true"
rostopic pub -1 /tirgo/tiago/delivered std_msgs/Bool "data: true"
rostopic pub -1 /tirgo/tiago/farewell_done std_msgs/Bool "data: true"
```

El `tirgo_mission_server` debería finalizar la misión en estado **SUCCESS**.

---




## 7. Troubleshooting

### La web no conecta a Mongo

* Revisa `MONGO_URI` (host, usuario, contraseña, `authSource`).
* En Docker, el host suele ser `tirgo_mongo`.
* En local, normalmente `127.0.0.1`.

---

### No funciona STT / micrófono

* El dispositivo de audio puede cambiar entre equipos.
* En Docker, el acceso al audio depende del host.

Consulta el README de `stt_vosk`.

---

### `use_move=true` provoca errores

Actualmente no existe un `tirgo_move.launch` dentro de `tirgo_bringup`.

Solución manual:

```bash
roslaunch move rviz.launch
rosrun move publish_initial_pose.py
rosrun move checkpointfollower.py
rosrun move comunication_move.py
```

---

### La misión se queda esperando

El `tirgo_mission_server` está esperando alguno de estos flags:

* `/tirgo/tiago/arrived`
* `/tirgo/dispense/ready`
* `/tirgo/tiago/picked`
* `/tirgo/tiago/at_patient`
* `/tirgo/tiago/delivered`
* `/tirgo/tiago/farewell_done`

Usa `rostopic echo` para verificar su publicación o simúlalos manualmente.
