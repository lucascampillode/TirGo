# Dispensador (ROS1 Noetic + Raspberry Pi + Pigpio)

Nodo ROS para controlar los servos del dispensador físico de TirGoPharma en la Raspberry Pi 3B.  
Escucha órdenes de dispensado y acciona los servos mediante `pigpio`.

---

## Topics ROS

### Suscribe
- `/tirgo/dispense/request` (`std_msgs/Int32`)  
  Recibe el ID del bote a dispensar:

  | ID | Servo | Movimiento | Bote |
  |----|-------|------------|------|
  | 1  | A     | +90°       | A    |
  | 2  | A     | -90°       | B    |
  | 3  | B     | +90°       | C    |
  | 4  | B     | -90°       | D    |

### Publica
- `/tirgo/dispense/ready` (`std_msgs/Bool`)  
  Publica `True` cuando se ha completado un ciclo de dispensado válido.

---

## Estructura del paquete

```bash
servo_dispenser/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── servo_dispenser_rpi.launch
├── scripts/
│   ├── servo_dispenser_node.py
│   └── setup_servo_rpi.sh
└── src/
    └──  # (reservado para lógica Python reutilizable, si se usa en el futuro)
````

---

## Requisitos

* Raspberry Pi 3B
* Ubuntu 20.04 + ROS **Noetic**
* Workspace TirGoPharma en `~/carpeta_compartida` usando tu entorno estándar:

  ```bash
  source /opt/ros/noetic/setup.bash
  source ~/carpeta_compartida/gallium/setup.bash
  source ~/carpeta_compartida/setup_env.sh
  ```
* Python 3.8
* Acceso GPIO (usuario en grupo `gpio`)
* Daemon `pigpiod` activo

---

## Instalación rápida en la Raspberry Pi

El paquete trae un script de setup que deja la Pi lista (pigpio, grupos, daemon, etc.).

### 1. Ejecutar script de setup

```bash
cd ~/carpeta_compartida/ros_ws/src/servo_dispenser/scripts

sudo ./setup_servo_rpi.sh
sudo reboot
```

El reboot es necesario para que los cambios de grupos (`gpio`, `dialout`, `video`) se apliquen al usuario.

### 2. Compilar el workspace con el entorno TirGoPharma

Tras el reinicio, en la Raspberry:

```bash
cd ~/carpeta_compartida/ros_ws

# Entorno unificado de ROS (Noetic + gallium + vars de TirGo)
source /opt/ros/noetic/setup.bash
source ~/carpeta_compartida/gallium/setup.bash
source ~/carpeta_compartida/setup_env.sh

catkin_make
source devel/setup.bash
```

Puedes comprobar que el paquete existe con:

```bash
rospack find servo_dispenser
```

---

## Ejecución

### 1. Asegurar que `roscore` está en marcha

Si el master está en la propia Raspberry:

```bash
# Entorno ROS completo
source /opt/ros/noetic/setup.bash
source ~/carpeta_compartida/gallium/setup.bash
source ~/carpeta_compartida/setup_env.sh

roscore
```

(Si el master está en otro PC, respeta igualmente este entorno y ajusta `ROS_MASTER_URI` / `ROS_HOSTNAME` en `setup_env.sh` o antes de lanzar.)

### 2. Lanzar el nodo del dispensador

En otra terminal de la Raspberry:

```bash
# Entorno ROS completo
source /opt/ros/noetic/setup.bash
source ~/carpeta_compartida/gallium/setup.bash
source ~/carpeta_compartida/setup_env.sh

# Workspace
cd ~/carpeta_compartida/ros_ws
source devel/setup.bash

# Opción recomendada: usar launch
roslaunch servo_dispenser servo_dispenser_rpi.launch

# Alternativa:
# rosrun servo_dispenser servo_dispenser_node.py
```

---

## Verificar funcionamiento

En otra terminal, siempre cargando el mismo entorno:

```bash
source /opt/ros/noetic/setup.bash
source ~/carpeta_compartida/gallium/setup.bash
source ~/carpeta_compartida/setup_env.sh
cd ~/carpeta_compartida/ros_ws
source devel/setup.bash
```

### Enviar una petición de dispensado

```bash
rostopic pub /tirgo/dispense/request std_msgs/Int32 "data: 1"
```

### Comprobar que el nodo recibe las peticiones

```bash
rostopic echo /tirgo/dispense/request
```

### Comprobar la señal de listo (`ready`)

```bash
rostopic echo /tirgo/dispense/ready
```

Deberías ver `data: True` tras cada dispensado válido.

---

## Notas

* Este nodo está pensado para ejecutarse **solo en la Raspberry Pi**, no en el Docker del portátil.
* La comunicación con el resto del sistema (UI, mission server, etc.) se hace únicamente vía topics ROS (`/tirgo/dispense/request` y `/tirgo/dispense/ready`).
* El script `scripts/setup_servo_rpi.sh` es idempotente: puedes reejecutarlo si cambias de usuario o tocas la configuración de la Pi.
* Siempre usa el entorno unificado:

  ```bash
  source /opt/ros/noetic/setup.bash
  source ~/carpeta_compartida/gallium/setup.bash
  source ~/carpeta_compartida/setup_env.sh
  ```

  antes de lanzar nodos o hacer `catkin_make`, para que todo esté en el mismo ROS space.
