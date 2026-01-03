<div align="center">

# servo_dispenser

Nodo ROS de dispensación para **TirGoPharma** (ROS 1 — Noetic) ejecutado en una **Raspberry Pi 3B**.  
Controla los servos del dispensador físico usando `pigpio` y publica un flag `ready` cuando un ciclo de dispensado termina correctamente.

El paquete incluye scripts para preparar la Pi (`setup_servo_rpi.sh`) y un script unificado `run_servo.sh` que compila el workspace y lanza el nodo.

</div>

---

## Índice

- [Quickstart](#quickstart)
- [API ROS](#api-ros)
- [Estructura del paquete](#estructura-del-paquete)
- [Requisitos](#requisitos)
- [Instalación rápida en la Raspberry Pi](#instalación-rápida-en-la-raspberry-pi)
- [Script de arranque: run_servo.sh](#script-de-arranque-run_servosh)
- [Ejecución (alternativas)](#ejecución-alternativas)
- [Comprobación rápida (smoke test)](#comprobación-rápida-smoke-test)
- [Notas importantes](#notas-importantes)
- [Tests automatizados](#tests-automatizados)

---

## Quickstart

> 1) Preparar la Raspberry Pi (una sola vez)
```bash
cd ~/carpeta_compartida/ros_ws/src/servo_dispenser/scripts
sudo ./setup_servo_rpi.sh
sudo reboot
```

> 2) (Tras el reboot) Levantar ROS y lanzar el nodo con el script unificado
```bash
cd ~/carpeta_compartida/ros_ws/src/servo_dispenser
chmod +x run_servo.sh
./run_servo.sh
```

---

## API ROS

### Suscribe

* **`/tirgo/dispense/request`** (`std_msgs/Int32`)  
  Recibe el **ID del bote** a dispensar:

  | ID | Servo | Movimiento | Bote |
  | -- | ----- | ---------- | ---- |
  | 1  | A     | +90°       | A    |
  | 2  | A     | -90°       | B    |
  | 3  | B     | +90°       | C    |
  | 4  | B     | -90°       | D    |

### Publica

* **`/tirgo/dispense/ready`** (`std_msgs/Bool`)  
  Publica `True` cuando se ha completado un **ciclo de dispensado válido**.  
  En IDs inválidos, el nodo **no** mueve servos y **no** publica `ready`.

---

## Estructura del paquete

```bash
servo_dispenser/
├── CMakeLists.txt
├── package.xml
├── run_servo.sh                      # script para compilar + lanzar todo
├── launch/
│   └── servo_dispenser_rpi.launch
├── scripts/
│   ├── servo_dispenser_node.py       # nodo principal ROS
│   └── setup_servo_rpi.sh            # script de preparación de la Pi
├── src/
│   └──  # (reservado para lógica Python reutilizable)
└── test/
    ├── conftest.py
    ├── test_servo_dispenser.py
    ├── test_servo_dispenser_flow.py
    └── servo_dispenser_flow.test
```

---

## Requisitos

* Hardware
  * Raspberry Pi 3B (dispensador físico conectado a sus GPIO).

* Software
  * Ubuntu 20.04
  * ROS **Noetic**
  * Python **3.8**

* Entorno TirGoPharma en `~/carpeta_compartida`:

```bash
source /opt/ros/noetic/setup.bash
source ~/carpeta_compartida/gallium/setup.bash
source ~/carpeta_compartida/setup_env.sh
```

* GPIO / pigpio
  * Usuario en el grupo `gpio`
  * Daemon `pigpiod` activo

> Siempre que vayas a compilar o lanzar, asegúrate de tener el **entorno unificado** cargado.

---

## Instalación rápida en la Raspberry Pi

El paquete incluye un script que deja la Pi preparada (grupos, pigpio, servicios…).

### 1. Ejecutar script de setup

```bash
cd ~/carpeta_compartida/ros_ws/src/servo_dispenser/scripts

sudo ./setup_servo_rpi.sh
sudo reboot
```

El reinicio es necesario para que se apliquen los cambios de pertenencia a grupos (`gpio`, `dialout`, `video`, etc.).

### 2. Compilar el workspace TirGo

Tras el reboot:

```bash
cd ~/carpeta_compartida/ros_ws

source /opt/ros/noetic/setup.bash
source ~/carpeta_compartida/gallium/setup.bash
source ~/carpeta_compartida/setup_env.sh

catkin_make
source devel/setup.bash
```

Comprobar que el paquete está visible:

```bash
rospack find servo_dispenser
```

Si no devuelve ruta, algo se ha roto en el build o en el entorno.

---

## Script de arranque: run_servo.sh

El script `run_servo.sh`, se encuentra en la raíz del paquete, sirve para simplificar arrancar el servicio en la Pi. Este script:

1. Carga el entorno base de ROS y TirGo (si existen los archivos).
2. Entra en `~/carpeta_compartida/ros_ws` y ejecuta `catkin_make`.
3. Carga `devel/setup.bash`.
4. Lanza el `roslaunch` del nodo: `roslaunch servo_dispenser servo_dispenser_rpi.launch`.

Ruta: `~/carpeta_compartida/ros_ws/src/servo_dispenser/run_servo.sh`

### Uso básico

Dar permisos y ejecutar:

```bash
cd ~/carpeta_compartida/ros_ws/src/servo_dispenser
chmod +x run_servo.sh
./run_servo.sh
```

El script muestra mensajes informativos y se detiene si algún comando falla (`set -e`).

### Notas sobre permisos

- El script no requiere `sudo` para ejecutar `catkin_make` ni `roslaunch` en condiciones normales.  
- Asegúrate de ejecutar como el usuario correcto (usuario que pertenece a `gpio` y al que está configurado en `setup_env.sh`).

---

## Ejecución (alternativas)

Asegurar de que `ROS_MASTER_URI` y `ROS_HOSTNAME` están bien configurados en la Pi (normalmente ya lo gestiona tu `setup_env.sh`).

```bash
cd ~/carpeta_compartida/ros_ws

source /opt/ros/noetic/setup.bash
source ~/carpeta_compartida/gallium/setup.bash
source ~/carpeta_compartida/setup_env.sh
source devel/setup.bash

roslaunch servo_dispenser servo_dispenser_rpi.launch
# alternativamente (solo el nodo):
# rosrun servo_dispenser servo_dispenser_node.py
```

---

## Comprobación rápida (smoke test)

Primero, carga el entorno (si no lo tienes ya):

```bash
cd ~/carpeta_compartida/ros_ws

source /opt/ros/noetic/setup.bash
source ~/carpeta_compartida/gallium/setup.bash
source ~/carpeta_compartida/setup_env.sh
source devel/setup.bash
```

### 1. Enviar una petición de dispensado

```bash
rostopic pub /tirgo/dispense/request std_msgs/Int32 "data: 1"
```

### 2. Ver qué llega al nodo

```bash
rostopic echo /tirgo/dispense/request
```

Deberías ver el `data: 1` que acabas de publicar.

### 3. Ver la señal de `ready`

```bash
rostopic echo /tirgo/dispense/ready
```

Tras el movimiento del servo, deberías ver:

```text
data: True
```

para un ID válido. Para IDs inválidos, no debería aparecer ningún `ready`.

---

## Notas importantes

* Este nodo se ejecuta **solo en la Raspberry Pi**, **nunca** dentro de Docker.
* La comunicación con el resto del sistema TirGoPharma es únicamente vía **tópicos ROS**.

---

## Tests automatizados

El paquete incluye dos tipos de tests:

1. **Tests unitarios (`pytest`)**  
   No requieren ROS ni Raspberry; se puede ejecutar en el portátil.

2. **Tests de flujo completo (`rostest`)**  
   Se ejecutan en la Raspberry Pi, contra `pigpio` real y el nodo corriendo.

Estos tests validan tanto la lógica del nodo como el comportamiento físico del dispensador.

---

### Estructura de tests

```text
servo_dispenser/test/
├── conftest.py                     ← fixtures comunes
├── test_servo_dispenser.py         ← unitarios (pytest)
├── test_servo_dispenser_flow.py    ← test de integración / flujo
└── servo_dispenser_flow.test       ← definición de rostest
```

---

### Tests unitarios (pytest)

Estos tests **no** usan ROS ni GPIO: se apoya en clases fake como **`FakePi`** y **`DummyPublisher`**.

Qué se comprueba:

* Conversión de ángulo → microsegundos (siempre dentro de rango).
* Selección correcta del servo según `bin_id`.
* Llamadas correctas a `set_servo_pulsewidth` con los parámetros esperados.
* Publicación de `ready=True` en ciclos de dispensado válidos.
* Que con `bin_id` inválido:
  * No se muevan servos.
  * No se publique `ready`.

Cómo ejecutarlos:

```bash
cd ~/carpeta_compartida/ros_ws/src/servo_dispenser
pytest -q
```

Se pueden pasar en local, sin Raspberry y sin `pigpiod`.

---

### Tests de flujo completo (rostest)

Estos tests lanzan el nodo real en la Raspberry y usan ROS para mandar pedidos y observar la respuesta.

Qué se comprueba:

* Publicar un `Int32` válido en `/tirgo/dispense/request` mueve el **servo correcto**.
* Tras el movimiento, se publica **exactamente un** `ready=True`.
* IDs inválidos → el nodo no mueve servos y no publica `ready`.
* El nodo no se cuelga ni casca durante la prueba.

Requisitos:

* Raspberry Pi 3B con `pigpiod` arrancado.
* Workspace compilado (`catkin_make` sin errores).
* Entorno cargado:

```bash
source /opt/ros/noetic/setup.bash
source ~/carpeta_compartida/gallium/setup.bash
source ~/carpeta_compartida/setup_env.sh
```

Cómo ejecutarlos:

```bash
cd ~/carpeta_compartida/ros_ws
rostest servo_dispenser servo_dispenser_flow.test
```

Salida esperada en un caso OK:

```text
SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0
```
