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
````

> 2. (Tras el reboot) Levantar ROS y lanzar el nodo con el script unificado

```bash
cd ~/carpeta_compartida/ros_ws/src/servo_dispenser
chmod +x run_servo.sh
./run_servo.sh
```

---

## API ROS

### Suscribe

* **`/tirgo/dispense/request`** (`std_msgs/Int32`)
  Recibe el **ID del bote** a dispensar.

  **Contrato del topic:**

  * IDs válidos: **1, 2, 3, 4**
  * Cualquier otro valor se considera **inválido**
  * Para IDs inválidos:

    * no se mueven servos
    * no se publica `/tirgo/dispense/ready`

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

### Hardware

* Raspberry Pi 3B (dispensador físico conectado a sus GPIO).

### Software

* Ubuntu 20.04
* ROS **Noetic**
* Python **3.8**

### Entorno TirGoPharma

El paquete asume el entorno del proyecto en `~/carpeta_compartida`:

```bash
source /opt/ros/noetic/setup.bash
source ~/carpeta_compartida/gallium/setup.bash
source ~/carpeta_compartida/setup_env.sh
```

### GPIO / pigpio

* Usuario perteneciente al grupo `gpio`
* Daemon `pigpiod` activo

---

## GPIO usados (demo)

En la configuración de referencia de la demo, el nodo utiliza:

* **Servo A** → GPIO **2**
* **Servo B** → GPIO **3**

Estos pines están definidos directamente en el código del nodo.
Si tu montaje físico utiliza otros GPIO, será necesario ajustar
la configuración en `scripts/servo_dispenser_node.py`.

---

## Instalación rápida en la Raspberry Pi

El paquete incluye un script que deja la Pi preparada (grupos, pigpio, servicios…).

### 1. Ejecutar script de setup

```bash
cd ~/carpeta_compartida/ros_ws/src/servo_dispenser/scripts

sudo ./setup_servo_rpi.sh
sudo reboot
```

El reinicio es necesario para que se apliquen los cambios de pertenencia
a grupos (`gpio`, `dialout`, `video`, etc.).

---

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

El script `run_servo.sh`, ubicado en la raíz del paquete, simplifica
el arranque del servicio en la Raspberry Pi.

Este script:

1. Carga el entorno base de ROS y TirGo (si existen los archivos).
2. Entra en `~/carpeta_compartida/ros_ws` y ejecuta `catkin_make`.
3. Carga `devel/setup.bash`.
4. Lanza el nodo mediante:

   ```
   roslaunch servo_dispenser servo_dispenser_rpi.launch
   ```

Ruta:

```
~/carpeta_compartida/ros_ws/src/servo_dispenser/run_servo.sh
```

### Uso básico

```bash
cd ~/carpeta_compartida/ros_ws/src/servo_dispenser
chmod +x run_servo.sh
./run_servo.sh
```

El script se detiene automáticamente si algún comando falla (`set -e`).

---

## Ejecución (alternativas)

Asegúrate de que `ROS_MASTER_URI` y `ROS_HOSTNAME`
están correctamente configurados en la Raspberry Pi.

```bash
cd ~/carpeta_compartida/ros_ws

source /opt/ros/noetic/setup.bash
source ~/carpeta_compartida/gallium/setup.bash
source ~/carpeta_compartida/setup_env.sh
source devel/setup.bash

roslaunch servo_dispenser servo_dispenser_rpi.launch
# alternativamente:
# rosrun servo_dispenser servo_dispenser_node.py
```

---

## Comprobación rápida (smoke test)

### 1. Enviar una petición válida

```bash
rostopic pub /tirgo/dispense/request std_msgs/Int32 "data: 1"
```

### 2. Ver la señal de `ready`

```bash
rostopic echo /tirgo/dispense/ready
```

Salida esperada tras el movimiento del servo:

```text
data: True
```

Para IDs inválidos, no debe aparecer ningún mensaje `ready`.

---

## Notas importantes

* Este nodo está diseñado para ejecutarse **exclusivamente en la Raspberry Pi**.
* **No es compatible con Docker ni con ejecución en un PC sin GPIO**.
* En un host sin hardware real, la lógica puede validarse mediante los **tests unitarios**.
* La comunicación con el resto del sistema TirGoPharma se realiza únicamente mediante **tópicos ROS**.

---

## Tests automatizados

El paquete incluye dos niveles de testing:

### 1. Tests unitarios (`pytest`)

* No requieren ROS ni Raspberry Pi.
* Usan clases fake (`FakePi`, `DummyPublisher`).

Qué validan:

* Conversión ángulo → microsegundos.
* Selección correcta del servo según `bin_id`.
* Publicación de `ready=True` en ciclos válidos.
* IDs inválidos → sin movimiento y sin `ready`.

Ejecución:

```bash
cd ~/carpeta_compartida/ros_ws/src/servo_dispenser
pytest -q
```

---

### 2. Tests de flujo completo (`rostest`)

* Se ejecutan en la Raspberry Pi.
* Usan el nodo real y `pigpio`.

Qué validan:

* Movimiento del servo correcto para cada ID.
* Publicación de **un único** `ready=True`.
* Manejo correcto de IDs inválidos.
* Estabilidad del nodo durante la prueba.

Ejecución:

```bash
cd ~/carpeta_compartida/ros_ws
rostest servo_dispenser servo_dispenser_flow.test
```

Salida esperada:

```text
SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0
```
