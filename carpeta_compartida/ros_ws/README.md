# Dispensador (ROS1 Noetic + GPIO/Pigpio)

Escucha `std_msgs/Int32` en `/tirgo/dispense/request` y acciona el mecanismo del dispensador.

## Requisitos
- Raspberry Pi con Ubuntu 20.04 + ROS **Noetic**
- `roscore` en ejecución (local o remoto)
- Python 3.8+
- `pigpio` activo (`sudo systemctl start pigpiod`)

## Instalación rápida
En la Raspberry Pi:
```bash
cd dispensador
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip pigpio rospy
Si usas workspace:

Bash

cd ~/carpeta_compartida/ros_ws
catkin_make
source devel/setup.bash
Ejecución
En una terminal (si el roscore está en la Raspberry):

Bash

roscore
En otra terminal:

Bash

source /opt/ros/noetic/setup.bash
cd dispensador
source .venv/bin/activate
rosrun dispensador dispensador_node.py
Verificar
Enviar una petición de dispensación:

Bash

rostopic pub /tirgo/dispense/request std_msgs/Int32 "data: 1"
Verificar que el nodo recibe las peticiones:

Bash

rostopic echo /tirgo/dispense/request
