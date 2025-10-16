# Web 3 botones (ROS1 Noetic + Flask)

Publica `std_msgs/String` en `/tirgo/ui/intent` con valores: `consultar`, `diagnosticar`, `leer`.

## Requisitos
- Ubuntu 20.04 + ROS **Noetic**
- `roscore` en ejecución
- Python 3.8+

## Instalación rápida
```bash
cd web-ros1-noetic-buttons
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip flask
```

## Ejecución
En una terminal:
```bash
roscore
```
En otra terminal (misma máquina o con variables ROS bien configuradas):
```bash
source /opt/ros/noetic/setup.bash
cd web-ros1-noetic-buttons
source .venv/bin/activate   # si usaste venv
python3 app.py
```

Abre el navegador en: `http://IP_DEL_SERVIDOR:5000`

## Verificar
```bash
rostopic echo /tirgo/ui/intent
```
Pulsa los botones y verás:
```
data: "consultar"
data: "diagnosticar"
data: "leer"
```
