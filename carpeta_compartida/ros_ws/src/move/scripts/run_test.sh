#!/bin/bash

# Cargar mi ws
source /home/TirGo/carpeta_compartida/ros_ws/devel/setup.bash

# Lanzar rviz
roslaunch move rviz.launch &
RVIZ_PID=$!
echo "[INFO] Lanzado rviz con PID $RVIZ_PID"

# Esperar un momento para que se abra rviz y le de tiempo antes de cargar el mapa
sleep 4

# Cargar entorno de ROS
source /opt/ros/noetic/setup.bash

# Lanzar el map_server
rosrun map_server map_server /home/TirGo/carpeta_compartida/ros_ws/src/move/maps/Mapa_aula_mod_1.0.yaml &
MAP_PID=$!
echo "[INFO] Lanzado map_server con PID $MAP_PID"

# Esperar un momento para asegurarse de que el mapa esta bien lanzado
sleep 1

# Cargar mi ws
source /home/TirGo/carpeta_compartida/ros_ws/devel/setup.bash

# Localizarse en posicion inicial
rosrun move publish_initial_pose.py

# Esperar un momento para asegurarse de que el mapa esta bien lanzado
sleep 10
#
## Lanzar los demas nodos
#rosrun move test_comunicaciones.py &
#CHECKPOINT_PID=$!
#echo "[INFO] Lanzado test_comunicaciones con PID $CHECKPOINT_PID"
#
## Esperar a que terminen todos los procesos lanzados
wait $CHECKPOINT_PID
echo "Terminado"

