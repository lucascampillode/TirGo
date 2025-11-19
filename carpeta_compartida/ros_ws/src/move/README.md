# Navegacion y uso de run_all.sh

Sistema mínimo para levantar navegación básica del robot TIAGo con: teleoperación, carga de mapa, RViz configurado y un cliente que publica posiciones objetivo.

## Requisitos
  - Ubuntu 20.04 + ROS Noetic
  - roscore en ejecución
  - Docker (opcional, recomendado)
  - Python 3.8+

## Paquetes necesarios:
  - ros-noetic-teleop-twist-keyboard (No es necesario en la ejecucion pero si para la obtencion de puntos)
  - ros-noetic-map-server
  - ros-noetic-move-base-msgs
  
  Este módulo implementa el flujo completo de navegación del robot TIAGo dentro del proyecto TirgoPharma, integrando:

## Ejecuciónes
 ### run_all.sh
 ```bash
    cd carpeta_compartida/ros_ws/src/move/scripts/
    ./run_all.sh
 ```
### checkpointfollower
```bash
    cd carpeta_compartida/ros_ws/src/move/scripts/
    ./run_all.sh
 ```
### rviz.launch (Carga rviz)
```bash
    source /home/TirGo/carpeta_compartida/ros_ws/devel/setup.bash
    roslaunch move rviz.launch 
 ```
### map_server (carga mapa)
```bash
    source /opt/ros/noetic/setup.bash
    rosrun map_server map_server /home/TirGo/carpeta_compartida/ros_ws/src/move/maps/Mapa_aula_mod_1.0.yaml 
 ```

 ## Verificar
 Al lanzar run_all se vera el robot en el mapay se moverá hasta el punto  
