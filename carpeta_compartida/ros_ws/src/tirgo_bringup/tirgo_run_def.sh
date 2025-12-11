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

# Lanzar los demas nodos
rosrun move comunication_move.py &






#!/usr/bin/env bash
set -e

WS=~/carpeta_compartida/ros_ws

echo "=== [TirGo] Build rápido antes de arrancar todo ==="

# 1) Entorno base para compilar
source /opt/ros/noetic/setup.bash
source ~/carpeta_compartida/gallium/setup.bash
source ~/carpeta_compartida/setup_env.sh

cd "$WS"

echo "[INFO] Ejecutando catkin_make en $WS..."
catkin_make
source devel/setup.bash

echo "[OK] Workspace compilado. Lanzando nodos..."

# 2) Lanzar cada cosa en background y guardar PIDs

# STT Vosk
roslaunch stt_vosk stt_vosk.launch &
PID_STT=$!
echo "[LAUNCH] STT Vosk (PID: $PID_STT)"

# Mission Server 1
rosrun tirgo_mission_server tirgo_mission_server.py &
PID_MS1=$!
echo "[LAUNCH] Mission Server 1 (PID: $PID_MS1)"

# Pequeña espera para que arranquen bien los primeros nodos
sleep 3

# Mission Server 2 (Tiago speech node) - con log aparte
rosrun tirgo_mission_server tiago_speech_node.py > /tmp/tiago_speech.log 2>&1 &
PID_MS2=$!
echo "[LAUNCH] Mission Server 2 (PID: $PID_MS2, log: /tmp/tiago_speech.log)"

# TirGo UI
roslaunch tirgo_ui web.launch &
PID_UI=$!
echo "[LAUNCH] TirGo UI (PID: $PID_UI)"

echo
echo "=== [TirGo] Todo lanzado."
echo "Pulsa Ctrl+C para parar todos los nodos."
echo

# 3) Si haces Ctrl+C → matar todos los procesos
cleanup() {
    echo
    echo "=== [TirGo] Parando todos los nodos... ==="
    kill $PID_STT $PID_MS1 $PID_MS2 $PID_UI 2>/dev/null || true
    sleep 2
    echo "[OK] Nodos parados. Bye 👋"
    exit 0
}

trap cleanup SIGINT SIGTERM

# 4) Esperar a que terminen (para que el script no salga)
wait
