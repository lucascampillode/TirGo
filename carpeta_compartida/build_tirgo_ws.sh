#!/usr/bin/env bash
set -e

WS=~/carpeta_compartida/ros_ws

echo "=== [TirGo] Compilando workspace ROS ==="

# Entorno base (sin devel, aún no existe la primera vez)
source /opt/ros/noetic/setup.bash
source ~/carpeta_compartida/gallium/setup.bash
source ~/carpeta_compartida/setup_env.sh

cd "$WS"

echo "[INFO] Ejecutando catkin_make en $WS..."
catkin_make

echo "[OK] catkin_make completado."
echo "Ahora puedes hacer: source ~/carpeta_compartida/setup_tirgo_env.sh en cada terminal."
