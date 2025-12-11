#!/usr/bin/env bash

# === Entorno común TirGo (ROS1 Noetic) ===

# 1) ROS Noetic
source /opt/ros/noetic/setup.bash

# 2) Gallium (tu overlay)
source ~/carpeta_compartida/gallium/setup.bash

# 3) Variables propias (TIRGO_* y demás)
source ~/carpeta_compartida/setup_env.sh

# 4) Workspace ROS
WS=~/carpeta_compartida/ros_ws
source "$WS/devel/setup.bash"

echo "[OK] Entorno TirGo (ROS1) cargado."
