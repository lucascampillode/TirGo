#!/bin/bash
set -e  # Detener el script si cualquier comando falla

# Rutas
WS_PATH=~/carpeta_compartida/ros_ws

echo "=========================================="
echo "🚀 Iniciando secuencia de arranque Servo"
echo "=========================================="

# 1. Cargar entorno base y herramientas de TirGo
echo "source /opt/ros/noetic/setup.bash"
source /opt/ros/noetic/setup.bash

if [ -f ~/carpeta_compartida/gallium/setup.bash ]; then
    echo "source ~/carpeta_compartida/gallium/setup.bash"
    source ~/carpeta_compartida/gallium/setup.bash
fi

if [ -f ~/carpeta_compartida/setup_env.sh ]; then
    echo "source ~/carpeta_compartida/setup_env.sh"
    source ~/carpeta_compartida/setup_env.sh
fi

# 2. Ir al workspace y compilar
echo "📂 Entrando en $WS_PATH..."
cd "$WS_PATH"

echo "🔨 Ejecutando catkin_make..."
catkin_make

# 3. Cargar el entorno del workspace recién compilado
echo "source devel/setup.bash"
source devel/setup.bash

# 4. Lanzar el nodo
echo "=========================================="
echo "✅ Compilación correcta. Lanzando nodo..."
echo "=========================================="
roslaunch servo_dispenser servo_dispenser_rpi.launch
