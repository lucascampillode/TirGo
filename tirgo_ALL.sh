#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# TirGoPharma · Arranque completo (DB + contenedor ROS + nodos)
# Uso:
#   ./tirgo_ALL.sh
# ============================================================

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MODE="${1:-}"   # dev | deploy | (de momento solo usamos dev)

# -----------------------------
# 1) Cargar configuración (.env)
# -----------------------------
if [[ -f "${ROOT_DIR}/.env" ]]; then
  echo "[TirGo] Cargando configuración desde .env..."
  # shellcheck disable=SC2046
  export $(grep -v '^#' "${ROOT_DIR}/.env" | xargs)
else
  echo "⚠️  No se ha encontrado .env en ${ROOT_DIR}"
  echo "    Crea uno a partir de .env.example antes de seguir."
  exit 1
fi

if [[ -z "${MODE}" ]]; then
  MODE="${TIRGO_MODE:-dev}"
fi
echo "[TirGo] Modo seleccionado: ${MODE}"

TIRGO_DB_STACK_DIR="${TIRGO_DB_STACK_DIR:-infra/tirgo_db_stack}"

# -----------------------------
# Helper: docker compose vs docker-compose
# -----------------------------
if docker compose version &>/dev/null; then
  DC="docker compose"
elif docker-compose version &>/dev/null; then
  DC="docker-compose"
else
  echo "❌ No se ha encontrado ni 'docker compose' ni 'docker-compose'."
  exit 1
fi

# =============================
# 2) Levantar / reutilizar Mongo
# =============================
echo "[TirGo] Comprobando contenedor tirgo_mongo..."

if docker ps -a --format '{{.Names}}' | grep -q '^tirgo_mongo$'; then
  echo "[TirGo] Ya existe un contenedor tirgo_mongo → lo reutilizo."
  docker start tirgo_mongo >/dev/null 2>&1 || true
  docker start tirgo_mongo_express >/dev/null 2>&1 || true
  echo "✅ MongoDB arrancado reutilizando tus datos actuales."
else
  echo "[TirGo] No existe tirgo_mongo → lo creo con docker compose..."
  pushd "${ROOT_DIR}/${TIRGO_DB_STACK_DIR}" >/dev/null
  $DC up -d
  popd >/dev/null
  echo "✅ MongoDB creado y levantado vía docker compose."
fi

echo
echo "🎉 Base de datos lista."

# =============================
# 3) Levantar contenedor ROS
# =============================
ROS_SERVICE_NAME="${TIRGO_ROS_SERVICE_NAME:-ros1_rob_tirgo}"

echo
echo "[TirGo] Levantando servicio ROS (${ROS_SERVICE_NAME})..."

pushd "${ROOT_DIR}" >/dev/null
$DC up -d "${ROS_SERVICE_NAME}"
ROS_CONTAINER_ID="$($DC ps -q "${ROS_SERVICE_NAME}")"
popd >/dev/null

if [[ -z "${ROS_CONTAINER_ID}" ]]; then
  echo "❌ No he podido obtener el contenedor de ${ROS_SERVICE_NAME}."
  echo "   Revisa 'docker ps' y el docker-compose.yml."
  exit 1
fi

echo "✅ Contenedor ROS en marcha: ${ROS_CONTAINER_ID}"

# =============================
# 4) Ejecutar build + todos los nodos dentro del contenedor
# =============================
echo
echo "[TirGo] Ejecutando build + lanzamiento de nodos dentro del contenedor..."
echo "       (Ctrl+C para parar todos los nodos TirGo)"

# OJO: solo -i, sin -t, porque usamos heredoc
docker exec -i "${ROS_CONTAINER_ID}" bash <<'IN_CONTAINER'
set -e

WS=~/carpeta_compartida/ros_ws

echo "=== [TirGo] Build rápido antes de arrancar todo ==="

# 1) Entorno base para compilar
source /opt/ros/noetic/setup.bash

if [ -f ~/carpeta_compartida/gallium/setup.bash ]; then
  source ~/carpeta_compartida/gallium/setup.bash
fi

if [ -f ~/carpeta_compartida/setup_env.sh ]; then
  source ~/carpeta_compartida/setup_env.sh
fi

cd "$WS"

echo "[INFO] Ejecutando catkin_make en $WS..."
catkin_make
source devel/setup.bash

echo "[OK] Workspace compilado. Lanzando nodos..."

############################################
# 2) Lanzar nodos TirGo (voz + UI + misión)
############################################

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

cd "$WS"

echo "[INFO] Ejecutando catkin_make en $WS..."
catkin_make
source devel/setup.bash

# Mission Server 2 (Tiago speech node) - con log aparte
rosrun tirgo_mission_server tiago_speech_node.py > /tmp/tiago_speech.log 2>&1 &
PID_MS2=$!
echo "[LAUNCH] Mission Server 2 (PID: $PID_MS2, log: /tmp/tiago_speech.log)"


cd "$WS"

echo "[INFO] Ejecutando catkin_make en $WS..."
catkin_make
source devel/setup.bash
# TirGo UI
roslaunch tirgo_ui web.launch &
PID_UI=$!
echo "[LAUNCH] TirGo UI (PID: $PID_UI)"



############################################
# 3) Lanzar navegación: RViz + mapa + move
############################################

# Rviz (visualización)
roslaunch move rviz.launch &
PID_RVIZ=$!
echo "[LAUNCH] rviz (PID: $PID_RVIZ)"

# Esperar a que se abra RViz
sleep 4

# Cargar entorno de ROS base (por si acaso)
source /opt/ros/noetic/setup.bash

# Lanzar el map_server con tu mapa
rosrun map_server map_server /home/TirGo/carpeta_compartida/ros_ws/src/move/maps/Mapa_aula_mod_1.0.yaml &
PID_MAP=$!
echo "[LAUNCH] map_server (PID: $PID_MAP)"

# Esperar a que el mapa esté bien lanzado
sleep 1

# Volver a cargar tu workspace
source /home/TirGo/carpeta_compartida/ros_ws/devel/setup.bash

# Localizarse en posición inicial (bloqueante, sin &)
echo "[INFO] Publicando initial pose..."
rosrun move publish_initial_pose.py

# Esperar un poco por seguridad
sleep 10

# Lanzar comunicación del move (en background)
rosrun move comunication_move.py &
PID_MOVE=$!
echo "[LAUNCH] comunication_move.py (PID: $PID_MOVE)"

echo
echo "=== [TirGo] Todo lanzado."
echo "Pulsa Ctrl+C para parar todos los nodos."
echo

############################################
# 4) Gestión de parada limpia
############################################
cleanup() {
    echo
    echo "=== [TirGo] Parando todos los nodos... ==="
    kill $PID_STT  $PID_MS1  $PID_MS2  $PID_UI \
         $PID_RVIZ $PID_MAP  $PID_MOVE 2>/dev/null || true
    sleep 2
    echo "[OK] Nodos parados. Bye 👋"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Mantener el script vivo mientras los nodos están corriendo
wait
IN_CONTAINER
