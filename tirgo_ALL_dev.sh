#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="dev"

echo "[TirGo] Modo DEV"

# 1) .env (en dev sigue siendo obligatorio en tu diseño)
if [[ -f "${ROOT_DIR}/.env" ]]; then
  export $(grep -v '^#' "${ROOT_DIR}/.env" | xargs)
else
  echo "❌ Falta .env"
  exit 1
fi

TIRGO_DB_STACK_DIR="${TIRGO_DB_STACK_DIR:-infra/tirgo_db_stack}"
ROS_SERVICE_NAME="${TIRGO_ROS_SERVICE_NAME:-ros1_rob_tirgo}"

# docker compose helper
if docker compose version &>/dev/null; then
  DC="docker compose"
else
  DC="docker-compose"
fi

# 2) Mongo
if docker ps -a --format '{{.Names}}' | grep -q '^tirgo_mongo$'; then
  docker start tirgo_mongo tirgo_mongo_express >/dev/null 2>&1 || true
else
  pushd "${ROOT_DIR}/${TIRGO_DB_STACK_DIR}" >/dev/null
  $DC up -d
  popd >/dev/null
fi

# 3) Contenedor ROS
pushd "${ROOT_DIR}" >/dev/null
$DC up -d "${ROS_SERVICE_NAME}"
ROS_CONTAINER_ID="$($DC ps -q "${ROS_SERVICE_NAME}")"
popd >/dev/null

# 4) Ejecutar dentro del contenedor
docker exec -i "${ROS_CONTAINER_ID}" bash <<'IN_CONTAINER'
set -e

source /opt/ros/noetic/setup.bash
cd ~/carpeta_compartida/ros_ws

catkin_make
source devel/setup.bash

echo "🚀 DEV: tirgo_all_dev.launch"
roslaunch tirgo_bringup tirgo_all_dev.launch
IN_CONTAINER
