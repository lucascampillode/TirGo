#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MODE="${1:-}"   # dev | deploy | vacío = usa TIRGO_MODE o dev por defecto

# -----------------------------
# Cargar configuración (.env)
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

# Resolver modo final (ahora mismo solo para info)
if [[ -z "${MODE}" ]]; then
  MODE="${TIRGO_MODE:-dev}"
fi
echo "[TirGo] Modo seleccionado: ${MODE}"

TIRGO_DB_STACK_DIR="${TIRGO_DB_STACK_DIR:-infra/tirgo_db_stack}"

# -----------------------------
# Levantar / reutilizar MongoDB
# -----------------------------
echo "[TirGo] Comprobando contenedor tirgo_mongo..."

if docker ps -a --format '{{.Names}}' | grep -q '^tirgo_mongo$'; then
  echo "[TirGo] Ya existe un contenedor tirgo_mongo → lo reutilizo, no lo recreo."
  docker start tirgo_mongo >/dev/null 2>&1 || true
  docker start tirgo_mongo_express >/dev/null 2>&1 || true
  echo "✅ MongoDB arrancado reutilizando tus datos actuales."
else
  echo "[TirGo] No existe tirgo_mongo → lo creo con docker compose..."
  pushd "${ROOT_DIR}/${TIRGO_DB_STACK_DIR}" >/dev/null
  docker compose up -d
  popd >/dev/null
  echo "✅ MongoDB creado y levantado vía docker compose."
fi

echo
echo "🎉 Base de datos lista."
