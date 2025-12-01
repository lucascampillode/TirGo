#!/usr/bin/env bash

set -e

# === CONFIGURACIÓN BÁSICA ===
WS=~/carpeta_compartida/ros_ws
PKG=tirgo_mission_server
TEST_LAUNCH=test_mission_flow.test

cd "$WS"
source devel/setup.bash

echo ""
echo "========================================"
echo "  Lanzando tests de TirgoMissionServer"
echo "========================================"
echo ""

rostest "$PKG" "$TEST_LAUNCH"

echo ""
echo "========================================"
echo "  Extrayendo resumen de los logs..."
echo "========================================"
echo ""

# Último directorio de logs
LAST_RUN_DIR=$(ls -dt ~/.ros/log/*/ | head -n1)

if [ -z "$LAST_RUN_DIR" ]; then
  echo "❌ No se ha encontrado ningún directorio de logs en ~/.ros/log"
  exit 1
fi

LOGFILE=$(ls -1t "$LAST_RUN_DIR"/test_mission_flow-*.log | head -n1)

if [ -z "$LOGFILE" ]; then
  echo "❌ No se ha encontrado test_mission_flow-*.log en $LAST_RUN_DIR"
  exit 1
fi

echo "Usando log: $LOGFILE"
echo ""

grep -A20 "RESUMEN DE TESTS" "$LOGFILE" || {
  echo "❌ No se ha encontrado el bloque de resumen en el log."
  exit 1
}

echo ""
echo "========================================"
echo "        FIN DE LOS TESTS ✅"
echo "========================================"
echo ""
