#!/usr/bin/env bash
set -e

echo "==========================================="
echo "  Script de setup para servo_dispenser 🩺"
echo "  (pigpio + grupos + daemon pigpiod)"
echo "==========================================="
echo

# 1) Comprobar que se ejecuta como root (sudo)
if [ "$EUID" -ne 0 ]; then
  echo "⚠️  Este script debe ejecutarse con sudo:"
  echo "    sudo $0"
  exit 1
fi

# 2) Detectar el usuario real (no 'root')
TARGET_USER="${SUDO_USER:-$USER}"

if [ "$TARGET_USER" = "root" ]; then
  echo "⚠️  No tiene sentido configurar todo para el usuario root."
  echo "    Ejecuta este script como tu usuario normal, por ejemplo:"
  echo "    sudo $0"
  exit 1
fi

echo "✅ Usuario objetivo: $TARGET_USER"
echo

# 3) Instalar pigpio y bindings de Python
echo "📦 Instalando pigpio y python3-pigpio..."
apt update
apt install -y pigpio python3-pigpio

echo
echo "✅ pigpio instalado."
echo

# 4) Activar y arrancar pigpiod
echo "⚙️  Habilitando y arrancando el servicio pigpiod..."
systemctl enable pigpiod
systemctl start pigpiod

echo
systemctl is-active --quiet pigpiod && \
  echo "✅ pigpiod está en marcha." || \
  echo "⚠️  pigpiod NO parece estar activo, revisa con: systemctl status pigpiod"

echo

# 5) Añadir al usuario a los grupos necesarios
echo "👤 Añadiendo al usuario '$TARGET_USER' a los grupos gpio, dialout y video..."
usermod -aG gpio "$TARGET_USER" || true
usermod -aG dialout "$TARGET_USER" || true
usermod -aG video "$TARGET_USER" || true

echo "✅ Grupos actualizados (necesitarás cerrar sesión o reiniciar)."
echo

# 6) Comprobar que el módulo de Python se importa bien
echo "🧪 Probando import de pigpio en Python..."
sudo -u "$TARGET_USER" python3 - << 'EOF'
try:
    import pigpio
    print("✅ Import pigpio OK.")
except Exception as e:
    print("❌ Error importando pigpio:", e)
EOF

echo
echo "==========================================="
echo "  Setup de la Raspberry para servo_dispenser"
echo "  completado."
echo
echo "➡️  RECUERDA:"
echo "  - Reinicia la Raspberry para que los cambios de grupos se apliquen:"
echo "      sudo reboot"
echo
echo "  - Luego, en la Raspberry:"
echo "      source /opt/ros/noetic/setup.bash"
echo "      source ~/carpeta_compartida/ros_ws/devel/setup.bash"
echo "      roslaunch servo_dispenser servo_dispenser_rpi.launch"
echo "==========================================="
