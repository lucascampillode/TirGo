# src/tirgo_ui/config.py
import os

# Rutas reales a templates/static (ajustadas a tu estructura)
BASE_DIR     = os.path.dirname(__file__)                  # …/src/tirgo_ui/src/tirgo_ui
PKG_SRC_DIR  = os.path.dirname(BASE_DIR)                  # …/src/tirgo_ui/src
PROJECT_ROOT = os.path.dirname(PKG_SRC_DIR)               # …/src/tirgo_ui

TEMPLATE_DIR = os.path.join(PROJECT_ROOT, "templates")
STATIC_DIR   = os.path.join(PROJECT_ROOT, "static")

# Web
PORT  = int(os.environ.get("PORT", "9001"))
DEBUG = os.environ.get("FLASK_DEBUG", "1") == "1"

# Hotword / STT
TIRGO_HOTWORD     = os.environ.get("TIRGO_HOTWORD", "hola")
STT_TOPIC_DEFAULT = os.environ.get("TIRGO_STT_TOPIC", "/stt/text")

# MongoDB
MONGO_URI = os.environ.get("MONGO_URI", "mongodb://127.0.0.1:27017/tirgo")

# Seguridad (lo afinamos en el siguiente paso)
TIRGO_PEPPER = os.environ.get("TIRGO_PEPPER", "cambia-esto")
PEPPER = TIRGO_PEPPER   # alias legacy para compatibilidad
