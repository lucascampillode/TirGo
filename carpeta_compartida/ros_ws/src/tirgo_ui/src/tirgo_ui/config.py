import os

# ============================================================
# Rutas de templates y estáticos
# ============================================================
# Este archivo está en: ros_ws/src/tirgo_ui/src/tirgo_ui/config.py
BASE_DIR   = os.path.dirname(__file__)              # .../src/tirgo_ui/src/tirgo_ui
PKG_ROOT   = os.path.dirname(os.path.dirname(BASE_DIR))  # .../src/tirgo_ui
TEMPLATE_DIR = os.path.join(PKG_ROOT, "templates")
STATIC_DIR   = os.path.join(PKG_ROOT, "static")

# ============================================================
# Puertos / servidor
# ============================================================
PORT = int(os.environ.get("PORT", "9001"))

# ============================================================
# Modos de funcionamiento
# ============================================================
TIRGO_DEV = os.environ.get("TIRGO_DEV", "1") == "1"

_debug_env = os.environ.get("FLASK_DEBUG", None)
if _debug_env is None:
    DEBUG = TIRGO_DEV
else:
    DEBUG = (_debug_env == "1")

# ============================================================
# MongoDB
# ============================================================
MONGO_URI = os.environ.get(
    "MONGO_URI",
    "mongodb://tirgo_app:tirgo@127.0.0.1:27017/tirgo?authSource=tirgo"
)

# ============================================================
# Voz / STT
# ============================================================
HOTWORD   = os.environ.get("TIRGO_HOTWORD", "hola tirgo")
STT_TOPIC = os.environ.get("TIRGO_STT_TOPIC", "/stt/text")

# ---- Compatibilidad hacia atrás con rosio.py ----
TIRGO_HOTWORD     = HOTWORD
STT_TOPIC_DEFAULT = STT_TOPIC
