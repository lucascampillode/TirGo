"""
[LEGACY] Servidor web original de TirGo (v1).

Este script montaba una interfaz mínima con detección de hotword
y visualización básica del texto de STT.

En la versión actual:
- El servidor principal es `tirgo_web_server` (ver `scripts/tirgo_web_server`)
- La app Flask real está en `tirgo_ui/app.py`
- La lógica de conversación y misión vive en `rosio.py`

Este archivo se mantiene solo como referencia histórica y NO se usa
en el despliegue normal (se prefiere `roslaunch tirgo_ui web.launch`).
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, sys, atexit, signal, subprocess, threading, time, re
from collections import deque
from datetime import datetime, timezone, timedelta

from flask import Flask, render_template, jsonify
import rospy
from std_msgs.msg import String
import rospkg

# ================== Config ==================
TZ_OFFSET = +2  # Europe/Madrid en verano ≈ +2; si quieres precisión total usa pytz/zoneinfo
HOTWORD = "hola"
WINDOW_SECS = 10.0
INTENT_TOPIC = "/tirgo/ui/intent"

# Topic de STT parametrizable (default a tu caso actual)
STT_TOPIC_DEFAULT = "/stt/text"

# (Opcional) comando para lanzar STT como subproceso (comenta si no quieres auto-lanzar)
LAUNCH_STT_CMD = ["roslaunch", "stt_vosk", "stt_vosk.launch"]  # o usa rosrun con args

# ================== Estado ==================
state = {"hotword_ts": 0.0, "last_text": "", "listening": True}
history = deque(maxlen=50)
_pub = None
_rosproc = None
_lock = threading.Lock()

# ================== Utilidades ==================
def now_local():
    # mini zona horaria sin dependencias pesadas
    return datetime.now(timezone(timedelta(hours=TZ_OFFSET)))

def normalize(text: str) -> str:
    t = text.lower().strip()
    for a, b in (("á","a"),("é","e"),("í","i"),("ó","o"),("ú","u"),
                 ("à","a"),("è","e"),("ì","i"),("ò","o"),("ù","u"),
                 ("ü","u"),("ï","i")):
        t = t.replace(a, b)
    return t

def hotword_active() -> bool:
    return (state["hotword_ts"] > 0) and ((time.time() - state["hotword_ts"]) <= WINDOW_SECS)

# ================== ROS callbacks ==================
def stt_cb(msg: String):
    txt = normalize(msg.data)
    with _lock:
        state["last_text"] = txt
        history.appendleft({"t": now_local().isoformat(timespec="seconds"), "text": txt})
        if re.search(r"\bhola\b", txt):
            state["hotword_ts"] = time.time()
            rospy.loginfo("HOTWORD detectada: %s", msg.data)

def ros_spin():
    rate = rospy.Rate(40)
    while not rospy.is_shutdown():
        rate.sleep()

# ================== Lanzar / parar STT como subproceso ==================
def start_stt_subprocess():
    """Arranca tu nodo/launch de STT como proceso hijo (opcional)."""
    global _rosproc
    if not LAUNCH_STT_CMD:
        return
    if _rosproc is not None:
        return
    # aviso amistoso si no hay entorno ROS
    if os.environ.get("ROS_MASTER_URI") is None:
        print("⚠️  ROS_MASTER_URI no está definido. ¿Arrancaste roscore?")

    # heredamos stdout/stderr para ver logs en la misma terminal
    _rosproc = subprocess.Popen(LAUNCH_STT_CMD, preexec_fn=os.setsid)

def stop_stt_subprocess():
    """Mata el launch/nodo hijo al cerrar la web."""
    global _rosproc
    if _rosproc is not None:
        try:
            os.killpg(os.getpgid(_rosproc.pid), signal.SIGTERM)
        except Exception:
            pass
        _rosproc = None

atexit.register(stop_stt_subprocess)

# ================== Flask ==================
# rutas absolutas al paquete para templates/estáticos
APP = Flask(__name__, static_folder='static', template_folder='templates')

@app.after_request
def no_cache(resp):
    resp.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
    resp.headers['Pragma'] = 'no-cache'
    return resp

@app.route("/")
def index():
    return render_template("index.html")

@app.get("/state")
def get_state():
    with _lock:
        hot = hotword_active()
        remaining = max(0.0, WINDOW_SECS - (time.time() - state["hotword_ts"])) if hot else 0.0
        data = {
            "listening": state["listening"],
            "hotword": hot,
            "remaining": round(remaining, 2),
            "last_text": state["last_text"],
            "window_secs": WINDOW_SECS
        }
    return jsonify(data)

@app.route("/press/<action>", methods=["POST"])
def press(action):
    if action not in ("consultar", "diagnosticar", "leer"):
        return jsonify({"ok": False, "error": "Acción no válida"}), 400
    # 🔒 gateo por hotword
    if not hotword_active():
        return jsonify({"ok": False, "error": "Hotword no activo. Di 'hola' primero."}), 403
    _pub.publish(String(data=action))
    rospy.loginfo("Intent enviado: %s", action)
    with _lock:
        state["hotword_ts"] = 0.0
    return jsonify({"ok": True, "pressed": action})

def start_ros_in_background():
    threading.Thread(target=ros_spin, daemon=True).start()

def main():
    global _pub
    rospy.init_node("tirgo_ui_web", anonymous=True, disable_signals=True)

    # lee el topic por parámetro (puedes cambiarlo en el launch)
    stt_topic = rospy.get_param("~stt_topic", STT_TOPIC_DEFAULT)
    rospy.Subscriber(stt_topic, String, stt_cb)
    _pub = rospy.Publisher(INTENT_TOPIC, String, queue_size=10)

    # opcional: arrancar STT como subproceso (si usas LAUNCH_STT_CMD)
    # start_stt_subprocess()

    start_ros_in_background()
    app.run(host="0.0.0.0", port=5000, debug=False)

if __name__ == "__main__":
    main()
