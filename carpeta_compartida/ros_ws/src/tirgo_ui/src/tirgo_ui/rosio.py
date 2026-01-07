# src/tirgo_ui/rosio.py
import os, socket, subprocess, threading
from urllib.parse import urlparse
from typing import Any, Dict
from .config import STT_TOPIC_DEFAULT, TIRGO_HOTWORD
from . import session

# --- ROS imports / dummies ---
try:
    import rospy
    from std_msgs.msg import String, Bool, Int32
    import actionlib
    from tirgo_msgs.msg import TirgoMissionAction, TirgoMissionGoal

    # (Antes se usaba TtsAction/TtsGoal/TtsText; ahora delegamos en /tirgo/say)
    ROS_AVAILABLE = True
except Exception:
    ROS_AVAILABLE = False

    class _DummyPub:
        def publish(self, *_a, **_k):
            pass

    class _DummyRospy:
        def init_node(self, *a, **k):
            pass

        def loginfo(self, *a, **k):
            print(*a)

        def signal_shutdown(self, *a, **k):
            pass

        def get_param(self, *a, **k):
            return STT_TOPIC_DEFAULT

        def Subscriber(self, *a, **k):
            return None

        Publisher = staticmethod(lambda *a, **_k: _DummyPub())

    rospy = _DummyRospy()  # type: ignore
    String = str           # type: ignore
    Bool = bool            # type: ignore
    Int32 = int            # type: ignore
    actionlib = None       # type: ignore

# publishers de estado
_pub_state = _pub_status = _pub_error = None

# Publisher de texto de voz de alto nivel (/tirgo/say)
_pub_say = None

# --- Cliente de misión (ActionServer /tirgo/mission) ---
_mission_client = None  # type: ignore

# --- Estado de la misión (para la web) ---
# running:    True mientras el Action está en curso
# success:    None (en curso / no lanzada), True, False
_mission_status: Dict[str, Any] = {
    "running": False,
    "state": "IDLE",
    "progress": 0.0,
    "success": None,
    "error_code": "",
    "error_message": "",
}


def _set_mission_status(**kwargs):
    """Actualiza el diccionario interno de estado de misión."""
    _mission_status.update(kwargs)


def get_mission_status() -> Dict[str, Any]:
    """
    Devuelve una copia del estado de misión para usar desde Flask.
    """
    return dict(_mission_status)


# --- Estado de conversación ---
# idle          -> no ha pasado nada todavía
# await_confirm -> ya hemos saludado al usuario, esperamos "sí"/"adelante"
# session       -> sesión web abierta
_conv_state = "idle"

# Variantes aceptadas (saludo)
HOTWORD_VARIANTS = [
    "hola tiago", "hola, tiago",
    "hola tirgo", "hola, tirgo",
    "hola thiago", "hola, thiago",
    (TIRGO_HOTWORD or "").lower() if TIRGO_HOTWORD else "hola tirgo",
]

# Palabras de confirmación
CONFIRM_WORDS = ["si", "sí", "adelante", "vale", "vamos", "empezar", "claro", "ok"]

# === VOICE NAV (intents de menú) ===
_last_voice_nav = None  # "consultar" | "leer" | "diagnostico" | None

VOICE_INTENTS = {
    "consultar": ["consultar", "ver recetas", "recetas", "mis recetas", "consulta"],
    "leer": ["leer", "pedir", "pedir medicamento", "pedido", "solicitar"],
    "diagnostico": ["diagnostico", "diagnóstico", "diagnosticar", "test", "diagnostico guiado", "diagnóstico guiado"],
}


def _match_intent(text_norm: str):
    for intent, kws in VOICE_INTENTS.items():
        for k in kws:
            if text_norm == k or k in text_norm:
                return intent
    return None


def get_and_clear_voice_nav() -> str:
    """Consumido por la web para saber si hay navegación por voz pendiente."""
    global _last_voice_nav
    v = _last_voice_nav
    _last_voice_nav = None
    return v or ""


# ============================================================
# CONTEXTO UI: en qué pantalla está la web (para "gating" de STT)
# ============================================================
# home | consultar | leer | diagnostico
_ui_menu = "home"

def set_ui_menu(menu: str):
    """
    La web llama a esto cuando cambia de pantalla.
    Sirve para ignorar intents globales en submenús.
    """
    global _ui_menu
    m = (menu or "").strip().lower()
    if m in ("home", "consultar", "leer", "diagnostico"):
        _ui_menu = m

def get_ui_menu() -> str:
    return _ui_menu


def master_up() -> bool:
    try:
        uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
        u = urlparse(uri)
        host, port = u.hostname or "localhost", u.port or 11311
        with socket.create_connection((host, port), timeout=0.3):
            return True
    except Exception:
        return False


def init():
    global _pub_state, _pub_status, _pub_error, _pub_say
    global _mission_client

    if ROS_AVAILABLE and master_up():
        try:
            rospy.init_node("tirgo_web_server_v2", anonymous=True, disable_signals=True)
        except Exception:
            # Si ya hay nodo inicializado, seguimos igual
            pass

        _pub_state = rospy.Publisher("tirgo/ui/state", String, queue_size=10)
        _pub_status = rospy.Publisher("tirgo/ui/status", String, queue_size=10)
        _pub_error = rospy.Publisher("tirgo/ui/error", String, queue_size=10)

        # Nuevo: publisher de alto nivel para texto a decir
        _pub_say = rospy.Publisher("/tirgo/say", String, queue_size=10)

        stt_topic = rospy.get_param("~stt_topic", STT_TOPIC_DEFAULT)
        rospy.Subscriber(stt_topic, String, _stt_cb)

        # Inicializa el cliente de misión (/tirgo/mission)
        try:
            _init_mission_client()
        except Exception:
            pass

        try:
            rospy.loginfo(f"[STT] suscrito a {stt_topic}")
        except Exception:
            pass
    else:
        class DummyPub:
            def publish(self, *_a, **_k):
                pass

        _pub_state = _pub_status = _pub_error = DummyPub()
        _pub_say = DummyPub()


def _init_mission_client():
    """Inicializa el cliente de la action /tirgo/mission."""
    global _mission_client
    if not ROS_AVAILABLE or actionlib is None:
        return
    try:
        _mission_client = actionlib.SimpleActionClient("/tirgo/mission", TirgoMissionAction)
        ok = _mission_client.wait_for_server(rospy.Duration(2.0))
        if ok:
            rospy.loginfo("[MISSION] /tirgo/mission listo ✅")
        else:
            rospy.loginfo("[MISSION] /tirgo/mission no responde (timeout)")
            _mission_client = None
    except Exception as e:
        rospy.loginfo(f"[MISSION] no disponible: {e}")
        _mission_client = None


def _say_with_tiago(text: str):
    """
    En vez de hablar directamente con /tts, publicamos en /tirgo/say
    y dejamos que tirgo_speech_node se encargue del TTS real.
    """
    global _pub_say

    text = (text or "").strip()
    if not text:
        return

    # Si no hay ROS o no hay publisher, logueamos solo
    if not ROS_AVAILABLE or _pub_say is None:
        try:
            rospy.loginfo(f"[ROSIO] (sin /tirgo/say) diría: {text}")
        except Exception:
            print("[ROSIO] (sin /tirgo/say) diría:", text)
        return

    try:
        # Aquí String es std_msgs.msg.String cuando ROS_AVAILABLE es True
        _pub_say.publish(String(data=text))
        rospy.loginfo(f"[ROSIO] Enviado a /tirgo/say: {text}")
    except Exception as e:
        try:
            rospy.loginfo(f"[ROSIO] Error publicando en /tirgo/say: {e}")
        except Exception:
            print("[ROSIO] Error publicando en /tirgo/say:", e)


def _stt_cb(msg: Any):
    """
    Callback de STT:
    - si estamos en idle y oímos el saludo -> TIAGo saluda y pasamos a await_confirm
    - si estamos en await_confirm y oímos un sí -> activamos sesión
    - si la sesión está activa -> detectar intents de navegación SOLO en home
      (en submenús se ignoran para que Tiago no se dispare)
    """
    global _conv_state
    try:
        txt = msg.data if hasattr(msg, "data") else str(msg)
    except Exception:
        txt = str(msg)

    text_norm = (txt or "").strip().lower()
    if not text_norm:
        return

    # 1) En idle: detectar saludo
    if _conv_state == "idle":
        if any(hw in text_norm for hw in HOTWORD_VARIANTS):
            _say_with_tiago(
                "Hola, soy Tiago, el robot de TirGo Pharma. "
                "Estoy aquí para ayudarte. ¿Quieres comenzar la consulta?"
            )
            _conv_state = "await_confirm"
            pub_state("tiago:greeted")
            try:
                rospy.loginfo(f"[HOTWORD] saludo detectado en: {text_norm}")
            except Exception:
                pass
            return

    # 2) Esperando confirmación
    elif _conv_state == "await_confirm":
        if any(text_norm.startswith(w) or f" {w}" in f" {text_norm}" for w in CONFIRM_WORDS):
            if not session.is_active():
                session.start_session(op_name="voice_unlock")
            _conv_state = "session"
            pub_state("session:active")
            try:
                rospy.loginfo(f"[CONV] confirmación recibida: {text_norm}")
            except Exception:
                pass
            return
        else:
            try:
                rospy.loginfo(f"[CONV] no es confirmación válida: {text_norm}")
            except Exception:
                pass
            return

    # 3) Con sesión activa
    else:
        try:
            rospy.loginfo(f"[STT] sesión activa, texto: {text_norm} (ui_menu={_ui_menu})")
        except Exception:
            pass

        # 🔒 Bloqueo de intents globales en submenús:
        # Si la UI no está en "home", ignoramos consultar/leer/diagnostico.
        if _ui_menu in ("consultar", "leer", "diagnostico"):
            return

        # Solo en HOME aceptamos intents de navegación
        intent = _match_intent(text_norm)
        if intent:
            global _last_voice_nav
            _last_voice_nav = intent
            pub_state(f"voice:navigate:{intent}")
            try:
                if intent == "consultar":
                    _say_with_tiago("Accediendo al apartado de consulta de recetas.")
                elif intent == "leer":
                    _say_with_tiago("Abriendo el módulo de petición de medicamentos.")
                elif intent == "diagnostico":
                    _say_with_tiago("Iniciando el asistente de diagnóstico.")
            except Exception:
                pass
            return


def conv_state() -> str:
    """Devuelve el estado de la pequeña conversación (para que la web lo pinte)."""
    return _conv_state


def reset_conv():
    """Permite volver al estado idle (por ejemplo al pulsar el logo)."""
    global _conv_state
    _conv_state = "idle"
    pub_state("idle")


def pub_state(s: str):
    try:
        # Ojo: este hasattr no es perfecto, pero lo dejamos como estaba para no romper nada
        _pub_state.publish(String(data=s) if ROS_AVAILABLE and hasattr(String, "data") else s)
        if ROS_AVAILABLE:
            rospy.loginfo(f"[STATE] {s}")
    except Exception as e:
        print("[STATE publish error]", e)


def pub_status(d: Dict[str, Any]):
    import json
    payload = json.dumps(d, ensure_ascii=False)
    try:
        _pub_status.publish(String(data=payload) if ROS_AVAILABLE and hasattr(String, "data") else payload)
        if ROS_AVAILABLE:
            rospy.loginfo(f"[STATUS] {payload}")
    except Exception as e:
        print("[STATUS publish error]", e)


def pub_error(code: str, msg: str):
    try:
        _pub_error.publish(String(data=code) if ROS_AVAILABLE and hasattr(String, "data") else code)
        if ROS_AVAILABLE:
            rospy.loginfo(f"[ERROR] {code}: {msg}")
    except Exception:
        pass


# === Helper interno: cerrar sesión y volver a idle tras misión ===
def _reset_session_after_mission():
    """
    Pone la sesión en 'no activa' y devuelve el pequeño autómata de conversación a 'idle'.
    Se llama tanto en éxito como en error de la misión.
    """
    global _conv_state
    try:
        if session.is_active():
            session.end_session()
    except Exception:
        # No queremos que un fallo aquí rompa el flujo de la misión
        pass
    _conv_state = "idle"


# === ACTION CLIENT: lanzar misión TirgoPharma ===
def start_mission_async(patient_id: str, med_id: int):
    """
    Lanza una misión de dispensación mediante el ActionServer /tirgo/mission
    sin bloquear el hilo de Flask.
    """
    if not ROS_AVAILABLE or _mission_client is None:
        try:
            rospy.loginfo("[MISSION] Cliente /tirgo/mission no disponible, no se lanza misión")
        except Exception:
            print("[MISSION] Cliente /tirgo/mission no disponible, no se lanza misión")
        pub_error("MISSION_CLIENT_UNAVAILABLE", "El servidor de misión no está disponible")
        _set_mission_status(
            running=False,
            state="ERROR",
            success=False,
            error_code="MISSION_CLIENT_UNAVAILABLE",
            error_message="El servidor de misión no está disponible",
        )
        return

    # Reset y estado inicial de la misión
    _set_mission_status(
        running=True,
        state="GOING_TO_DISPENSER",
        progress=0.0,
        success=None,
        error_code="",
        error_message="",
    )

    goal = TirgoMissionGoal()
    goal.patient_id = patient_id
    goal.med_id = med_id

    def _run():
        try:
            rospy.loginfo(f"[MISSION] Enviando goal: patient_id={patient_id}, med_id={med_id}")
            pub_state("DISPENSING")
            pub_status({"state": "STARTED", "progress": 0.0})
            _mission_client.send_goal(
                goal,
                done_cb=_mission_done_cb,
                feedback_cb=_mission_feedback_cb,
            )
            _mission_client.wait_for_result()
        except Exception as e:
            try:
                rospy.logerr(f"[MISSION] Error en start_mission_async: {e}")
            except Exception:
                print("[MISSION] Error en start_mission_async:", e)
            pub_error("MISSION_CLIENT_FAIL", str(e))
            pub_state("ERROR")
            _set_mission_status(
                running=False,
                state="ERROR",
                success=False,
                error_code="MISSION_CLIENT_FAIL",
                error_message=str(e),
            )
            # Ante fallo gordo también dejamos la sesión en idle
            _reset_session_after_mission()

    threading.Thread(target=_run, daemon=True).start()


def _mission_feedback_cb(feedback):
    """
    Callback de feedback de TirgoMissionAction.
    """
    try:
        rospy.loginfo(f"[MISSION FEEDBACK] state={feedback.state} progress={feedback.progress}")
    except Exception:
        pass

    _set_mission_status(
        running=True,
        state=feedback.state,
        progress=float(getattr(feedback, "progress", 0.0)),
    )

    pub_status(
        {
            "state": feedback.state,
            "progress": feedback.progress,
        }
    )
    # Propagamos el estado tal cual; la web ya decide cómo pintarlo
    pub_state(feedback.state)


def _mission_done_cb(state, result):
    """
    Callback de finalización de TirgoMissionAction.
    """
    if not result.success:
        msg = result.error_message or "Misión fallida"
        code = result.error_code or "MISSION_FAIL"
        try:
            rospy.logwarn(f"[MISSION DONE] FAIL {code}: {msg}")
        except Exception:
            print("[MISSION DONE] FAIL", code, msg)
        pub_error(code, msg)
        pub_state("ERROR")
        _set_mission_status(
            running=False,
            success=False,
            state="ERROR",
            error_code=code,
            error_message=msg,
            progress=0.0,
        )
        # En caso de error, también cerramos la sesión y volvemos a idle
        _reset_session_after_mission()
    else:
        try:
            rospy.loginfo("[MISSION DONE] success ✅")
        except Exception:
            print("[MISSION DONE] success")
        pub_status({"state": "DONE", "progress": 1.0})
        pub_state("IDLE")
        _set_mission_status(
            running=False,
            success=True,
            state="DONE",
            error_code="",
            error_message="",
            progress=1.0,
        )
        # Misión OK -> sesión terminada y conversación a idle
        _reset_session_after_mission()
