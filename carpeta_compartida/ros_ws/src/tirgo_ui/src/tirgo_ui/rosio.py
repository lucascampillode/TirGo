# src/tirgo_ui/rosio.py
import os, socket, subprocess
from urllib.parse import urlparse
from typing import Any, Dict
from .config import STT_TOPIC_DEFAULT, TIRGO_HOTWORD
from . import session

# --- ROS imports / dummies ---
try:
    import rospy
    from std_msgs.msg import String, Bool, Int32
    import actionlib
    # Mensajes de TTS nativo de TIAGo
    from pal_interaction_msgs.msg import TtsAction, TtsGoal, TtsText
    ROS_AVAILABLE = True
except Exception:
    ROS_AVAILABLE = False

    class _DummyPub:
        def publish(self, *_a, **_k): pass

    class _DummyRospy:
        def init_node(self, *a, **k): pass
        def loginfo(self, *a, **k): print(*a)
        def signal_shutdown(self, *a, **k): pass
        def get_param(self, *a, **k): return STT_TOPIC_DEFAULT
        def Subscriber(self, *a, **k): return None
        Publisher = staticmethod(lambda *a, **k: _DummyPub())

    rospy = _DummyRospy()     # type: ignore
    String = str              # type: ignore
    Bool   = bool             # type: ignore
    Int32  = int              # type: ignore
    actionlib = None          # type: ignore
    TtsAction = TtsGoal = TtsText = object  # type: ignore

# publishers de estado
_pub_state = _pub_status = _pub_error = None

# Pub/sub para misión/dispensador
pub_mission_start = None       # /tirgo/mission/start (String)
pub_dispense_req  = None       # /tirgo/dispense/request (Int32)
_last_flags = {"tiago_arrived": False, "dispense_ready": False, "tiago_picked": False}

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
    "diagnostico": ["diagnostico", "diagnóstico", "diagnosticar", "test", "diagnostico guiado", "diagnóstico guiado"]
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

# --- Cliente TTS (action /tts) ---
_tts_client = None  # type: ignore

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
    global _pub_state, _pub_status, _pub_error
    global pub_mission_start, pub_dispense_req
    if ROS_AVAILABLE and master_up():
        try:
            rospy.init_node('tirgo_web_server_v2', anonymous=True, disable_signals=True)
        except Exception:
            pass

        _pub_state  = rospy.Publisher('tirgo/ui/state',  String, queue_size=10)
        _pub_status = rospy.Publisher('tirgo/ui/status', String, queue_size=10)
        _pub_error  = rospy.Publisher('tirgo/ui/error',  String, queue_size=10)

        pub_mission_start = rospy.Publisher('/tirgo/mission/start', String, queue_size=10)
        pub_dispense_req  = rospy.Publisher('/tirgo/dispense/request', Int32,  queue_size=10)

        stt_topic = rospy.get_param("~stt_topic", STT_TOPIC_DEFAULT)
        rospy.Subscriber(stt_topic, String, _stt_cb)
        rospy.Subscriber('/tirgo/tiago/arrived', Bool,  lambda m: _set_flag("tiago_arrived", bool(getattr(m,'data',m))))
        rospy.Subscriber('/tirgo/dispense/ready', Bool, lambda m: _set_flag("dispense_ready", bool(getattr(m,'data',m))))
        rospy.Subscriber('/tirgo/tiago/picked', Bool,  lambda m: _set_flag("tiago_picked", bool(getattr(m,'data',m))))

        # Inicializa el cliente TTS si existe el action server /tts
        try:
            _init_tts()
        except Exception:
            pass

        try: rospy.loginfo(f"[STT] suscrito a {stt_topic}")
        except Exception: pass
    else:
        class DummyPub:
            def publish(self, *_a, **_k): pass
        _pub_state = _pub_status = _pub_error = DummyPub()
        pub_mission_start = pub_dispense_req = DummyPub()


def _init_tts():
    """Inicializa el cliente de la action /tts (TIAGo TTS)."""
    global _tts_client
    if not ROS_AVAILABLE or actionlib is None:
        return
    try:
        _tts_client = actionlib.SimpleActionClient('/tts', TtsAction)
        ok = _tts_client.wait_for_server(rospy.Duration(2.0))
        if ok:
            rospy.loginfo("[TTS] /tts listo ✅")
        else:
            rospy.loginfo("[TTS] /tts no responde (timeout). Fallback a sound_play si está disponible.")
            _tts_client = None
    except Exception as e:
        rospy.loginfo(f"[TTS] no disponible: {e}")
        _tts_client = None


def _set_flag(k: str, v: bool):
    _last_flags[k] = v
    try: rospy.loginfo(f"[FLAG] {k}={v}")
    except Exception: pass


def flags() -> Dict[str, bool]:
    return dict(_last_flags)


def _say_with_tiago(text: str, lang: str = "es_ES", wait_before: float = 0.0):
    """
    Dice 'text' con el TTS nativo de TIAGo (/tts). Si no está, intenta fallback a sound_play.
    """
    # Primero: action /tts
    if ROS_AVAILABLE and _tts_client is not None:
        try:
            goal = TtsGoal()
            goal.rawtext = TtsText(text=text, lang_id=lang)
            goal.speakerName = ""  # por defecto
            goal.wait_before_speaking = wait_before
            _tts_client.send_goal(goal)
            rospy.loginfo(f"[TTS] diciendo: {text}")
            return
        except Exception as e:
            rospy.loginfo(f"[TTS] error con /tts: {e} -> probando sound_play")

    # Segundo: fallback a sound_play (si está instalado en este host)
    try:
        subprocess.Popen(["rosrun", "sound_play", "say.py", text])
        if ROS_AVAILABLE:
            rospy.loginfo(f"[TTS] (fallback sound_play) diciendo: {text}")
    except Exception as e:
        if ROS_AVAILABLE:
            rospy.loginfo(f"[TTS] fallback sound_play falló: {e}")


def _stt_cb(msg: Any):
    """
    Callback de STT:
    - si estamos en idle y oímos el saludo -> TIAGo saluda y pasamos a await_confirm
    - si estamos en await_confirm y oímos un sí -> activamos sesión
    - si la sesión está activa -> detectar intents de navegación (consultar/leer/diagnóstico)
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
            # phonético suave si tu TTS no tiene es_ES real
            _say_with_tiago("Hola, soy Tiago, el robot de TEER-go FAR-mah. Estoy aquí para ah-yu-DAR-teh. ¿Quieres empezar?")
            _conv_state = "await_confirm"
            pub_state("tiago:greeted")
            try: rospy.loginfo(f"[HOTWORD] saludo detectado en: {text_norm}")
            except Exception: pass
            return
        # si no es saludo, ignoramos

    # 2) Esperando confirmación
    elif _conv_state == "await_confirm":
        if any(text_norm.startswith(w) or f" {w}" in f" {text_norm}" for w in CONFIRM_WORDS):
            if not session.is_active():
                session.start_session(op_name="voice_unlock")
            _conv_state = "session"
            pub_state("session:active")
            try: rospy.loginfo(f"[CONV] confirmación recibida: {text_norm}")
            except Exception: pass
            return
        else:
            # Podríamos repreguntar o ignorar. De momento, log e ignorar.
            try: rospy.loginfo(f"[CONV] no es confirmación válida: {text_norm}")
            except Exception: pass
            return

    # 3) Con sesión activa: detectar intents de navegación
    else:
        try: rospy.loginfo(f"[STT] sesión activa, texto: {text_norm}")
        except Exception: pass

        intent = _match_intent(text_norm)
        if intent:
            # guarda para que el front lo lea y navegue
            global _last_voice_nav
            _last_voice_nav = intent
            pub_state(f"voice:navigate:{intent}")
            # (opcional) confirma por voz
            try:
                if intent == "consultar":
                    _say_with_tiago("Entrando en Consultar.")
                elif intent == "leer":
                    _say_with_tiago("Entrando en Leer.")
                elif intent == "diagnostico":
                    _say_with_tiago("Entrando en Diagnóstico.")
            except Exception:
                pass
            return
        # si no hay intent, no hacemos nada especial


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
        _pub_state.publish(String(data=s) if ROS_AVAILABLE and hasattr(String, 'data') else s)
        if ROS_AVAILABLE: rospy.loginfo(f"[STATE] {s}")
    except Exception as e:
        print('[STATE publish error]', e)


def pub_status(d: Dict[str, Any]):
    import json
    payload = json.dumps(d, ensure_ascii=False)
    try:
        _pub_status.publish(String(data=payload) if ROS_AVAILABLE and hasattr(String, 'data') else payload)
        if ROS_AVAILABLE: rospy.loginfo(f"[STATUS] {payload}")
    except Exception as e:
        print('[STATUS publish error]', e)


def pub_error(code: str, msg: str):
    try:
        _pub_error.publish(String(data=code) if ROS_AVAILABLE and hasattr(String, 'data') else code)
        if ROS_AVAILABLE: rospy.loginfo(f"[ERROR] {code}: {msg}")
    except Exception:
        pass
