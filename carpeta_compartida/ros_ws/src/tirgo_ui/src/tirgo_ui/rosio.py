# src/tirgo_ui/rosio.py
import os, socket
from urllib.parse import urlparse
from typing import Any, Dict
from .config import STT_TOPIC_DEFAULT, TIRGO_HOTWORD
from . import session

try:
    import rospy
    from std_msgs.msg import String, Bool, Int32
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

_pub_state = _pub_status = _pub_error = None

# Pub/sub para misión/dispensador
pub_mission_start = None       # /tirgo/mission/start (String)
pub_dispense_req  = None       # /tirgo/dispense/request (Int32)
_last_flags = {"tiago_arrived": False, "dispense_ready": False, "tiago_picked": False}

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

        try: rospy.loginfo(f"[STT] suscrito a {stt_topic}")
        except Exception: pass
    else:
        class DummyPub:
            def publish(self, *_a, **_k): pass
        _pub_state = _pub_status = _pub_error = DummyPub()
        pub_mission_start = pub_dispense_req = DummyPub()

def _set_flag(k: str, v: bool):
    _last_flags[k] = v
    try: rospy.loginfo(f"[FLAG] {k}={v}")
    except Exception: pass

def flags() -> Dict[str, bool]:
    return dict(_last_flags)

def _stt_cb(msg: Any):
    """Callback de STT: si oímos la hotword, abrimos sesión de operación."""
    try:
        txt = msg.data if hasattr(msg, "data") else str(msg)
    except Exception:
        txt = str(msg)
    text_norm = (txt or "").strip().lower()

    if TIRGO_HOTWORD.lower() in text_norm:
        if not session.is_active():
            session.start_session(op_name="voice_unlock")
            try: rospy.loginfo(f"[HOTWORD] '{TIRGO_HOTWORD}' detectada en: {text_norm}")
            except Exception: pass
            pub_state(f"hotword:{TIRGO_HOTWORD}")
        else:
            try: rospy.loginfo("[HOTWORD] sesión ya activa, ignorando nuevo unlock")
            except Exception: pass

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
