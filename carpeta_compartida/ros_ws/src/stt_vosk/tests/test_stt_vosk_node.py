# tests/test_stt_vosk_node.py
import sys
import types
import json
from pathlib import Path
import importlib.util

import pytest


# ==========================
# FAKES / DOBLES GLOBALES
# ==========================

# --- Publicadores capturados por topic ---
PUBLISHERS = {}


class DummyPublisher:
    def __init__(self, topic, *_, **__):
        self.topic = topic
        self.messages = []
        PUBLISHERS[topic] = self

    def publish(self, msg):
        # En el nodo original se publican strings "normales"
        # (no std_msgs/String), así que guardamos tal cual
        self.messages.append(getattr(msg, "data", msg))


class DummyStream:
    def __init__(self):
        self.read_calls = 0

    def start_stream(self):
        pass

    def read(self, chunk, exception_on_overflow=False):
        # Devolvemos "audio" falso
        self.read_calls += 1
        return b"x" * chunk

    def stop_stream(self):
        pass

    def close(self):
        pass


class DummyPyAudio:
    def __init__(self):
        self.open_called = False

    def open(self, *_, **__):
        self.open_called = True
        return DummyStream()

    def terminate(self):
        pass


# --- Estado para el recognizer fake de Vosk ---
VOSK_STATE = {
    "accept_sequence": [],  # lista de bools que devolverá AcceptWaveform
    "texts": [],            # textos finales para Result()
    "partials": [],         # textos parciales para PartialResult()
}


class DummyModel:
    def __init__(self, path):
        self.path = path


class DummyRecognizer:
    def __init__(self, model, rate):
        self.model = model
        self.rate = rate

    def SetWords(self, flag):
        # No necesitamos hacer nada
        pass

    def AcceptWaveform(self, data):
        seq = VOSK_STATE.get("accept_sequence", [])
        if seq:
            return seq.pop(0)
        return False

    def Result(self):
        texts = VOSK_STATE.get("texts", [])
        txt = texts.pop(0) if texts else ""
        return json.dumps({"text": txt})

    def PartialResult(self):
        partials = VOSK_STATE.get("partials", [])
        txt = partials.pop(0) if partials else ""
        return json.dumps({"partial": txt})


# --- std_msgs.msg.String fake (por si en algún momento lo usas) ---
fake_std_msgs = types.ModuleType("std_msgs")
fake_std_msgs_msg = types.ModuleType("std_msgs.msg")


class String:
    def __init__(self, data=""):
        self.data = data


fake_std_msgs_msg.String = String
fake_std_msgs.msg = fake_std_msgs_msg


# --- rospy fake ---
class DummyRospy(types.ModuleType):
    def __init__(self):
        super().__init__("rospy")
        self.params = {}
        self._shutdown_loops = 1  # cuántas veces devuelve False antes de True
        self.loginfo_msgs = []
        self.logerr_msgs = []

    def init_node(self, *_, **__):
        # No hacemos nada especial
        pass

    def get_param(self, name, default=None):
        return self.params.get(name, default)

    def loginfo(self, msg, *a):
        self.loginfo_msgs.append(str(msg))

    def logerr(self, msg, *a):
        self.logerr_msgs.append(str(msg))

    def Publisher(self, topic, *a, **kw):
        return DummyPublisher(topic, *a, **kw)

    def is_shutdown(self):
        if self._shutdown_loops > 0:
            self._shutdown_loops -= 1
        else:
            return True
        return False


dummy_rospy = DummyRospy()

# --- Módulos falsos inyectados en sys.modules ANTES de cargar el nodo ---
fake_pyaudio = types.ModuleType("pyaudio")
fake_pyaudio.PyAudio = DummyPyAudio
fake_pyaudio.paInt16 = 8  # valor cualquiera

fake_vosk = types.ModuleType("vosk")
fake_vosk.Model = DummyModel
fake_vosk.KaldiRecognizer = DummyRecognizer

sys.modules.setdefault("rospy", dummy_rospy)
sys.modules.setdefault("pyaudio", fake_pyaudio)
sys.modules.setdefault("vosk", fake_vosk)
sys.modules.setdefault("std_msgs", fake_std_msgs)
sys.modules.setdefault("std_msgs.msg", fake_std_msgs_msg)


# ==========================
# FIXTURE: cargar stt_vosk_node con los mocks
# ==========================

@pytest.fixture
def stt_module():
    """
    Carga scripts/stt_vosk_node.py como módulo, usando los módulos falsos
    (rospy, pyaudio, vosk) definidos arriba.
    """
    # Resetear estado global entre tests
    PUBLISHERS.clear()
    VOSK_STATE["accept_sequence"] = []
    VOSK_STATE["texts"] = []
    VOSK_STATE["partials"] = []
    dummy_rospy.params.clear()
    dummy_rospy._shutdown_loops = 1
    dummy_rospy.loginfo_msgs.clear()
    dummy_rospy.logerr_msgs.clear()

    script_path = (
        Path(__file__).resolve().parents[1] / "scripts" / "stt_vosk_node.py"
    )
    assert script_path.exists(), f"No se encontró {script_path}"

    spec = importlib.util.spec_from_file_location(
        "stt_vosk_node_tested", script_path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)  # type: ignore

    return module


# ==========================
# TESTS
# ==========================

def _set_common_params(publish_partial=True, wake_word=""):
    """
    Parámetros ROS que usa el nodo.
    """
    dummy_rospy.params.update(
        {
            "~model_path": "/fake/model",
            "~sample_rate": 16000,
            "~device_index": None,
            "~chunk": 8000,
            "~publish_partial": publish_partial,
            "~wake_word": wake_word,
        }
    )


def test_hotword_publica_wake_una_sola_vez(stt_module):
    """
    Comprobar que al detectar la hotword se publica el evento correcto
    (mensaje '__WAKE__') solo una vez en stt/text.
    """
    _set_common_params(publish_partial=True, wake_word="hola tirgo")
    dummy_rospy._shutdown_loops = 1  # 1 iteración del bucle

    # El modelo "existe"
    stt_module.os.path.isdir = lambda p: True

    # Simulamos 1 resultado final con la hotword
    VOSK_STATE["accept_sequence"] = [True]
    VOSK_STATE["texts"] = ["hola tirgo que tal"]

    stt_module.main()

    text_pub = PUBLISHERS.get("stt/text")
    assert text_pub is not None, "No se creó el publisher stt/text"

    msgs = text_pub.messages
    # Debe haber como mínimo 2 mensajes: "__WAKE__" + texto completo
    assert "__WAKE__" in msgs
    assert msgs.count("__WAKE__") == 1, "La hotword se publicó más de una vez"


def test_publica_parciales_cuando_publish_partial_true(stt_module):
    """
    Comprobar publicación de parciales cuando publish_partial=True.
    """
    _set_common_params(publish_partial=True, wake_word="")
    dummy_rospy._shutdown_loops = 1

    stt_module.os.path.isdir = lambda p: True

    # accept_sequence=False -> entra en rama de parciales
    VOSK_STATE["accept_sequence"] = [False]
    VOSK_STATE["partials"] = ["hola parc", "hola parcial entera"]

    stt_module.main()

    partial_pub = PUBLISHERS.get("stt/partial")
    assert partial_pub is not None, "No se creó el publisher stt/partial"

    msgs = partial_pub.messages
    assert msgs, "No se publicó ningún parcial"
    assert "hola parc" in msgs or "hola parcial entera" in msgs


def test_no_publica_parciales_cuando_publish_partial_false(stt_module):
    """
    No publicar parciales cuando publish_partial=False.
    """
    _set_common_params(publish_partial=False, wake_word="")
    dummy_rospy._shutdown_loops = 1

    stt_module.os.path.isdir = lambda p: True

    VOSK_STATE["accept_sequence"] = [False]
    VOSK_STATE["partials"] = ["esto NO debería publicarse"]

    stt_module.main()

    partial_pub = PUBLISHERS.get("stt/partial")
    # El publisher existe, pero no debe tener mensajes
    assert partial_pub is not None
    assert partial_pub.messages == [], "Se publicaron parciales aunque publish_partial=False"


def test_modelo_inexistente_loggea_error_y_sale_limpio(stt_module):
    """
    Manejo de error al inicializar el modelo Vosk:
    - Si el directorio no existe, debe loggear error y salir sin lanzar excepciones.
    """
    _set_common_params(publish_partial=True, wake_word="hola tirgo")

    # Forzamos que el modelo "no exista"
    stt_module.os.path.isdir = lambda p: False

    # Si main() lanzase una excepción, el test fallaría
    stt_module.main()

    # Debe haberse registrado un log de error
    assert dummy_rospy.logerr_msgs, "No se ha logueado ningún error"
    assert "Modelo Vosk no encontrado" in dummy_rospy.logerr_msgs[0]

    # Y no debería haberse creado ningún publisher
    assert "stt/text" not in PUBLISHERS
    assert "stt/partial" not in PUBLISHERS
