# test/test_tirgo_mission.py
#
# Tests UNITARIOS de TirgoMissionServer SIN ROS real.
# - Falsificamos rospy, actionlib, std_msgs y tirgo_msgs.
# - Importamos la clase desde scripts/tirgo_mission_server.py
# - Probamos:
#   * callbacks de flags
#   * _wait_flag (éxito, timeout, preempt)

import sys
import types
from pathlib import Path

import pytest

# ==========================
# 1) FAKES de rospy / actionlib / msgs
# ==========================

# --- Tiempo fake ---

class FakeDuration:
    def __init__(self, t):
        self._t = float(t)

    def to_sec(self):
        return self._t


class FakeTime:
    def __init__(self, t=0.0):
        self._t = float(t)

    def __sub__(self, other):
        # devolvemos un FakeDuration para que .to_sec() funcione
        return FakeDuration(self._t - other._t)


# "reloj global" para controlar el tiempo en los tests
_FAKE_NOW = [0.0]


def fake_now():
    # Devuelve un FakeTime con el valor actual del “reloj”
    return FakeTime(_FAKE_NOW[0])


class FakeTimeClass:
    """
    Imitamos la interfaz de rospy.Time para lo que usa tu código:
    - rospy.Time.now() → objeto tipo tiempo con __sub__ que devuelve Duration.
    """
    @staticmethod
    def now():
        return fake_now()



class FakeRate:
    def __init__(self, hz):
        self.hz = hz

    def sleep(self):
        # avanzamos el tiempo un poquito en cada iteración
        _FAKE_NOW[0] += 0.1


def fake_is_shutdown():
    # nunca paramos en estos tests
    return False


def fake_get_param(name, default=None):
    # usamos siempre el valor por defecto de los timeouts
    return default


def fake_loginfo(*_args, **_kwargs):
    pass


def fake_logwarn(*_args, **_kwargs):
    pass


def fake_init_node(*_args, **_kwargs):
    pass


class FakePublisher:
    def __init__(self, *_args, **_kwargs):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class FakeSubscriber:
    def __init__(self, *_args, **_kwargs):
        # no necesitamos guardar nada
        pass

fake_rospy = types.SimpleNamespace(
    Time=FakeTimeClass,      # 👈 aquí el cambio importante
    Rate=FakeRate,
    get_param=fake_get_param,
    loginfo=fake_loginfo,
    logwarn=fake_logwarn,
    is_shutdown=fake_is_shutdown,
    init_node=fake_init_node,
    Publisher=FakePublisher,
    Subscriber=FakeSubscriber,
)


# --- actionlib fake ---

class FakeSimpleActionServer:
    def __init__(self, name, action_type, execute_cb=None, auto_start=True):
        self.name = name
        self.action_type = action_type
        self._execute_cb = execute_cb
        self.auto_start = auto_start

        self.preempted = False
        self.succeeded = False
        self.aborted = False

        self.feedback_list = []
        self.result_obj = None

    def start(self):
        # en los tests no hacemos nada especial
        pass

    def publish_feedback(self, fb):
        self.feedback_list.append(fb)

    def is_preempt_requested(self):
        return self.preempted

    def set_preempted(self, res=None):
        self.preempted = True
        self.result_obj = res

    def set_aborted(self, res=None):
        self.aborted = True
        self.result_obj = res

    def set_succeeded(self, res=None):
        self.succeeded = True
        self.result_obj = res


fake_actionlib = types.SimpleNamespace(
    SimpleActionServer=FakeSimpleActionServer
)

# --- Mensajes fake std_msgs / tirgo_msgs ---

class Bool:
    def __init__(self, data=False):
        self.data = data


class Int32:
    def __init__(self, data=0):
        self.data = data


class String:
    def __init__(self, data=""):
        self.data = data


fake_std_msgs = types.ModuleType("std_msgs.msg")
fake_std_msgs.Bool = Bool
fake_std_msgs.Int32 = Int32
fake_std_msgs.String = String

class FakeFeedback:
    def __init__(self):
        self.state = ""
        self.progress = 0.0


class FakeResult:
    def __init__(self):
        self.success = False
        self.error_code = ""
        self.error_message = ""


fake_tirgo_msgs = types.ModuleType("tirgo_msgs.msg")
fake_tirgo_msgs.TirgoMissionAction = object
fake_tirgo_msgs.TirgoMissionFeedback = FakeFeedback
fake_tirgo_msgs.TirgoMissionResult = FakeResult

# Registramos los módulos fake en sys.modules ANTES de importar la clase real
sys.modules["rospy"] = fake_rospy
sys.modules["actionlib"] = fake_actionlib
sys.modules["std_msgs.msg"] = fake_std_msgs
sys.modules["tirgo_msgs.msg"] = fake_tirgo_msgs

# ==========================
# 2) Importar TirgoMissionServer desde scripts/
# ==========================

CURRENT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = CURRENT_DIR.parent
SCRIPTS_DIR = PROJECT_ROOT / "scripts"

sys.path.insert(0, str(SCRIPTS_DIR))

from tirgo_mission_server import TirgoMissionServer  # noqa: E402


# ==========================
# 3) FIXTURE base para los tests
# ==========================

@pytest.fixture
def server():
    """Devuelve una instancia fresca de TirgoMissionServer con el 'reloj' a 0."""
    _FAKE_NOW[0] = 0.0
    srv = TirgoMissionServer()
    return srv


# ==========================
# 4) Tests de callbacks de flags
# ==========================

def test_callbacks_set_flags(server):
    """Las callbacks deben actualizar sus flags correspondientes."""
    # por defecto todo debe estar a False
    assert server.arrived_flag is False
    assert server.ready_flag is False
    assert server.picked_flag is False
    assert server.at_patient_flag is False
    assert server.delivered_flag is False
    assert server.farewell_flag is False

    # arrived
    server.cb_arrived(Bool(True))
    assert server.arrived_flag is True

    # ready
    server.cb_ready(Bool(True))
    assert server.ready_flag is True

    # picked
    server.cb_picked(Bool(True))
    assert server.picked_flag is True

    # at_patient
    server.cb_at_patient(Bool(True))
    assert server.at_patient_flag is True

    # delivered
    server.cb_delivered(Bool(True))
    assert server.delivered_flag is True

    # farewell
    server.cb_farewell(Bool(True))
    assert server.farewell_flag is True


# ==========================
# 5) Tests de _wait_flag
# ==========================

def test_wait_flag_success_when_flag_is_true(server):
    """
    Si el flag ya está a True, _wait_flag debe devolver True sin esperar al timeout.
    """
    server.arrived_flag = True
    result = server._wait_flag("arrived_flag", timeout=5.0, label="arrived")
    assert result is True


def test_wait_flag_timeout_when_flag_never_true(server):
    """
    Si el flag nunca se pone a True, _wait_flag devuelve False cuando se supera el timeout.
    """
    server.arrived_flag = False

    # timeout pequeño para que pase rápido
    result = server._wait_flag("arrived_flag", timeout=0.15, label="arrived")
    assert result is False


def test_wait_flag_preempted_returns_none(server):
    """
    Si el ActionServer marca preempt mientras esperamos, _wait_flag debe devolver None.
    """
    # simulamos que alguien ha pedido preempt
    server._as.preempted = True

    result = server._wait_flag("arrived_flag", timeout=5.0, label="arrived")
    assert result is None


def test_execute_cb_bad_goal_empty_patient(server):
    """
    patient_id vacío debe provocar BAD_GOAL inmediatamente.
    """
    goal = types.SimpleNamespace(patient_id="", med_id=3)

    # capturamos qué hace set_aborted
    aborted = {}
    server._as.set_aborted = lambda res: aborted.setdefault("res", res)

    server.execute_cb(goal)

    assert aborted["res"].success is False
    assert aborted["res"].error_code == "BAD_GOAL"
    assert "patient_id" in aborted["res"].error_message.lower()


def test_execute_cb_bad_goal_invalid_med_id(server):
    """
    med_id <= 0 debe provocar BAD_GOAL.
    """
    goal = types.SimpleNamespace(patient_id="abc123", med_id=0)

    aborted = {}
    server._as.set_aborted = lambda res: aborted.setdefault("res", res)

    server.execute_cb(goal)

    assert aborted["res"].success is False
    assert aborted["res"].error_code == "BAD_GOAL"
    assert "med_id" in aborted["res"].error_message.lower()
