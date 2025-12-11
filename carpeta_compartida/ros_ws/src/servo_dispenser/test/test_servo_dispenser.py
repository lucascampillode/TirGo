import sys
from pathlib import Path

from std_msgs.msg import Int32

# ==============================
#  Stub de pigpio para el entorno de tests
#  (en el portátil no hace falta pigpio real)
# ==============================
import types

if "pigpio" not in sys.modules:
    fake_pigpio_mod = types.SimpleNamespace(
        pi=lambda *args, **kwargs: None,  # nunca lo usamos en tests
        OUTPUT=0,
    )
    sys.modules["pigpio"] = fake_pigpio_mod

# ==============================
#  Cargar servo_dispenser_node
# ==============================
CURRENT_DIR = Path(__file__).resolve().parent
PKG_ROOT = CURRENT_DIR.parent
SCRIPTS_DIR = PKG_ROOT / "scripts"

if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import servo_dispenser_node as sd_mod  # type: ignore
from servo_dispenser_node import (     # type: ignore
    ServoDispenser,
    SERVO_A,
    SERVO_B,
    MIN_US,
    MAX_US,
    NEUTRAL,
)

# ==============================
#  DOBLES / FAKES
# ==============================

class FakePi:
    """
    Fake muy simple de pigpio.pi():
    - Guarda todas las llamadas a set_servo_pulsewidth.
    """
    def __init__(self):
        self.calls = []  # lista de tuplas (gpio, us)

    def set_mode(self, *_args, **_kwargs):
        pass

    def set_servo_pulsewidth(self, gpio, us):
        self.calls.append((gpio, us))

    def stop(self):
        pass


class DummyPublisher:
    """
    Publisher falso para capturar lo que se publica en /tirgo/dispense/ready.
    """
    def __init__(self):
        self.published = []

    def publish(self, msg):
        # std_msgs/Bool o lo que sea -> nos quedamos con .data si existe
        data = getattr(msg, "data", msg)
        self.published.append(data)


fake_pi = FakePi()


def make_server():
    """
    Creamos un ServoDispenser SIN pasar por __init__ para evitar ROS/pigpio reales.
    Le inyectamos nuestra FakePi y un publisher falso.
    """
    sd = ServoDispenser.__new__(ServoDispenser)

    # Inyección de dependencias mínimas
    sd.pi = fake_pi
    sd.pub_ready = DummyPublisher()

    # No llamamos a __init__, pero los métodos de instancia
    # (push_A/B/C/D, angle_to_us, etc.) se resuelven desde la clase.
    return sd


# ==============================
#  TESTS UNITARIOS
# ==============================

def test_angle_to_us_respects_limits():
    """
    angle_to_us debe devolver pulsos dentro de [MIN_US, MAX_US] y alrededor de NEUTRAL.
    """
    sd = make_server()

    for angle in (-180, -90, 0, 90, 180):
        us = sd.angle_to_us(angle)
        assert MIN_US <= us <= MAX_US

    # 0 grados debe ir cerca de NEUTRAL
    assert abs(sd.angle_to_us(0) - NEUTRAL) < 10


def test_valid_bins_move_correct_servo_and_publish_ready():
    """
    bin_id válidos (1..4) deben:
      - mover el servo correcto (A o B)
      - publicar EXACTAMENTE un ready=True
    Según tu nodo:
      1 → A
      2 → A
      3 → B
      4 → B
    """
    from std_msgs.msg import Bool

    sd = make_server()

    # Asegurarnos de que no hay estado previo
    fake_pi.calls.clear()
    sd.pub_ready.published.clear()

    # -------- bin 1 y 2 → SERVO_A --------
    for bin_id in (1, 2):
        fake_pi.calls.clear()
        sd.pub_ready.published.clear()

        sd.cb_dispense(Int32(data=bin_id))

        # Debe haber habido movimientos sobre SERVO_A (al menos 1 pulso)
        assert len(fake_pi.calls) >= 1
        gpios = {gpio for (gpio, _us) in fake_pi.calls}
        assert gpios == {SERVO_A}

        # READY publicado exactamente una vez y a True
        assert sd.pub_ready.published == [True]

    # -------- bin 3 y 4 → SERVO_B --------
    for bin_id in (3, 4):
        fake_pi.calls.clear()
        sd.pub_ready.published.clear()

        sd.cb_dispense(Int32(data=bin_id))

        assert len(fake_pi.calls) >= 1
        gpios = {gpio for (gpio, _us) in fake_pi.calls}
        assert gpios == {SERVO_B}
        assert sd.pub_ready.published == [True]


def test_invalid_bin_does_not_move_servo_and_no_ready():
    """
    bin_id inválido:
      - NO debe mover ningún servo
      - NO debe publicar READY
    """
    sd = make_server()

    fake_pi.calls.clear()
    sd.pub_ready.published.clear()

    sd.cb_dispense(Int32(data=999))  # inválido

    assert fake_pi.calls == []
    assert sd.pub_ready.published == []
