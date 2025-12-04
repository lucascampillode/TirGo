import unittest
import rospy
from std_msgs.msg import Int32, Bool

# -------------------------------------------------------------------
#  Si se ejecuta con pytest en el portátil, saltamos TODO el módulo.
#  Con rostest (rosunit + unittest) esto NO se ejecuta porque no hay pytest.
# -------------------------------------------------------------------
try:
    import pytest  # type: ignore
except ImportError:
    pytest = None

if pytest is not None:
    pytest.skip(
        "Tests de flujo ROS para servo_dispenser: ejecutar sólo vía rostest en la Raspberry.",
        allow_module_level=True,
    )


class ServoDispenserFlowTest(unittest.TestCase):
    """
    Test de flujo ROS completo para servo_dispenser.

    Se usa en el .test de rostest:
      <test pkg="servo_dispenser" type="test_servo_dispenser_flow.py" ... />
    """

    @classmethod
    def setUpClass(cls):
        rospy.init_node("test_servo_dispenser_flow", anonymous=True)

        # Publisher hacia el nodo de dispensador
        cls.pub_req = rospy.Publisher(
            "/tirgo/dispense/request", Int32, queue_size=10
        )

        cls.ready_msgs = []

        def _ready_cb(msg):
            cls.ready_msgs.append(bool(msg.data))

        cls.sub_ready = rospy.Subscriber(
            "/tirgo/dispense/ready", Bool, _ready_cb
        )

        # Dar un poco de tiempo a que se conecten los pubs/subs
        rospy.sleep(1.0)

    @classmethod
    def tearDownClass(cls):
        cls.sub_ready.unregister()
        # No apagamos el nodo aquí; lo hace rostest.

    def _clear_ready(self):
        type(self).ready_msgs = []

    # -------------------------------
    #  TESTS
    # -------------------------------

    def test_valid_bin_publishes_ready_once(self):
        """
        Publicar un bin válido (1..4) debe provocar:
          - EXACTAMENTE un ready=True.
        (Requiere que el nodo servo_dispenser_node esté lanzado en este test)
        """
        self._clear_ready()

        self.pub_req.publish(Int32(data=1))
        rospy.sleep(1.0)  # tiempo para que el nodo procese

        count_true = self.ready_msgs.count(True)
        self.assertEqual(
            count_true,
            1,
            f"Se esperaba un único ready=True, pero se recibieron {count_true}",
        )

    def test_invalid_bin_never_publishes_ready(self):
        """
        Publicar un bin inválido NO debe emitir ready=True.
        """
        self._clear_ready()

        self.pub_req.publish(Int32(data=999))
        rospy.sleep(1.0)

        count_true = self.ready_msgs.count(True)
        self.assertEqual(
            count_true,
            0,
            f"No se esperaba ningún ready=True, pero se recibieron {count_true}",
        )


if __name__ == "__main__":
    import rostest

    rostest.rosrun("servo_dispenser", "servo_dispenser_flow", ServoDispenserFlowTest)
