#!/usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import rospy
import actionlib

from std_msgs.msg import Bool
from tirgo_msgs.msg import TirgoMissionAction, TirgoMissionGoal


class MissionFlowTest(unittest.TestCase):
    """
    Tests de integración del Action Server /tirgo/mission.

    Escenarios:
    - test_happy_path: TIAGo llega y dispensador ready → success = True
    - test_timeout_arrive: TIAGo nunca llega → success = False, error_code = TIMEOUT_ARRIVE
    - test_timeout_ready: TIAGo llega pero dispensador nunca ready → success = False, error_code = TIMEOUT_READY
    """

    @classmethod
    def setUpClass(cls):
        rospy.init_node("test_mission_flow", anonymous=True)

        # Cliente del Action
        cls.client = actionlib.SimpleActionClient("/tirgo/mission", TirgoMissionAction)
        connected = cls.client.wait_for_server(rospy.Duration(10.0))
        if not connected:
            raise RuntimeError("No se ha encontrado el ActionServer /tirgo/mission")

        # Publishers para simular los flags de otros nodos
        cls.pub_arrived = rospy.Publisher("/tirgo/tiago/arrived", Bool, queue_size=1, latch=True)
        cls.pub_ready = rospy.Publisher("/tirgo/dispense/ready", Bool, queue_size=1, latch=True)

        # Pequeña pausa para asegurar conexiones
        rospy.sleep(1.0)

    def _reset_flags(self):
        """
        Pone los flags de arrived/ready a False al inicio de cada test.
        """
        self.pub_arrived.publish(Bool(data=False))
        self.pub_ready.publish(Bool(data=False))
        rospy.sleep(0.2)

    def _send_goal_and_wait(self, simulate_arrived_after=None, simulate_ready_after=None, timeout=20.0):
        """
        Envia un goal y, opcionalmente, programa timers para simular:
        - /tirgo/tiago/arrived = True tras simulate_arrived_after segundos
        - /tirgo/dispense/ready = True tras simulate_ready_after segundos

        Devuelve (state, result).
        """
        self._reset_flags()

        goal = TirgoMissionGoal()
        goal.patient_id = "test_patient"
        goal.med_id = 1

        # Timers para simular señales
        if simulate_arrived_after is not None:
            def _arrived_cb(_evt):
                rospy.loginfo("[TEST] Simulando /tirgo/tiago/arrived = True")
                self.pub_arrived.publish(Bool(data=True))
            rospy.Timer(rospy.Duration(simulate_arrived_after), _arrived_cb, oneshot=True)

        if simulate_ready_after is not None:
            def _ready_cb(_evt):
                rospy.loginfo("[TEST] Simulando /tirgo/dispense/ready = True")
                self.pub_ready.publish(Bool(data=True))
            rospy.Timer(rospy.Duration(simulate_ready_after), _ready_cb, oneshot=True)

        rospy.loginfo("[TEST] Enviando goal al ActionServer...")
        self.client.send_goal(goal)

        finished = self.client.wait_for_result(rospy.Duration(timeout))
        self.assertTrue(finished, "El Action no ha terminado dentro del timeout de test")

        result = self.client.get_result()
        state = self.client.get_state()

        rospy.loginfo("[TEST] Action finalizado. state=%s, result=%s", str(state), str(result))
        return state, result

    # ===========================
    #   TEST 1: CAMINO FELIZ
    # ===========================
    def test_happy_path(self):
        """
        TIAGo llega y el dispensador pasa a ready.
        Esperamos success=True.
        """
        state, result = self._send_goal_and_wait(
            simulate_arrived_after=1.0,
            simulate_ready_after=2.0,
            timeout=15.0,
        )

        self.assertIsNotNone(result, "Result es None en happy path")
        # Si tu server pone estados específicos (p.ej. 3 = SUCCEEDED), puedes comprobarlo:
        # self.assertEqual(state, 3)
        self.assertTrue(result.success, "La misión debería ser success=True en el camino feliz")
        # self.assertIn(result.error_code, ["", "OK", None])

    # ===========================
    #   TEST 2: TIMEOUT ARRIVE
    # ===========================
    def test_timeout_arrive(self):
        """
        No simulamos arrived ni ready → debería saltar TIMEOUT_ARRIVE.
        """
        state, result = self._send_goal_and_wait(
            simulate_arrived_after=None,
            simulate_ready_after=None,
            timeout=15.0,
        )

        self.assertIsNotNone(result, "Result es None en timeout_arrive")
        self.assertFalse(result.success, "En timeout de llegada la misión debería fallar")
        self.assertEqual(result.error_code, "TIMEOUT_ARRIVE")

    # ===========================
    #   TEST 3: TIMEOUT READY
    # ===========================
    def test_timeout_ready(self):
        """
        TIAGo llega, pero el dispensador nunca pasa a ready.
        Esperamos TIMEOUT_READY.
        """
        state, result = self._send_goal_and_wait(
            simulate_arrived_after=1.0,
            simulate_ready_after=None,
            timeout=15.0,
        )

        self.assertIsNotNone(result, "Result es None en timeout_ready")
        self.assertFalse(result.success, "En timeout de ready la misión debería fallar")
        self.assertEqual(result.error_code, "TIMEOUT_READY")


if __name__ == '__main__':
    import rostest
    rostest.rosrun("tirgo_mission_server", "test_mission_flow", MissionFlowTest)
