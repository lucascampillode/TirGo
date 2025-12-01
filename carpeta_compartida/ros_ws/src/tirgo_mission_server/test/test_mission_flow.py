#!/usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import sys
import rospy
import actionlib

from std_msgs.msg import Bool, String, Int32
from tirgo_msgs.msg import TirgoMissionAction, TirgoMissionGoal


class MissionFlowTest(unittest.TestCase):
    """
    Tests de integración del Action Server /tirgo/mission.

    Cubre estos escenarios del TirgoMissionServer (versión definitiva):

    - Camino feliz completo:
        /tirgo/tiago/arrived        -> GOING_TO_DISPENSER OK
        /tirgo/dispense/ready       -> WAITING_DISPENSE OK
        /tirgo/tiago/picked         -> PICKING_UP OK
        /tirgo/tiago/at_patient     -> GOING_TO_PATIENT / AT_PATIENT OK
        /tirgo/tiago/delivered      -> entrega OK
        /tirgo/tiago/farewell_done  -> despedida OK

    - Timeouts por fase:
        TIMEOUT_ARRIVE
        TIMEOUT_READY
        TIMEOUT_PICK
        TIMEOUT_PATIENT
        TIMEOUT_DELIVER
        TIMEOUT_FAREWELL

    - PREEMPTED cuando el cliente cancela el goal.

    También comprueba en happy path:
    - Publicación en /tirgo/mission/start
    - Publicación en /tirgo/dispense/request con el med_id correcto
    - Secuencia de feedback.state coherente
    """

    # Resumen para el log final con ticks
    RESULTS = {
        "happy_path": False,
        "timeout_arrive": False,
        "timeout_ready": False,
        "timeout_pick": False,
        "timeout_patient": False,
        "timeout_deliver": False,
        "timeout_farewell": False,
        "preempted": False,
    }

    @classmethod
    def setUpClass(cls):
        rospy.init_node("test_mission_flow", anonymous=True)

        # Cliente del Action
        cls.client = actionlib.SimpleActionClient("/tirgo/mission", TirgoMissionAction)
        connected = cls.client.wait_for_server(rospy.Duration(10.0))
        if not connected:
            raise RuntimeError("No se ha encontrado el ActionServer /tirgo/mission")

        # Publishers para simular TODOS los flags del server definitivo
        cls.pub_arrived = rospy.Publisher(
            "/tirgo/tiago/arrived", Bool, queue_size=1, latch=True
        )
        cls.pub_ready = rospy.Publisher(
            "/tirgo/dispense/ready", Bool, queue_size=1, latch=True
        )
        cls.pub_picked = rospy.Publisher(
            "/tirgo/tiago/picked", Bool, queue_size=1, latch=True
        )
        cls.pub_at_patient = rospy.Publisher(
            "/tirgo/tiago/at_patient", Bool, queue_size=1, latch=True
        )
        cls.pub_delivered = rospy.Publisher(
            "/tirgo/tiago/delivered", Bool, queue_size=1, latch=True
        )
        cls.pub_farewell = rospy.Publisher(
            "/tirgo/tiago/farewell_done", Bool, queue_size=1, latch=True
        )

        # Suscriptores para ver topics de alto nivel
        cls.mission_start_msgs = []
        cls.dispense_req_msgs = []

        def _cb_mission_start(msg):
            cls.mission_start_msgs.append(msg)

        def _cb_dispense_req(msg):
            cls.dispense_req_msgs.append(msg)

        cls.sub_mission_start = rospy.Subscriber(
            "/tirgo/mission/start", String, _cb_mission_start
        )
        cls.sub_dispense_req = rospy.Subscriber(
            "/tirgo/dispense/request", Int32, _cb_dispense_req
        )

        rospy.sleep(1.0)  # dejar que todo conecte

    # ===========================
    # Helpers internos
    # ===========================
    def _reset_flags(self):
        """
        Pone todos los flags a False. El server además
        los resetea internamente, pero esto deja el bus limpio.
        """
        self.__class__.pub_arrived.publish(Bool(data=False))
        self.__class__.pub_ready.publish(Bool(data=False))
        self.__class__.pub_picked.publish(Bool(data=False))
        self.__class__.pub_at_patient.publish(Bool(data=False))
        self.__class__.pub_delivered.publish(Bool(data=False))
        self.__class__.pub_farewell.publish(Bool(data=False))
        rospy.sleep(0.2)

    def _reset_observed(self):
        """
        Resetea feedback + mensajes observados.
        """
        self.feedback_states = []
        self.feedback_progress = []

        self.__class__.mission_start_msgs[:] = []
        self.__class__.dispense_req_msgs[:] = []

    def _feedback_cb(self, fb):
        self.feedback_states.append(fb.state)
        self.feedback_progress.append(fb.progress)

    def _send_goal_and_wait(
        self,
        simulate_arrived_after=None,
        simulate_ready_after=None,
        simulate_picked_after=None,
        simulate_at_patient_after=None,
        simulate_delivered_after=None,
        simulate_farewell_after=None,
        timeout=20.0,
    ):
        """
        Envía un goal al ActionServer y programa timers para ir
        levantando los flags en el orden que quieres.

        Cada parámetro *_after es el segundo (desde el envío del goal)
        en el que se publica ese flag a True.
        """
        self._reset_flags()
        self._reset_observed()

        goal = TirgoMissionGoal()
        goal.patient_id = "test_patient"
        goal.med_id = 1

        # arrived
        if simulate_arrived_after is not None:
            def _arrived_cb(_):
                rospy.loginfo("[TEST] Simulando arrived=True")
                self.__class__.pub_arrived.publish(Bool(True))
            rospy.Timer(rospy.Duration(simulate_arrived_after), _arrived_cb, oneshot=True)

        # ready
        if simulate_ready_after is not None:
            def _ready_cb(_):
                rospy.loginfo("[TEST] Simulando ready=True")
                self.__class__.pub_ready.publish(Bool(True))
            rospy.Timer(rospy.Duration(simulate_ready_after), _ready_cb, oneshot=True)

        # picked
        if simulate_picked_after is not None:
            def _picked_cb(_):
                rospy.loginfo("[TEST] Simulando picked=True")
                self.__class__.pub_picked.publish(Bool(True))
            rospy.Timer(rospy.Duration(simulate_picked_after), _picked_cb, oneshot=True)

        # at_patient
        if simulate_at_patient_after is not None:
            def _atp_cb(_):
                rospy.loginfo("[TEST] Simulando at_patient=True")
                self.__class__.pub_at_patient.publish(Bool(True))
            rospy.Timer(rospy.Duration(simulate_at_patient_after), _atp_cb, oneshot=True)

        # delivered
        if simulate_delivered_after is not None:
            def _del_cb(_):
                rospy.loginfo("[TEST] Simulando delivered=True")
                self.__class__.pub_delivered.publish(Bool(True))
            rospy.Timer(rospy.Duration(simulate_delivered_after), _del_cb, oneshot=True)

        # farewell_done
        if simulate_farewell_after is not None:
            def _fw_cb(_):
                rospy.loginfo("[TEST] Simulando farewell_done=True")
                self.__class__.pub_farewell.publish(Bool(True))
            rospy.Timer(rospy.Duration(simulate_farewell_after), _fw_cb, oneshot=True)

        rospy.loginfo("[TEST] Enviando goal al ActionServer...")
        self.__class__.client.send_goal(goal, feedback_cb=self._feedback_cb)

        finished = self.__class__.client.wait_for_result(rospy.Duration(timeout))
        self.assertTrue(finished, "El Action no ha terminado dentro del timeout de test")

        result = self.__class__.client.get_result()
        state = self.__class__.client.get_state()
        rospy.loginfo("[TEST] Action finalizado. state=%s, result=%s", str(state), str(result))
        return state, result

    # ===========================
    #   TEST 1: CAMINO FELIZ
    # ===========================
    def test_happy_path(self):
        state, result = self._send_goal_and_wait(
            simulate_arrived_after=1.0,
            simulate_ready_after=2.0,
            simulate_picked_after=3.0,
            simulate_at_patient_after=4.0,
            simulate_delivered_after=5.0,
            simulate_farewell_after=6.0,
            timeout=30.0,
        )

        # Result
        self.assertIsNotNone(result, "Result es None en happy path")
        self.assertTrue(result.success, "La misión debería terminar con éxito")
        self.assertEqual(result.error_code, "")

        # /tirgo/mission/start se publica
        self.assertGreaterEqual(
            len(self.__class__.mission_start_msgs), 1,
            "No se ha publicado nada en /tirgo/mission/start en el happy path",
        )

        # /tirgo/dispense/request con med_id=1
        self.assertGreaterEqual(
            len(self.__class__.dispense_req_msgs), 1,
            "No se ha publicado nada en /tirgo/dispense/request en el happy path",
        )
        last_req = self.__class__.dispense_req_msgs[-1]
        self.assertEqual(
            last_req.data, 1,
            "El med_id publicado en /tirgo/dispense/request no es el esperado (1)",
        )

        # Feedback: estados en orden
        expected_states = [
            "GOING_TO_DISPENSER",
            "WAITING_DISPENSE",
            "PICKING_UP",
            "GOING_TO_PATIENT",
            "AT_PATIENT",
            "FAREWELL",
            "DONE",
        ]
        states = self.feedback_states

        self.assertGreater(
            len(states), 0,
            "No se ha recibido ningún estado de feedback en happy path",
        )

        idx = 0
        for est in expected_states:
            while idx < len(states) and states[idx] != est:
                idx += 1
            self.assertLess(
                idx, len(states),
                f"No se ha encontrado el estado de feedback '{est}' en orden",
            )
            idx += 1

        self.__class__.RESULTS["happy_path"] = True

    # ===========================
    #   TEST 2: TIMEOUT ARRIVE
    # ===========================
    def test_timeout_arrive(self):
        state, result = self._send_goal_and_wait(
            simulate_arrived_after=None,
            simulate_ready_after=None,
            simulate_picked_after=None,
            simulate_at_patient_after=None,
            simulate_delivered_after=None,
            simulate_farewell_after=None,
            timeout=15.0,
        )

        self.assertIsNotNone(result, "Result es None en timeout_arrive")
        self.assertFalse(result.success, "En timeout de llegada la misión debería fallar")
        self.assertEqual(result.error_code, "TIMEOUT_ARRIVE")

        self.__class__.RESULTS["timeout_arrive"] = True

    # ===========================
    #   TEST 3: TIMEOUT READY
    # ===========================
    def test_timeout_ready(self):
        state, result = self._send_goal_and_wait(
            simulate_arrived_after=1.0,
            simulate_ready_after=None,
            simulate_picked_after=None,
            simulate_at_patient_after=None,
            simulate_delivered_after=None,
            simulate_farewell_after=None,
            timeout=20.0,
        )

        self.assertIsNotNone(result, "Result es None en timeout_ready")
        self.assertFalse(result.success, "En timeout de ready la misión debería fallar")
        self.assertEqual(result.error_code, "TIMEOUT_READY")

        self.__class__.RESULTS["timeout_ready"] = True

    # ===========================
    #   TEST 4: TIMEOUT PICK
    # ===========================
    def test_timeout_pick(self):
        state, result = self._send_goal_and_wait(
            simulate_arrived_after=1.0,
            simulate_ready_after=2.0,
            simulate_picked_after=None,
            simulate_at_patient_after=None,
            simulate_delivered_after=None,
            simulate_farewell_after=None,
            timeout=25.0,
        )

        self.assertIsNotNone(result, "Result es None en timeout_pick")
        self.assertFalse(result.success, "En timeout de pick la misión debería fallar")
        self.assertEqual(result.error_code, "TIMEOUT_PICK")

        self.__class__.RESULTS["timeout_pick"] = True

    # ===========================
    #   TEST 5: TIMEOUT PATIENT
    # ===========================
    def test_timeout_patient(self):
        state, result = self._send_goal_and_wait(
            simulate_arrived_after=1.0,
            simulate_ready_after=2.0,
            simulate_picked_after=3.0,
            simulate_at_patient_after=None,
            simulate_delivered_after=None,
            simulate_farewell_after=None,
            timeout=25.0,
        )

        self.assertIsNotNone(result, "Result es None en timeout_patient")
        self.assertFalse(result.success, "En timeout de patient la misión debería fallar")
        self.assertEqual(result.error_code, "TIMEOUT_PATIENT")

        self.__class__.RESULTS["timeout_patient"] = True

    # ===========================
    #   TEST 6: TIMEOUT DELIVER
    # ===========================
    def test_timeout_deliver(self):
        state, result = self._send_goal_and_wait(
            simulate_arrived_after=1.0,
            simulate_ready_after=2.0,
            simulate_picked_after=3.0,
            simulate_at_patient_after=4.0,
            simulate_delivered_after=None,
            simulate_farewell_after=None,
            timeout=30.0,
        )

        self.assertIsNotNone(result, "Result es None en timeout_deliver")
        self.assertFalse(result.success, "En timeout de deliver la misión debería fallar")
        self.assertEqual(result.error_code, "TIMEOUT_DELIVER")

        self.__class__.RESULTS["timeout_deliver"] = True

    # ===========================
    #   TEST 7: TIMEOUT FAREWELL
    # ===========================
    def test_timeout_farewell(self):
        state, result = self._send_goal_and_wait(
            simulate_arrived_after=1.0,
            simulate_ready_after=2.0,
            simulate_picked_after=3.0,
            simulate_at_patient_after=4.0,
            simulate_delivered_after=5.0,
            simulate_farewell_after=None,
            timeout=30.0,
        )

        self.assertIsNotNone(result, "Result es None en timeout_farewell")
        self.assertFalse(result.success, "En timeout de farewell la misión debería fallar")
        self.assertEqual(result.error_code, "TIMEOUT_FAREWELL")

        self.__class__.RESULTS["timeout_farewell"] = True

    # ===========================
    #   TEST 8: PREEMPTED
    # ===========================
    def test_preempted(self):
        """
        Enviamos un goal y lo cancelamos desde el cliente.
        Esperamos PREEMPTED (el server llama a set_preempted con ese error_code).
        """
        self._reset_flags()
        self._reset_observed()

        goal = TirgoMissionGoal()
        goal.patient_id = "test_patient"
        goal.med_id = 1

        rospy.loginfo("[TEST] Enviando goal para preempt...")
        self.__class__.client.send_goal(goal, feedback_cb=self._feedback_cb)

        def _cancel_cb(_):
            rospy.loginfo("[TEST] Cancelando misión desde el cliente")
            self.__class__.client.cancel_goal()

        rospy.Timer(rospy.Duration(2.0), _cancel_cb, oneshot=True)

        finished = self.__class__.client.wait_for_result(rospy.Duration(20.0))
        self.assertTrue(finished, "El Action no ha terminado tras cancel_goal")

        result = self.__class__.client.get_result()
        self.assertIsNotNone(result, "Result es None en preempt")
        self.assertFalse(result.success, "En PREEMPTED success debería ser False")
        self.assertEqual(result.error_code, "PREEMPTED")

        self.__class__.RESULTS["preempted"] = True

    # ===========================
    #   RESUMEN FINAL
    # ===========================
    @classmethod
    def tearDownClass(cls):
        # Usamos print para que SIEMPRE salga por consola de rostest
        lines = []
        lines.append("")
        lines.append("===========================================")
        lines.append(" RESUMEN DE TESTS: TirgoMissionServer")
        lines.append("===========================================")

        nombres_bonitos = {
            "happy_path": "Camino feliz completo",
            "timeout_arrive": "Timeout llegada al dispensador",
            "timeout_ready": "Timeout dispensador ready",
            "timeout_pick": "Timeout recogida envase",
            "timeout_patient": "Timeout llegada a paciente",
            "timeout_deliver": "Timeout entrega al paciente",
            "timeout_farewell": "Timeout despedida",
            "preempted": "Misión cancelada (PREEMPTED)",
        }

        for key, ok in cls.RESULTS.items():
            mark = "✅" if ok else "❌"
            desc = nombres_bonitos.get(key, key)
            lines.append(f"{mark} {desc}")

        lines.append("===========================================")
        text = "\n".join(lines) + "\n"
        sys.stdout.write(text)
        sys.stdout.flush()

        # Por si acaso también lo mandamos a rosout
        for l in lines:
            rospy.loginfo(l)


if __name__ == '__main__':
    import rostest
    rostest.rosrun("tirgo_mission_server", "test_mission_flow", MissionFlowTest)
