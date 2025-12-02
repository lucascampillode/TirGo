#!/usr/bin/env python3
import rospy
import actionlib

from std_msgs.msg import Bool, Int32, String
from tirgo_msgs.msg import TirgoMissionAction, TirgoMissionFeedback, TirgoMissionResult


class TirgoMissionServer:
    """
    Action Server /tirgo/mission

    - Orquesta la misión (flags, estados, timeouts).
    - NO toca Mongo ni stock. Eso se hace en tirgo_ui al recibir la petición 'pedir'.
    """

    def __init__(self):
        # Parámetros de timeout (segundos)
        self.timeout_arrive   = rospy.get_param("~timeout_arrive",   120.0)
        self.timeout_ready    = rospy.get_param("~timeout_ready",     60.0)
        self.timeout_pick     = rospy.get_param("~timeout_pick",      30.0)
        self.timeout_patient  = rospy.get_param("~timeout_patient",  120.0)
        self.timeout_deliver  = rospy.get_param("~timeout_deliver",   30.0)
        self.timeout_farewell = rospy.get_param("~timeout_farewell",  30.0)

        self._as = actionlib.SimpleActionServer(
            "/tirgo/mission",
            TirgoMissionAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )

        self.pub_mission_start = rospy.Publisher(
            "/tirgo/mission/start", String, queue_size=1
        )
        self.pub_dispense_req = rospy.Publisher(
            "/tirgo/dispense/request", Int32, queue_size=1
        )

        self.arrived_flag     = False
        self.ready_flag       = False
        self.picked_flag      = False
        self.at_patient_flag  = False
        self.delivered_flag   = False
        self.farewell_flag    = False

        self.sub_arrived = rospy.Subscriber(
            "/tirgo/tiago/arrived", Bool, self.cb_arrived
        )
        self.sub_ready = rospy.Subscriber(
            "/tirgo/dispense/ready", Bool, self.cb_ready
        )
        self.sub_picked = rospy.Subscriber(
            "/tirgo/tiago/picked", Bool, self.cb_picked
        )
        self.sub_at_patient = rospy.Subscriber(
            "/tirgo/tiago/at_patient", Bool, self.cb_at_patient
        )
        self.sub_delivered = rospy.Subscriber(
            "/tirgo/tiago/delivered", Bool, self.cb_delivered
        )
        self.sub_farewell = rospy.Subscriber(
            "/tirgo/tiago/farewell_done", Bool, self.cb_farewell
        )

        rospy.loginfo("[TMS] Iniciando TirgoMissionServer...")
        self._as.start()
        rospy.loginfo("[TMS] TirgoMissionServer listo en /tirgo/mission")

    # --- Callbacks de flags ---
    def cb_arrived(self, msg: Bool):
        self.arrived_flag = bool(msg.data)
        rospy.loginfo(f"[TMS] /tirgo/tiago/arrived = {self.arrived_flag}")

    def cb_ready(self, msg: Bool):
        self.ready_flag = bool(msg.data)
        rospy.loginfo(f"[TMS] /tirgo/dispense/ready = {self.ready_flag}")

    def cb_picked(self, msg: Bool):
        self.picked_flag = bool(msg.data)
        rospy.loginfo(f"[TMS] /tirgo/tiago/picked = {self.picked_flag}")

    def cb_at_patient(self, msg: Bool):
        self.at_patient_flag = bool(msg.data)
        rospy.loginfo(f"[TMS] /tirgo/tiago/at_patient = {self.at_patient_flag}")

    def cb_delivered(self, msg: Bool):
        self.delivered_flag = bool(msg.data)
        rospy.loginfo(f"[TMS] /tirgo/tiago/delivered = {self.delivered_flag}")

    def cb_farewell(self, msg: Bool):
        self.farewell_flag = bool(msg.data)
        rospy.loginfo(f"[TMS] /tirgo/tiago/farewell_done = {self.farewell_flag}")

    # --- Lógica del Action Server ---
    def execute_cb(self, goal):
        """
        goal.patient_id (string)  -> en tu flujo: dni_hash (u otro identificador)
        goal.med_id (int32)      -> en tu flujo actual: BIN_ID físico
        """
        rospy.loginfo(
            f"[TMS] Nuevo goal: patient_id={goal.patient_id}, med_id(bin_id)={goal.med_id}"
        )

        fb = TirgoMissionFeedback()
        res = TirgoMissionResult()

        self.arrived_flag     = False
        self.ready_flag       = False
        self.picked_flag      = False
        self.at_patient_flag  = False
        self.delivered_flag   = False
        self.farewell_flag    = False

        # 1) GOING_TO_DISPENSER
        fb.state = "GOING_TO_DISPENSER"
        fb.progress = 0.1
        self._as.publish_feedback(fb)

        self.arrived_flag = False
        self.pub_mission_start.publish(String(data="start"))
        rospy.loginfo("[TMS] GOING_TO_DISPENSER -> esperando arrived_flag...")

        result = self.wait_for_arrive()
        if result is None:
            res.success = False
            res.error_code = "PREEMPTED"
            res.error_message = "Misión cancelada por el cliente"
            self._as.set_preempted(res)
            return
        if result is False:
            res.success = False
            res.error_code = "TIMEOUT_ARRIVE"
            res.error_message = "TIAGo no ha llegado al dispensador a tiempo"
            self._as.set_aborted(res)
            return

        # 2) WAITING_DISPENSE
        fb.state = "WAITING_DISPENSE"
        fb.progress = 0.3
        self._as.publish_feedback(fb)

        self.ready_flag = False
        self.pub_dispense_req.publish(Int32(data=goal.med_id))
        rospy.loginfo("[TMS] WAITING_DISPENSE -> esperando ready_flag...")

        result = self.wait_for_ready()
        if result is None:
            res.success = False
            res.error_code = "PREEMPTED"
            res.error_message = "Misión cancelada por el cliente"
            self._as.set_preempted(res)
            return
        if result is False:
            res.success = False
            res.error_code = "TIMEOUT_READY"
            res.error_message = "El dispensador no ha marcado ready a tiempo"
            self._as.set_aborted(res)
            return

        # 3) PICKING_UP
        fb.state = "PICKING_UP"
        fb.progress = 0.5
        self._as.publish_feedback(fb)

        self.picked_flag = False
        rospy.loginfo("[TMS] PICKING_UP -> esperando picked_flag...")
        result = self.wait_for_picked()
        if result is None:
            res.success = False
            res.error_code = "PREEMPTED"
            res.error_message = "Misión cancelada por el cliente"
            self._as.set_preempted(res)
            return
        if result is False:
            res.success = False
            res.error_code = "TIMEOUT_PICK"
            res.error_message = "El robot no ha confirmado la recogida del envase"
            self._as.set_aborted(res)
            return

        # 4) GOING_TO_PATIENT
        fb.state = "GOING_TO_PATIENT"
        fb.progress = 0.7
        self._as.publish_feedback(fb)

        self.at_patient_flag = False
        rospy.loginfo("[TMS] GOING_TO_PATIENT -> esperando at_patient_flag...")
        result = self.wait_for_at_patient()
        if result is None:
            res.success = False
            res.error_code = "PREEMPTED"
            res.error_message = "Misión cancelada por el cliente"
            self._as.set_preempted(res)
            return
        if result is False:
            res.success = False
            res.error_code = "TIMEOUT_PATIENT"
            res.error_message = "El robot no ha llegado a la zona del paciente a tiempo"
            self._as.set_aborted(res)
            return

        # 5) AT_PATIENT
        fb.state = "AT_PATIENT"
        fb.progress = 0.85
        self._as.publish_feedback(fb)

        self.delivered_flag = False
        rospy.loginfo("[TMS] AT_PATIENT -> esperando delivered_flag...")
        result = self.wait_for_delivered()
        if result is None:
            res.success = False
            res.error_code = "PREEMPTED"
            res.error_message = "Misión cancelada por el cliente"
            self._as.set_preempted(res)
            return
        if result is False:
            res.success = False
            res.error_code = "TIMEOUT_DELIVER"
            res.error_message = "El robot no ha confirmado la entrega al paciente"
            self._as.set_aborted(res)
            return

        # 6) FAREWELL
        fb.state = "FAREWELL"
        fb.progress = 0.95
        self._as.publish_feedback(fb)

        self.farewell_flag = False
        rospy.loginfo("[TMS] FAREWELL -> esperando farewell_flag...")
        result = self.wait_for_farewell()
        if result is None:
            res.success = False
            res.error_code = "PREEMPTED"
            res.error_message = "Misión cancelada por el cliente"
            self._as.set_preempted(res)
            return
        if result is False:
            res.success = False
            res.error_code = "TIMEOUT_FAREWELL"
            res.error_message = "El robot no ha confirmado el fin de la despedida"
            self._as.set_aborted(res)
            return

        # FIN: éxito
        fb.state = "DONE"
        fb.progress = 1.0
        self._as.publish_feedback(fb)

        res.success = True
        res.error_code = ""
        res.error_message = ""
        self._as.set_succeeded(res)
        rospy.loginfo("[TMS] Misión completada con éxito")

    # --- Helper genérico ---
    def _wait_flag(self, attr_name: str, timeout: float, label: str):
        start = rospy.Time.now()
        rate = rospy.Rate(10)

        rospy.loginfo(f"[TMS] Esperando flag '{label}' (timeout={timeout}s)...")

        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                rospy.logwarn(f"[TMS] Preempt solicitado mientras esperaba '{label}'")
                return None

            if getattr(self, attr_name):
                rospy.loginfo(f"[TMS] Flag '{label}' = True recibido")
                return True

            elapsed = (rospy.Time.now() - start).to_sec()
            if elapsed > timeout:
                rospy.logwarn(f"[TMS] TIMEOUT en flag '{label}' tras {elapsed:.1f}s")
                return False

            rate.sleep()

    def wait_for_arrive(self):
        return self._wait_flag("arrived_flag", self.timeout_arrive, "arrived")

    def wait_for_ready(self):
        return self._wait_flag("ready_flag", self.timeout_ready, "ready")

    def wait_for_picked(self):
        return self._wait_flag("picked_flag", self.timeout_pick, "picked")

    def wait_for_at_patient(self):
        return self._wait_flag("at_patient_flag", self.timeout_patient, "at_patient")

    def wait_for_delivered(self):
        return self._wait_flag("delivered_flag", self.timeout_deliver, "delivered")

    def wait_for_farewell(self):
        return self._wait_flag("farewell_flag", self.timeout_farewell, "farewell_done")


def main():
    rospy.init_node("tirgo_mission_server")
    server = TirgoMissionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
