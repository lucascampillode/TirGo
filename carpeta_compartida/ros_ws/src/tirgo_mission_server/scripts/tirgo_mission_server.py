#!/usr/bin/env python3
import rospy
import actionlib

from std_msgs.msg import Bool, Int32, String
from tirgo_msgs.msg import TirgoMissionAction, TirgoMissionFeedback, TirgoMissionResult


class TirgoMissionServer:
    """
    Action Server /tirgo/mission

    Responsabilidad:
    - Expone el Action Server.
    - Escucha los flags de TIAGo y el dispensador:
        /tirgo/tiago/arrived        (Bool)  -> ha llegado al dispensador
        /tirgo/dispense/ready       (Bool)  -> dispensador ha terminado
        /tirgo/tiago/picked         (Bool)  -> el robot ha cogido el envase
        /tirgo/tiago/at_patient     (Bool)  -> el robot ha llegado al paciente
        /tirgo/tiago/delivered      (Bool)  -> ha entregado el envase
        /tirgo/tiago/farewell_done  (Bool)  -> ha terminado la despedida

    - Publica feedback.state y feedback.progress con la secuencia:

        GOING_TO_DISPENSER
        WAITING_DISPENSE
        PICKING_UP
        GOING_TO_PATIENT
        AT_PATIENT
        FAREWELL
        DONE

    - Termina con success o error_code (TIMEOUT_ARRIVE, TIMEOUT_READY, etc.).
    """

    def __init__(self):
        # Parámetros de timeout (segundos)
        self.timeout_arrive   = rospy.get_param("~timeout_arrive",   120.0)
        self.timeout_ready    = rospy.get_param("~timeout_ready",     60.0)
        self.timeout_pick     = rospy.get_param("~timeout_pick",      30.0)
        self.timeout_patient  = rospy.get_param("~timeout_patient",  120.0)
        self.timeout_deliver  = rospy.get_param("~timeout_deliver",   30.0)
        self.timeout_farewell = rospy.get_param("~timeout_farewell",  30.0)

        # Action Server
        self._as = actionlib.SimpleActionServer(
            "/tirgo/mission",
            TirgoMissionAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )

        # Publishers de alto nivel (los usan otros nodos)
        self.pub_mission_start = rospy.Publisher(
            "/tirgo/mission/start", String, queue_size=1
        )
        self.pub_dispense_req = rospy.Publisher(
            "/tirgo/dispense/request", Int32, queue_size=1
        )

        # Flags que vendrán de otros nodos (tú solo te suscribes)
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

    # ==========================
    # Callbacks de tópicos
    # ==========================
    def cb_arrived(self, msg: Bool):
        """Lo publica el nodo de navegación (llegada al dispensador)."""
        self.arrived_flag = bool(msg.data)
        rospy.loginfo(f"[TMS] /tirgo/tiago/arrived = {self.arrived_flag}")

    def cb_ready(self, msg: Bool):
        """Lo publica el nodo del dispensador (dispensación completada)."""
        self.ready_flag = bool(msg.data)
        rospy.loginfo(f"[TMS] /tirgo/dispense/ready = {self.ready_flag}")

    def cb_picked(self, msg: Bool):
        """El robot confirma que ha cogido el envase."""
        self.picked_flag = bool(msg.data)
        rospy.loginfo(f"[TMS] /tirgo/tiago/picked = {self.picked_flag}")

    def cb_at_patient(self, msg: Bool):
        """El robot indica que ha llegado a la zona del paciente."""
        self.at_patient_flag = bool(msg.data)
        rospy.loginfo(f"[TMS] /tirgo/tiago/at_patient = {self.at_patient_flag}")

    def cb_delivered(self, msg: Bool):
        """El robot indica que ha entregado el envase al paciente."""
        self.delivered_flag = bool(msg.data)
        rospy.loginfo(f"[TMS] /tirgo/tiago/delivered = {self.delivered_flag}")

    def cb_farewell(self, msg: Bool):
        """El robot indica que ha terminado la despedida."""
        self.farewell_flag = bool(msg.data)
        rospy.loginfo(f"[TMS] /tirgo/tiago/farewell_done = {self.farewell_flag}")

    # ==========================
    # Lógica del Action Server
    # ==========================
    def execute_cb(self, goal):
        """
        goal.patient_id (string)
        goal.med_id (int32)
        """
        rospy.loginfo(
            f"[TMS] Nuevo goal: patient_id={goal.patient_id}, med_id={goal.med_id}"
        )

        fb = TirgoMissionFeedback()
        res = TirgoMissionResult()

        # Reset completo de flags al inicio de cada misión
        self.arrived_flag     = False
        self.ready_flag       = False
        self.picked_flag      = False
        self.at_patient_flag  = False
        self.delivered_flag   = False
        self.farewell_flag    = False

        # ----- Paso 1: lanzar misión hacia el dispensador -----
        fb.state = "GOING_TO_DISPENSER"
        fb.progress = 0.1
        self._as.publish_feedback(fb)

        # IMPORTANTE: reseteamos la flag justo antes de esperar
        self.arrived_flag = False
        self.pub_mission_start.publish(String(data="start"))
        rospy.loginfo("[TMS] GOING_TO_DISPENSER -> esperando arrived_flag...")

        if not self.wait_for_arrive():
            res.success = False
            res.error_code = "TIMEOUT_ARRIVE"
            res.error_message = "TIAGo no ha llegado al dispensador a tiempo"
            self._as.set_aborted(res)
            return

        # ----- Paso 2: pedir dispensación -----
        fb.state = "WAITING_DISPENSE"
        fb.progress = 0.3
        self._as.publish_feedback(fb)

        self.ready_flag = False
        self.pub_dispense_req.publish(Int32(data=goal.med_id))
        rospy.loginfo("[TMS] WAITING_DISPENSE -> esperando ready_flag...")

        if not self.wait_for_ready():
            res.success = False
            res.error_code = "TIMEOUT_READY"
            res.error_message = "El dispensador no ha marcado ready a tiempo"
            self._as.set_aborted(res)
            return

        # ----- Paso 3: robot recoge el envase -----
        fb.state = "PICKING_UP"
        fb.progress = 0.5
        self._as.publish_feedback(fb)

        self.picked_flag = False
        rospy.loginfo("[TMS] PICKING_UP -> esperando picked_flag...")

        if not self.wait_for_picked():
            res.success = False
            res.error_code = "TIMEOUT_PICK"
            res.error_message = "El robot no ha confirmado la recogida del envase"
            self._as.set_aborted(res)
            return

        # ----- Paso 4: robot va hacia el paciente -----
        fb.state = "GOING_TO_PATIENT"
        fb.progress = 0.7
        self._as.publish_feedback(fb)

        self.at_patient_flag = False
        rospy.loginfo("[TMS] GOING_TO_PATIENT -> esperando at_patient_flag...")

        if not self.wait_for_at_patient():
            res.success = False
            res.error_code = "TIMEOUT_PATIENT"
            res.error_message = "El robot no ha llegado a la zona del paciente a tiempo"
            self._as.set_aborted(res)
            return

        # ----- Paso 5: entrega al paciente -----
        fb.state = "AT_PATIENT"
        fb.progress = 0.85
        self._as.publish_feedback(fb)

        self.delivered_flag = False
        rospy.loginfo("[TMS] AT_PATIENT -> esperando delivered_flag...")

        if not self.wait_for_delivered():
            res.success = False
            res.error_code = "TIMEOUT_DELIVER"
            res.error_message = "El robot no ha confirmado la entrega al paciente"
            self._as.set_aborted(res)
            return

        # ----- Paso 6: despedida -----
        fb.state = "FAREWELL"
        fb.progress = 0.95
        self._as.publish_feedback(fb)

        self.farewell_flag = False
        rospy.loginfo("[TMS] FAREWELL -> esperando farewell_flag...")

        if not self.wait_for_farewell():
            res.success = False
            res.error_code = "TIMEOUT_FAREWELL"
            res.error_message = "El robot no ha confirmado el fin de la despedida"
            self._as.set_aborted(res)
            return

        # ----- Fin: éxito -----
        fb.state = "DONE"
        fb.progress = 1.0
        self._as.publish_feedback(fb)

        res.success = True
        res.error_code = ""
        res.error_message = ""
        self._as.set_succeeded(res)
        rospy.loginfo("[TMS] Misión completada con éxito")

    # ==========================
    # Helpers
    # ==========================
    def check_preempt(self) -> bool:
        """Comprueba si el cliente ha cancelado la misión."""
        if self._as.is_preempt_requested():
            res = TirgoMissionResult()
            res.success = False
            res.error_code = "PREEMPTED"
            res.error_message = "Misión cancelada por el cliente"
            self._as.set_preempted(res)
            rospy.logwarn("[TMS] Misión cancelada por el cliente")
            return True
        return False

    def _wait_flag(self, attr_name: str, timeout: float, label: str) -> bool:
        """
        Helper genérico: espera a que self.<attr_name> se ponga True
        o se alcance el timeout.
        """
        start = rospy.Time.now()
        rate = rospy.Rate(10)

        rospy.loginfo(f"[TMS] Esperando flag '{label}' (timeout={timeout}s)...")

        while not rospy.is_shutdown():
            if self.check_preempt():
                return False

            if getattr(self, attr_name):
                rospy.loginfo(f"[TMS] Flag '{label}' = True recibido")
                return True

            elapsed = (rospy.Time.now() - start).to_sec()
            if elapsed > timeout:
                rospy.logwarn(f"[TMS] TIMEOUT en flag '{label}' tras {elapsed:.1f}s")
                return False

            rate.sleep()

    def wait_for_arrive(self) -> bool:
        return self._wait_flag("arrived_flag", self.timeout_arrive, "arrived")

    def wait_for_ready(self) -> bool:
        return self._wait_flag("ready_flag", self.timeout_ready, "ready")

    def wait_for_picked(self) -> bool:
        return self._wait_flag("picked_flag", self.timeout_pick, "picked")

    def wait_for_at_patient(self) -> bool:
        return self._wait_flag("at_patient_flag", self.timeout_patient, "at_patient")

    def wait_for_delivered(self) -> bool:
        return self._wait_flag("delivered_flag", self.timeout_deliver, "delivered")

    def wait_for_farewell(self) -> bool:
        return self._wait_flag("farewell_flag", self.timeout_farewell, "farewell_done")


def main():
    rospy.init_node("tirgo_mission_server")
    server = TirgoMissionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
