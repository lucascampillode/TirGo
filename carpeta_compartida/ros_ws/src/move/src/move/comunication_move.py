#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import threading
from std_msgs.msg import Bool, String
from checkpointfollower import Follower


class DispenseProcessStateMachine:
    """
    Máquina de estados del 'Proceso de dispensación' con:
      - Movimiento REAL (Follower) para ir al dispensador y volver al paciente.
      - Espera REAL a que otros nodos publiquen los eventos de hardware:
        /tirgo/dispense/ready, /tirgo/tiago/picked, /tirgo/tiago/delivered.
    """

    PHASE_TEXT = {
        "MOVING_TO_DISPENSER": (
            "Desplazamiento al dispensador",
            "El robot se dirige al módulo de dispensación."
        ),
        "DISPENSING": (
            "Dispensación del medicamento",
            "El sistema prepara y libera el envase con la medicación."
        ),
        "PICKING_CONTAINER": (
            "Recogida del envase",
            "El robot recoge de forma segura el envase dispensado."
        ),
        "RETURNING_TO_PATIENT": (
            "Regreso al paciente",
            "El robot se desplaza de nuevo hacia la ubicación del paciente."
        ),
        "DELIVERING_TO_PATIENT": (
            "Entrega al paciente",
            "El robot entrega el medicamento al paciente."
        ),
        "CLOSING_INTERACTION": (
            "Cierre de la interacción",
            "Se confirma la entrega y el robot se despide."
        ),
    }

    def __init__(self) -> None:
        rospy.init_node("tirgo_dispense_state_machine", anonymous=False)

        # --------- ESTADO INTERNO ---------
        self.state = "IDLE"
        rospy.loginfo("Máquina de estados iniciada. Estado: IDLE (esperando /tirgo/mission/start)")

        # --------- NAVEGACIÓN: FOLLOWER ---------
        # OJO: init_node=False para no volver a llamar a rospy.init_node()
        self.follower = Follower(init_node=False)

        # Posición del paciente (donde empieza la misión, ya está el robot)
        self.checkpoint_paciente = [
            0.18342637607098958,
            1.6793108675387105,
            -0.16066645574411148,
            0.9870087588256882,
        ]

        # Posición del dispensador (donde recoge el medicamento)
        self.checkpoint_dispensador = [
            1.486899128592301,
            -1.9125020321710504,
            -0.7576225009086673,
            0.65269299530245951,
        ]

        # --------- PUBLICADORES (solo los que nacen aquí) ---------
        self.pub_arrived = rospy.Publisher("/tirgo/tiago/arrived", Bool, queue_size=1)
        self.pub_at_patient = rospy.Publisher("/tirgo/tiago/at_patient", Bool, queue_size=1)

        # --------- SUBSCRIPTORES (eventos externos) ---------
        rospy.Subscriber("/tirgo/mission/start", String, self.cb_mission_start, queue_size=1)
        rospy.Subscriber("/tirgo/tiago/arrived", Bool, self.cb_arrived, queue_size=1)
        rospy.Subscriber("/tirgo/dispense/ready", Bool, self.cb_dispense_ready, queue_size=1)
        rospy.Subscriber("/tirgo/tiago/picked", Bool, self.cb_picked, queue_size=1)
        rospy.Subscriber("/tirgo/tiago/at_patient", Bool, self.cb_at_patient, queue_size=1)
        rospy.Subscriber("/tirgo/tiago/delivered", Bool, self.cb_delivered, queue_size=1)

    # ------------------------------------------------------------------
    #  Helpers
    # ------------------------------------------------------------------

    def set_phase(self, new_phase: str) -> None:
        """Actualiza la fase interna y escribe en el log."""
        if new_phase not in self.PHASE_TEXT:
            rospy.logerr("Fase desconocida: %s", new_phase)
            return

        self.state = new_phase
        title, desc = self.PHASE_TEXT[new_phase]

        rospy.loginfo("")
        rospy.loginfo("========== CAMBIO DE FASE ==========")
        rospy.loginfo("Fase actual: %s", title)
        rospy.loginfo("%s", desc)
        rospy.loginfo("====================================")

    def _launch_navigation(self, checkpoint, done_publisher, label: str):
        """
        Lanza en un hilo el movimiento usando Follower y, al finalizar con éxito,
        publica Bool(True) en el topic correspondiente.
        """
        def worker():
            rospy.loginfo("[NAV] Iniciando movimiento: %s", label)
            # Follower espera a que llegue y se pare
            ok = self.follower.enviar_puntos([checkpoint])
            if ok:
                rospy.loginfo("[NAV] Movimiento '%s' completado. Publicando True...", label)
                done_publisher.publish(Bool(data=True))
            else:
                rospy.logwarn("[NAV] Movimiento '%s' NO se completó correctamente.", label)

        t = threading.Thread(target=worker, daemon=True)
        t.start()

    # ------------------------------------------------------------------
    #  Callbacks
    # ------------------------------------------------------------------

    def cb_mission_start(self, msg: String) -> None:
        """Arranca el proceso: movimiento del paciente al dispensador."""
        rospy.loginfo("[mission_start] '%s' (estado=%s)", msg.data, self.state)
        if self.state != "IDLE":
            return

        self.set_phase("MOVING_TO_DISPENSER")

        # Movimiento REAL al dispensador
        self._launch_navigation(
            self.checkpoint_dispensador,
            self.pub_arrived,
            label="al_dispensador"
        )

    def cb_arrived(self, msg: Bool) -> None:
        """El robot llegó al dispensador. Pasamos a DISPENSING."""
        if not msg.data or self.state != "MOVING_TO_DISPENSER":
            return

        rospy.loginfo("[EVENTO] Robot ha llegado al dispensador.")
        self.set_phase("DISPENSING")
        # A partir de aquí, se espera a /tirgo/dispense/ready desde el nodo del dispensador.

    def cb_dispense_ready(self, msg: Bool) -> None:
        """Medicación lista. Pasamos a PICKING."""
        if not msg.data or self.state != "DISPENSING":
            return

        rospy.loginfo("[EVENTO] Dispensador listo.")
        self.set_phase("PICKING_CONTAINER")
        # Se espera a /tirgo/tiago/picked desde el nodo del brazo.

    def cb_picked(self, msg: Bool) -> None:
        """Envase recogido. Pasamos a RETURNING (vuelta al paciente)."""
        if not msg.data or self.state != "PICKING_CONTAINER":
            return

        rospy.loginfo("[EVENTO] Envase recogido.")
        self.set_phase("RETURNING_TO_PATIENT")

        # Movimiento REAL de vuelta al paciente
        self._launch_navigation(
            self.checkpoint_paciente,
            self.pub_at_patient,
            label="al_paciente"
        )

    def cb_at_patient(self, msg: Bool) -> None:
        """Llegada al paciente. Pasamos a DELIVERING."""
        if not msg.data or self.state != "RETURNING_TO_PATIENT":
            return

        rospy.loginfo("[EVENTO] Junto al paciente.")
        self.set_phase("DELIVERING_TO_PATIENT")
        # Se espera a /tirgo/tiago/delivered desde el nodo de HRI / diálogo.

    def cb_delivered(self, msg: Bool) -> None:
        """Entrega completada. Fin."""
        if not msg.data or self.state != "DELIVERING_TO_PATIENT":
            return

        rospy.loginfo("[EVENTO] Medicamento entregado.")
        self.set_phase("CLOSING_INTERACTION")
        rospy.loginfo("--- PROCESO COMPLETADO ---")
        # Vuelve a admitir otra misión
        self.state = "IDLE"
        rospy.loginfo("Máquina de estados lista para una nueva misión (estado=IDLE).")


def main():
    sm = DispenseProcessStateMachine()
    rospy.loginfo(
        "Máquina de estados lista.\n"
        "Lanza una misión con:\n"
        "  rostopic pub /tirgo/mission/start std_msgs/String 'go' -1"
    )
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
