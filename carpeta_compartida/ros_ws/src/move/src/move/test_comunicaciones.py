#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool, String


class DispenseProcessStateMachine:
    """
    Máquina de estados del 'Proceso de dispensación'.

    Fases:
      1) MOVING_TO_DISPENSER        -> Desplazamiento al dispensador
      2) DISPENSING                 -> Dispensación del medicamento
      3) PICKING_CONTAINER          -> Recogida del envase
      4) RETURNING_TO_PATIENT       -> Regreso al paciente
      5) DELIVERING_TO_PATIENT      -> Entrega al paciente
      6) CLOSING_INTERACTION        -> Cierre de la interacción
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

        # --------- PUBLICADORES DE ETAPAS COMPLETADAS ---------
        self.pub_arrived = rospy.Publisher(
            "/tirgo/tiago/arrived", Bool, queue_size=1
        )
        self.pub_picked = rospy.Publisher(
            "/tirgo/tiago/picked", Bool, queue_size=1
        )
        self.pub_at_patient = rospy.Publisher(
            "/tirgo/tiago/at_patient", Bool, queue_size=1
        )
        self.pub_delivered = rospy.Publisher(
            "/tirgo/tiago/delivered", Bool, queue_size=1
        )

        # --------- SUBSCRIPTORES ---------
        rospy.Subscriber(
            "/tirgo/mission/start",
            String,
            self.cb_mission_start,
            queue_size=1,
        )

        # Llegada al dispensador (evento externo: navegación)
        rospy.Subscriber(
            "/tirgo/tiago/arrived",
            Bool,
            self.cb_arrived,
            queue_size=1,
        )

        # Medicación lista en el dispensador (evento dispensador)
        rospy.Subscriber(
            "/tirgo/dispense/ready",
            Bool,
            self.cb_dispense_ready,
            queue_size=1,
        )

        # Estas tres también nos llegan de otros módulos,
        # pero además las republicamos cuando se completa cada fase.
        rospy.Subscriber(
            "/tirgo/tiago/picked",
            Bool,
            self.cb_picked,
            queue_size=1,
        )
        rospy.Subscriber(
            "/tirgo/tiago/at_patient",
            Bool,
            self.cb_at_patient,
            queue_size=1,
        )
        rospy.Subscriber(
            "/tirgo/tiago/delivered",
            Bool,
            self.cb_delivered,
            queue_size=1,
        )

    # ------------------------------------------------------------------
    #  Helpers
    # ------------------------------------------------------------------

    def set_phase(self, new_phase: str) -> None:
        """Actualiza la fase interna y escribe en el log el texto de la UI."""
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

    # ------------------------------------------------------------------
    #  Callbacks de los topics
    # ------------------------------------------------------------------

    def cb_mission_start(self, msg: String) -> None:
        """
        /tirgo/mission/start

        Arranca el proceso: IDLE -> MOVING_TO_DISPENSER.
        """
        rospy.loginfo("[mission_start] '%s' (estado=%s)", msg.data, self.state)

        if self.state != "IDLE":
            rospy.logwarn("mission_start recibido pero la máquina no está en IDLE (estado=%s)", self.state)
            return

        self.set_phase("MOVING_TO_DISPENSER")

    def cb_arrived(self, msg: Bool) -> None:
        """
        /tirgo/tiago/arrived

        Indica que el robot ha llegado al dispensador.
        Terminamos la fase 'Desplazamiento al dispensador' y pasamos a 'Dispensación'.
        Además publicamos /tirgo/tiago/arrived = True para notificar la etapa completada.
        """
        rospy.loginfo("[arrived] %s (estado=%s)", msg.data, self.state)

        if not msg.data:
            return

        if self.state == "MOVING_TO_DISPENSER":
            # Cambio de fase
            self.set_phase("DISPENSING")
            # Publicamos que esta etapa ha terminado
            self.pub_arrived.publish(Bool(data=True))
        else:
            rospy.logwarn("arrived recibido en fase inesperada (estado=%s)", self.state)

    def cb_dispense_ready(self, msg: Bool) -> None:
        """
        /tirgo/dispense/ready

        Indica que el envase con la medicación ya está listo.
        Pasamos a 'Recogida del envase'.
        """
        rospy.loginfo("[dispense_ready] %s (estado=%s)", msg.data, self.state)

        if not msg.data:
            return

        if self.state in ["DISPENSING", "MOVING_TO_DISPENSER"]:
            self.set_phase("PICKING_CONTAINER")
        else:
            rospy.logwarn("dispense_ready recibido en fase inesperada (estado=%s)", self.state)

    def cb_picked(self, msg: Bool) -> None:
        """
        /tirgo/tiago/picked

        Indica que el robot ya ha recogido el envase.
        Terminamos 'Recogida del envase' y pasamos a 'Regreso al paciente'.
        También publicamos /tirgo/tiago/picked = True.
        """
        rospy.loginfo("[picked] %s (estado=%s)", msg.data, self.state)

        if not msg.data:
            return

        if self.state == "PICKING_CONTAINER":
            self.set_phase("RETURNING_TO_PATIENT")
            self.pub_picked.publish(Bool(data=True))
        else:
            rospy.logwarn("picked recibido en fase inesperada (estado=%s)", self.state)

    def cb_at_patient(self, msg: Bool) -> None:
        """
        /tirgo/tiago/at_patient

        Indica que el robot ha llegado al paciente.
        Terminamos 'Regreso al paciente' y pasamos a 'Entrega al paciente'.
        Publicamos /tirgo/tiago/at_patient = True.
        """
        rospy.loginfo("[at_patient] %s (estado=%s)", msg.data, self.state)

        if not msg.data:
            return

        if self.state == "RETURNING_TO_PATIENT":
            self.set_phase("DELIVERING_TO_PATIENT")
            self.pub_at_patient.publish(Bool(data=True))
        else:
            rospy.logwarn("at_patient recibido en fase inesperada (estado=%s)", self.state)

    def cb_delivered(self, msg: Bool) -> None:
        """
        /tirgo/tiago/delivered

        Indica que la medicación se ha entregado al paciente.
        Terminamos 'Entrega al paciente' y pasamos a 'Cierre de la interacción'.
        Publicamos /tirgo/tiago/delivered = True.
        """
        rospy.loginfo("[delivered] %s (estado=%s)", msg.data, self.state)

        if not msg.data:
            return

        if self.state == "DELIVERING_TO_PATIENT":
            self.set_phase("CLOSING_INTERACTION")
            self.pub_delivered.publish(Bool(data=True))
        else:
            rospy.logwarn("delivered recibido en fase inesperada (estado=%s)", self.state)


def main():
    sm = DispenseProcessStateMachine()
    rospy.loginfo("tirgo_dispense_state_machine listo, esperando eventos...")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
