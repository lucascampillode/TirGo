#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from std_msgs.msg import Bool
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SecuenciaBrazoTiago(object):
    def __init__(self):
        rospy.init_node("tiago_secuencia_brazo", anonymous=False)

        # Joints
        self.arm_joint_names = [
            "arm_1_joint", "arm_2_joint", "arm_3_joint",
            "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"
        ]
        self.torso_joint_names = ["torso_lift_joint"]
        self.head_joint_names = ["head_1_joint", "head_2_joint"]

        # GRIPPER
        self.gripper_joint_names = [
            "gripper_left_finger_joint",
            "gripper_right_finger_joint"
        ]
        self.gripper_pub = rospy.Publisher(
            "/gripper_controller/command", JointTrajectory, queue_size=1
        )

        # PUBLICADORES de estado alto nivel
        self.picked_pub = rospy.Publisher(
            "/tirgo/tiago/picked", Bool, queue_size=1
        )
        self.delivered_pub = rospy.Publisher(
            "/tirgo/tiago/delivered", Bool, queue_size=1
        )

        # Cliente de acción SOLO para el brazo
        rospy.loginfo("Esperando a servidor de trayectoria del brazo...")
        self.arm_client = actionlib.SimpleActionClient(
            "/arm_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )
        self.arm_client.wait_for_server()
        rospy.loginfo("Conectado al servidor de trayectoria del brazo.")

        # Publicadores a tópico para torso y cabeza
        self.torso_pub = rospy.Publisher(
            "/torso_controller/command", JointTrajectory, queue_size=1
        )
        self.head_pub = rospy.Publisher(
            "/head_controller/command", JointTrajectory, queue_size=1
        )

        # Definimos la secuencia de poses (valen para coger y dejar)
        self.secuencia = self._definir_secuencia()

        # Flags de eventos (coger / dejar)
        self.ready_to_pick = False      # /tirgo/dispense/ready
        self.at_patient = False         # /tirgo/tiago/at_patient

        # Suscriptores a los triggers lógicos
        rospy.Subscriber("/tirgo/dispense/ready", Bool, self._ready_cb)
        rospy.Subscriber("/tirgo/tiago/at_patient", Bool, self._at_patient_cb)

    # -------------------  CALLBACKS DE EVENTOS -------------------

    def _ready_cb(self, msg):
        if msg.data:
            rospy.loginfo("Trigger recibido: /tirgo/dispense/ready = true")
            self.ready_to_pick = True
        else:
            rospy.loginfo("Recibido /tirgo/dispense/ready = false (ignorado).")

    def _at_patient_cb(self, msg):
        if msg.data:
            rospy.loginfo("Trigger recibido: /tirgo/tiago/at_patient = true")
            self.at_patient = True
        else:
            rospy.loginfo("Recibido /tirgo/tiago/at_patient = false (ignorado).")

    # -------------------  DEFINICION SECUENCIA -------------------

    def _definir_secuencia(self):
        dur = 5.0

        POS_HOME = {
            "nombre": "POSICION DE HOME",
            "torso": 0.15,
            "arm": [0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0],
            "head": [0.05, -0.02],
            "duration": dur
        }

        SUBIR_TORSO_PREPARAR = {
            "nombre": "SUBIR TORSO Y PREPARAR BRAZO",
            "torso": 0.35,
            "arm": [0.20, 0.33, -0.20, 1.94, -1.57, 1.37, 0.0],
            "head": [-0.39, -0.02],
            "duration": dur
        }

        AJUSTE_EVITAR_MESA = {
            "nombre": "AJUSTE EVITAR MESA",
            "torso": 0.35,
            "arm": [0.20, 1.02, -0.20, 1.94, -1.57, -1.39, 0.0],
            "head": [-0.01, -0.02],
            "duration": dur
        }

        PRIMERA_APROX = {
            "nombre": "PRIMERA APROXIMACION",
            "torso": 0.35,
            "arm": [1.37, 0.98, -0.20, 2.29, -1.64, -1.27, -0.05],
            "head": [-0.39, -0.02],
            "duration": dur
        }

        SEGUNDA_APROX = {
            "nombre": "SEGUNDA APROXIMACION",
            "torso": 0.35,
            "arm": [1.37, 0.42, -0.20, 1.91, -1.59, -1.39, -0.11],
            "head": [0.00, -0.02],
            "duration": dur
        }

        POS_COGIDA = {
            "nombre": "POSICION COGIDA",
            "torso": 0.35,
            "arm": [1.37, 0.17, -0.19, 1.22, -1.58, -0.95, -0.11],
            "head": [-0.01, -0.02],
            "duration": dur
        }

        secuencia = [
            POS_HOME,
            SUBIR_TORSO_PREPARAR,
            AJUSTE_EVITAR_MESA,
            PRIMERA_APROX,
            SEGUNDA_APROX,
            POS_COGIDA,
            SEGUNDA_APROX,
            PRIMERA_APROX,
            AJUSTE_EVITAR_MESA,
            SUBIR_TORSO_PREPARAR,
            POS_HOME
        ]

        return secuencia

    # -------------------  PUBLICADORES DE JOINTS -------------------

    def _publicar_torso(self, torso_pos, duration):
        traj = JointTrajectory()
        traj.joint_names = self.torso_joint_names

        p = JointTrajectoryPoint()
        p.positions = [torso_pos]
        p.time_from_start = rospy.Duration(duration)

        traj.points = [p]
        self.torso_pub.publish(traj)

    def _publicar_head(self, head_pos, duration):
        traj = JointTrajectory()
        traj.joint_names = self.head_joint_names

        p = JointTrajectoryPoint()
        p.positions = head_pos
        p.time_from_start = rospy.Duration(duration)

        traj.points = [p]
        self.head_pub.publish(traj)

    def _publicar_gripper(self, posiciones, duration=2.0):
        traj = JointTrajectory()
        traj.joint_names = self.gripper_joint_names

        p = JointTrajectoryPoint()
        p.positions = posiciones
        p.time_from_start = rospy.Duration(duration)

        traj.points = [p]
        self.gripper_pub.publish(p)

    def _abrir_gripper(self):
        rospy.loginfo("Abriendo gripper...")
        self._publicar_gripper([0.045, 0.045], duration=1.0)
        rospy.sleep(2.0)

    def _cerrar_gripper(self):
        rospy.loginfo("Cerrando gripper...")
        self._publicar_gripper([0.0, 0.0], duration=1.0)
        rospy.sleep(2.0)

    # -------------------  BRAZO (ACTION SERVER) -------------------

    def _enviar_brazo(self, arm_pos, duration):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.arm_joint_names

        p = JointTrajectoryPoint()
        p.positions = arm_pos
        p.time_from_start = rospy.Duration(duration)

        goal.trajectory.points = [p]
        self.arm_client.send_goal(goal)

    # -------------------  EJECUCION DE UN PASO -------------------

    def _ejecutar_paso(self, paso):
        nombre = paso["nombre"]
        torso_pos = paso["torso"]
        arm_pos = paso["arm"]
        head_pos = paso["head"]
        duration = paso["duration"]

        rospy.loginfo("Ejecutando paso: %s", nombre)

        self._publicar_torso(torso_pos, duration)
        self._publicar_head(head_pos, duration)
        self._enviar_brazo(arm_pos, duration)

        timeout = rospy.Duration(duration + 3.0)
        self.arm_client.wait_for_result(timeout)

        rospy.loginfo("Paso '%s' completado.", nombre)

    # -------------------  SECUENCIA DE COGIDA -------------------

    def _ejecutar_secuencia_coger(self):
        rospy.loginfo("Iniciando SECUENCIA DE COGIDA...")
        rospy.sleep(0.5)

        for paso in self.secuencia:
            if rospy.is_shutdown():
                return

            if paso["nombre"] == "POSICION COGIDA":
                self._abrir_gripper()

            self._ejecutar_paso(paso)
            rospy.sleep(0.5)

            if paso["nombre"] == "POSICION COGIDA":
                self._cerrar_gripper()

        rospy.loginfo("Secuencia de COGIDA finalizada. Publicando /tirgo/tiago/picked = true")
        self.picked_pub.publish(Bool(data=True))
        rospy.sleep(0.2)

    # -------------------  SECUENCIA DE DEJADA -------------------

    def _ejecutar_secuencia_dejar(self):
        rospy.loginfo("Iniciando SECUENCIA DE DEJADA...")
        rospy.sleep(0.5)

        pasada_pos_cogida = False

        for paso in self.secuencia:
            if rospy.is_shutdown():
                return

            nombre = paso["nombre"]

            if nombre == "POSICION COGIDA":
                self._ejecutar_paso(paso)
                rospy.sleep(0.5)
                self._abrir_gripper()
                pasada_pos_cogida = True
                continue

            if nombre == "SEGUNDA_APROX" and pasada_pos_cogida:
                self._ejecutar_paso(paso)
                rospy.sleep(0.5)
                self._cerrar_gripper()
                continue

            self._ejecutar_paso(paso)
            rospy.sleep(0.5)

        rospy.loginfo("Secuencia de DEJADA finalizada. Publicando /tirgo/tiago/delivered = true")
        self.delivered_pub.publish(Bool(data=True))
        rospy.sleep(0.2)

    # -------------------  BUCLE PRINCIPAL -------------------

    def run(self):
        rospy.loginfo("Nodo listo. Esperando eventos:")
        rospy.loginfo(" - /tirgo/dispense/ready  (coger)")
        rospy.loginfo(" - /tirgo/tiago/at_patient (dejar)")

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.ready_to_pick:
                self.ready_to_pick = False
                self._ejecutar_secuencia_coger()
                rospy.loginfo("Fin secuencia coger. Esperando nuevos eventos...")

            elif self.at_patient:
                self.at_patient = False
                self._ejecutar_secuencia_dejar()
                rospy.loginfo("Fin secuencia dejar. Esperando nuevos eventos...")

            rate.sleep()


if __name__ == "__main__":
    try:
        node = SecuenciaBrazoTiago()
        node.run()
    except rospy.ROSInterruptException:
        pass
