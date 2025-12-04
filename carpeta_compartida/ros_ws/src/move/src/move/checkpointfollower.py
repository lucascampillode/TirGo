#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionGoal
import numpy as np
from tf.transformations import quaternion_inverse, quaternion_multiply
from copy import deepcopy


class Follower:
    """
        Publica goals a 'move_base' y espera a que el robot se mueva y se detenga para enviar siguiente punto

        Idea general:
            1. 'enviar_puntos()' construye objeto PoseStamped() con posición y orientación destino.
            2. 'enviar_punto()' empaqueta el objeto PoseStamped() en MoveBaseActionGoal() y lo publica en '/move_base/goal'.
            3. Se recogen datos de '/robot_pose' y se comparan con el goal para detectar inicio y fin de movimiento, con ventanas de tiempo para comprobar movimiento y umbrales de tolerancia configurables.
    """

    def __init__(self, init_node: bool = True) -> None:
        """
        init_node:
            - True  -> este objeto inicializa el nodo ROS (comportamiento original).
            - False -> asume que el nodo ROS ya está inicializado desde fuera.
        """
        # Cambio mínimo: solo inicializar el nodo si procede
        if init_node and not rospy.core.is_initialized():
            rospy.init_node("checkpoint_action_client", anonymous=True)

        self.pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)  # Publicador de goals
        self.current_pose = None
        rospy.Subscriber('/robot_pose', PoseWithCovarianceStamped, self.pose_cb, queue_size=10)  # Suscriptor de la pose actual
        rospy.wait_for_message('/robot_pose', PoseWithCovarianceStamped)  # Espera a recibir la primera pose
        rospy.sleep(0.5)  # Da tiempo a conectar
        
    def quat_angle_diff(self, q1: list, q2: list) -> float:
        """
            Calcula la diferencia angular entre dos quaternios.

            Args:
                q1: Primer quaternio.
                q2: Segundo quaternio.

            Returns:
                float: diferencia angular en radianes.
        """
        q_rel = quaternion_multiply(q2, quaternion_inverse(q1))
        # el ángulo está en la parte escalar: angle = 2 * atan2(|v|, w)
        v = np.linalg.norm(q_rel[:3])
        w = q_rel[3]
        return 2 * np.math.atan2(v, w)
    
    def pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        """
            Callback para actualizar la pose actual del robot.
        """
        self.current_pose = msg.pose
        
    def __se_ha_movido(self, pose_actual: PoseStamped, nueva_posicion: PoseStamped, umbral: float = 0.05) -> bool:
        """
            Comprueba si el robot se ha movido más allá de un umbral dado.

            Args:
                pose_actual (PoseStamped): pose actual del robot.
                nueva_posicion (PoseStamped): pose resultado con la que comparar.
                umbral (float, optional): umbral de distancia o ángulo para considerar que se ha movido. Defaults to 0.05.

            Returns:
                bool: True si se ha movido más allá del umbral, False en caso contrario.
        """
        dx = pose_actual.pose.position.x - nueva_posicion.pose.position.x
        dy = pose_actual.pose.position.y - nueva_posicion.pose.position.y
        distancia = (dx**2 + dy**2)**0.5
        dang = self.quat_angle_diff(
            [pose_actual.pose.orientation.w,
             pose_actual.pose.orientation.x,
             pose_actual.pose.orientation.y,
             pose_actual.pose.orientation.z],
            [nueva_posicion.pose.orientation.w,
             nueva_posicion.pose.orientation.x,
             nueva_posicion.pose.orientation.y,
             nueva_posicion.pose.orientation.z]
        )
        
        return distancia + dang > umbral
    
    def robot_moviendose(self, frecuencia_compr: float = 1, umbral: float = 0.05) -> bool:
        """
            Guarda las poses actual y en x tiempo, y llama a comprobar si el robot se está moviendo comparando su pose actual con la pose después de un tiempo.

            Args:
                frecuencia_compr (float, optional): Tiempo en segundos entre captura de datos. Defaults to 1.
                umbral (float, optional): Umbral de distancia o ángulo para considerar que se ha movido. Defaults to 0.05.

            Returns:
                bool: True si el robot se está moviendo, False en caso contrario.
        """
        pose_inicial = deepcopy(self.current_pose)
        rospy.sleep(frecuencia_compr)
        pose_final = deepcopy(self.current_pose)
        return self.__se_ha_movido(pose_inicial, pose_final, umbral)
    
    def esperar_a_que_se_empiece_a_mover(self, timeout: float = 10.0, umbral: float = 0.05, frecuencia_compr: float = 1) -> bool:
        """
            Espera a que el robot empiece a moverse dentro de un tiempo límite.
        """
        inicio = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - inicio < timeout:
            if self.robot_moviendose(umbral=umbral, frecuencia_compr=frecuencia_compr):
                return True
        
        return False
    
    def esperar_a_que_se_detenga(self, timeout: float = 30.0, umbral: float = 0.05, frecuencia_compr: float = 1) -> bool:
        """
            Espera a que el robot se detenga dentro de un tiempo límite.
        """
        inicio = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - inicio < timeout:
            if not self.robot_moviendose(umbral=umbral, frecuencia_compr=frecuencia_compr):
                return True
        
        return False
        
    def enviar_punto(self, punto: PoseStamped) -> bool:
        """
            Envía un punto objetivo al robot y espera a que se mueva y se detenga.

            Returns:
                bool: True si el robot se mueve y se detiene correctamente llegando a su destino, False en caso contrario.
        """
        msg = MoveBaseActionGoal()
        msg.goal.target_pose = PoseStamped()
        msg.goal.target_pose.header.frame_id = "map"
        msg.goal.target_pose.header.stamp = rospy.Time.now()
        msg.goal.target_pose.pose = punto.pose

        rospy.loginfo("[FOLLOWER] Publicando nuevo goal a /move_base/goal")
        self.pub.publish(msg)

        # -------------------------
        # Nueva lógica de espera:
        #   - Espera hasta ver al robot MOVERSE al menos una vez.
        #   - Luego espera a que DEJE de moverse.
        #   - Todo dentro de un timeout total.
        # -------------------------
        timeout_total = 180.0  # tiempo máx total para toda la secuencia (s)
        frecuencia = 1.0       # segundos entre comprobaciones
        umbral = 0.05          # mismo umbral que ya usas

        inicio = rospy.Time.now().to_sec()
        ha_movidose = False

        rospy.loginfo("[FOLLOWER] Esperando secuencia movimiento -> parada...")

        while (rospy.Time.now().to_sec() - inicio) < timeout_total and not rospy.is_shutdown():
            # Esta función ya duerme 'frecuencia' segundos internamente
            moviendose = self.robot_moviendose(frecuencia_compr=frecuencia, umbral=umbral)

            if moviendose:
                if not ha_movidose:
                    rospy.loginfo("[FOLLOWER] Detectado movimiento del robot.")
                ha_movidose = True
            else:
                # Si ya se había movido y ahora deja de moverse: objetivo alcanzado
                if ha_movidose:
                    rospy.loginfo("[FOLLOWER] Detectada parada tras movimiento. Objetivo alcanzado.")
                    return True
                # Si todavía no se ha movido, seguimos esperando

        rospy.logwarn("[FOLLOWER] No se ha detectado secuencia movimiento->parada antes del timeout.")
        return False
        """
            Envía un punto objetivo al robot y espera a que se mueva y se detenga.

            Returns:
                bool: True si el robot se mueve y se detiene correctamente llegando a su destino, False en caso contrario.
        """
        msg = MoveBaseActionGoal()
        msg.goal.target_pose = PoseStamped()
        msg.goal.target_pose.header.frame_id = "map"
        msg.goal.target_pose.header.stamp = rospy.Time.now()
        msg.goal.target_pose.pose = punto.pose
        
        self.pub.publish(msg)
        if not self.esperar_a_que_se_empiece_a_mover(timeout=15.0, frecuencia_compr=1):
            return False
        if not self.esperar_a_que_se_detenga(timeout=120.0, frecuencia_compr=1):
            return False
        
        return True

    def enviar_puntos(self, puntos: list[list]) -> bool:
        """
            Envía una lista de puntos objetivo al robot.

            Returns:
                bool: True si el robot llega a todos los puntos correctamente, False en caso contrario.
        """
        all_ok = True
        for punto in puntos:
            x, y, oz, ow = punto

            target_pose = PoseStamped()
            target_pose.header.frame_id = "map"
            target_pose.header.stamp = rospy.Time.now()
            target_pose.pose.position.x = x
            target_pose.pose.position.y = y
            target_pose.pose.position.z = 0.0
            target_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=oz, w=ow)

            ok = self.enviar_punto(target_pose)
            if not ok:
                all_ok = False
                break

        return all_ok


if __name__ == "__main__":
    #checkpoints = [
    #    [1.801448077821011,  -0.7272143308845652,  -0.17600744219629474, 0.9843888359238527], #puerta
    #    [0.75413808767915,    0.6935195599805307,   0.8738231258256374,  0.48624391489489344], #puerta2
    #    [3.6704948592312454, -1.8971807817955268,  -0.34134149988312906, 0.9399393493505502], #AlMedio
    #    [6.128891746903331,  -2.156612477116119,  -0.100474241670117,   0.9949396598592374], #Alfondo
    #    [1.4998722751648397, -0.7247556435570256, 0.989510915323257, 0.14445812007682451] #Punto referencia
    #]



    ### 1.600 -0.8636  0.9939  0.109555
    checkpoints = [
        [1.4998722751648397, -0.7247556435570256, 0.989510915323257, 0.14445812007682451], # Pasillo
    ]
    ### 0.27089 2.3590 0.22086 0.97539
    
    follower = Follower()  # comportamiento original: init_node=True
    for i in range(5):
        follower.enviar_puntos(checkpoints)
