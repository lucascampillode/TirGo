#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
# Importamos la clase Follower del archivo checkpoint_follower.py
# Asegúrate de que el archivo se llame exactamente así o cambia el import
from checkpointfollower import Follower

def ejecutar_ruta_robot(lista_de_puntos: list[list]) -> None:
    """
    Función externa que instancia el seguidor y le manda la lista de objetivos.
    """
    try:
        # 1. Instanciamos la clase. 
        # NOTA: Al hacer esto, la clase Follower ejecutará rospy.init_node internamente
        # tal como está definida en tu código original.
        rospy.loginfo("Inicializando el seguidor de checkpoints...")
        navegador = Follower()

        rospy.loginfo(f"Recibida una ruta con {len(lista_de_puntos)} puntos. Iniciando...")

        # 2. Llamamos a la función de la clase para enviar los puntos
        exito = navegador.enviar_puntos(lista_de_puntos)

        # 3. Comprobamos el resultado
        if exito:
            rospy.loginfo("Misión completada: El robot ha recorrido todos los puntos.")
        else:
            rospy.logwarn("Misión finalizada con incidencias: El robot no pudo completar algún punto.")

    except rospy.ROSInterruptException:
        rospy.logerr("La misión fue interrumpida por el usuario o el sistema.")

# --- Definición de la Misión ---

if __name__ == "__main__":
    # Definimos los puntos (x, y, orient_z, orient_w)º
    # Puedes añadir tantos como quieras aquí
    ruta_farmacia = [
        # Punto A: Salida
        [1.499, -0.724, 0.989, 0.144],
        # Punto B: Pasillo
        [1.485, -1.819, -0.779, 0.626],
        # Punto C: Destino final (ejemplo)
        [3.670, -1.897, -0.341, 0.939] 
    ]

    # Llamamos a la función externa pasándole los datos
    ejecutar_ruta_robot(ruta_farmacia)