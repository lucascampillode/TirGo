from control_tiago import ControladorTiago
from math import pi
from time import sleep
from copy import deepcopy


# Clase base
controlador = ControladorTiago()

# Hablar (no funciona en simulación)
controlador.talk("Hello World", 2)

# Conseguir articulaciones actuales radianes
current_joints = controlador.get_current_joints()
current_joints[1] -= pi/8

# Mover a articulaciones (ángulos de los motores en radianes)
controlador.move_joints([0.2, pi/2, 0, -pi/2, 0, pi/2, 0, 0])

# Cambiar velocidad y aceleración de 0.0 a 1.0, pero no se nota mucho, mejor entre 0.0 y 0.1
controlador.set_velocity_and_acceleration_arm(.05, .05)

# Mover pinza
controlador.move_gripper(0.03)

# # Conseguir pose actual posición en metros y orientación adimensional (cuaternio)
# current_pose = controlador.get_current_pose()

# # Movimientos de la punta del robot en línea recta
# lista_movimientos = [] # crear la lista de movimientos
# current_pose.position.z -= 0.05 # bajar un poco en z
# lista_movimientos.append(deepcopy(current_pose)) # añadir la pose a la lista
# current_pose.position.z += 0.05 # subir un poco en z
# lista_movimientos.append(deepcopy(current_pose)) # añadir la pose a la lista
# current_pose.position.x += 0.05 # movimiento combinado, bajar y aumentar en x (haría una línea recta diagonal)
# current_pose.position.z -= 0.05
# lista_movimientos.append(deepcopy(current_pose)) # añadir la pose a la lista
# controlador.move_in_straight_line(lista_movimientos)


# Configuraciones robot
# configuracion[0] torso levantar-bajar en metros
# configuracion[1] Movimiento lateral hombro
# configuracion[2] Movimiento "hacia arriba/abajo" hombro
# configuracion[3] Movimiento rotación codo (los humanos no tenemos)
# configuracion[4] Movimiento flexión codo
# configuracion[5] Movimiento rotación codo
# configuracion[6] Movimiento flexión muñera
# configuracion[7] Movimiento rotación muñeca

# Demo de mover el codo en bucle infinito
config_inicial = controlador.get_current_joints()
config_codo_retraido = deepcopy(config_inicial)
config_codo_retraido[4] = pi/2

while True:
    controlador.move_joints(config_inicial)
    sleep(.5)
    controlador.move_joints(config_codo_retraido)

