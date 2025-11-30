#!/usr/bin/env python3
"""
test_tirgo_mission.py

Script de prueba automática para el Action Server /tirgo/mission.

Funcionalidades:
- Crea un SimpleActionClient a /tirgo/mission (TirgoMissionAction).
- Envía un goal con patient_id y med_id (por parámetros).
- Opcionalmente SIMULA:
    - /tirgo/tiago/arrived = True tras un delay.
    - /tirgo/dispense/ready = True tras un delay.
- Espera al resultado del Action y lo imprime por consola.

Uso típico:
    rosrun tirgo_mission_server test_tirgo_mission.py \
        _patient_id:="test123" _med_id:=3 \
        _simulate_arrived:=true _simulate_ready:=true

Parámetros (rosparam ~):
    ~patient_id         (string, por defecto "test123")
    ~med_id             (int, por defecto 1)
    ~simulate_arrived   (bool, por defecto true)
    ~simulate_ready     (bool, por defecto true)
    ~arrive_delay       (float, seg, por defecto 5.0)
    ~ready_delay        (float, seg, por defecto 10.0)
"""

import rospy
import actionlib
from std_msgs.msg import Bool
from tirgo_msgs.msg import TirgoMissionAction, TirgoMissionGoal


def main():
    rospy.init_node("tirgo_mission_test", anonymous=True)

    # Leer parámetros
    patient_id = rospy.get_param("~patient_id", "test123")
    med_id = int(rospy.get_param("~med_id", 1))

    simulate_arrived = bool(rospy.get_param("~simulate_arrived", True))
    simulate_ready = bool(rospy.get_param("~simulate_ready", True))

    arrive_delay = float(rospy.get_param("~arrive_delay", 5.0))
    ready_delay = float(rospy.get_param("~ready_delay", 10.0))

    rospy.loginfo("=== Tirgo Mission Test ===")
    rospy.loginfo(f"patient_id = {patient_id}")
    rospy.loginfo(f"med_id     = {med_id}")
    rospy.loginfo(f"simulate_arrived = {simulate_arrived} (delay={arrive_delay}s)")
    rospy.loginfo(f"simulate_ready   = {simulate_ready} (delay={ready_delay}s)")

    # Publishers para simular los flags (si se usan)
    pub_arrived = rospy.Publisher("/tirgo/tiago/arrived", Bool, queue_size=1)
    pub_ready = rospy.Publisher("/tirgo/dispense/ready", Bool, queue_size=1)

    # Cliente de Action
    client = actionlib.SimpleActionClient("/tirgo/mission", TirgoMissionAction)
    rospy.loginfo("[TEST] Esperando al ActionServer /tirgo/mission...")
    if not client.wait_for_server(rospy.Duration(5.0)):
        rospy.logerr("[TEST] No se encontró /tirgo/mission. ¿Está lanzado tirgo_mission_server?")
        return

    rospy.loginfo("[TEST] ActionServer /tirgo/mission disponible, enviando goal...")

    goal = TirgoMissionGoal()
    goal.patient_id = patient_id
    goal.med_id = med_id

    # Enviar goal
    client.send_goal(goal)
    rospy.loginfo("[TEST] Goal enviado.")

    # Planificar simulación de arrived / ready con timers
    # Solo si simulate_* == True
    if simulate_arrived:
        def _timer_arrived_cb(_evt):
            rospy.loginfo("[TEST] Simulando /tirgo/tiago/arrived = True")
            pub_arrived.publish(Bool(data=True))
        rospy.Timer(rospy.Duration(arrive_delay), _timer_arrived_cb, oneshot=True)

    if simulate_ready:
        def _timer_ready_cb(_evt):
            rospy.loginfo("[TEST] Simulando /tirgo/dispense/ready = True")
            pub_ready.publish(Bool(data=True))
        rospy.Timer(rospy.Duration(ready_delay), _timer_ready_cb, oneshot=True)

    # Esperar resultado
    rospy.loginfo("[TEST] Esperando resultado del Action...")
    client.wait_for_result()
    result = client.get_result()
    state = client.get_state()

    rospy.loginfo("=== RESULTADO DE LA MISIÓN ===")
    rospy.loginfo(f"Action state (num): {state}")
    if result is not None:
        rospy.loginfo(f"success      = {result.success}")
        rospy.loginfo(f"error_code   = {result.error_code}")
        rospy.loginfo(f"error_message= {result.error_message}")
    else:
        rospy.logwarn("Result == None (algo ha ido raro)")

    rospy.loginfo("=== FIN DEL TEST ===")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
