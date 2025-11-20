#!/usr/bin/env python3
import rospy
import pigpio
import time
from std_msgs.msg import Int32, Bool

# ⚠️ Usa los MISMOS PINES QUE EN TU SCRIPT DE PRUEBA
SERVO_A = 2   # Bote A/B
SERVO_B = 3   # Bote C/D

# Límites del servo
MIN_US = 500
MAX_US = 2500
NEUTRAL = 1500

# ⚡ MOVIMIENTO ULTRARRÁPIDO
DWELL_PUSH = 0.10     # Tiempo empujando
MOVE_TIME  = 0.02     # Tiempo mínimo en la posición

class ServoDispenser:
    def __init__(self):
        rospy.loginfo("Iniciando ServoDispenser RÁPIDO...")

        self.pi = pigpio.pi()
        if not self.pi.connected:
            rospy.logerr("No conectado a pigpiod. Lanza 'sudo pigpiod'.")
            raise RuntimeError("pigpio not connected")

        self.pi.set_mode(SERVO_A, pigpio.OUTPUT)
        self.pi.set_mode(SERVO_B, pigpio.OUTPUT)

        self.set_neutral()

        # 👉 NUEVO TOPIC
        self.pub_ready = rospy.Publisher("/tirgo/dispense/ready", Bool, queue_size=10)

        rospy.Subscriber("/tirgo/dispense/request", Int32,
                         self.cb_dispense, queue_size=10)

        rospy.loginfo("ServoDispenser listo (modo rápido). IDs: 1(A),2(B),3(C),4(D).")

    # --------------------------
    #  Utilidades
    # --------------------------

    @staticmethod
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def angle_to_us(self, angle_deg):
        span = (MAX_US - MIN_US) / 2.0
        us = NEUTRAL + (angle_deg / 90.0) * span
        return int(self.clamp(us, MIN_US, MAX_US))

    # ⚡ MOVIMIENTO ULTRARRÁPIDO (sin interpolación)
    def fast_move(self, gpio, us_target):
        self.pi.set_servo_pulsewidth(gpio, int(us_target))
        time.sleep(MOVE_TIME)

    def set_neutral(self):
        self.pi.set_servo_pulsewidth(SERVO_A, NEUTRAL)
        self.pi.set_servo_pulsewidth(SERVO_B, NEUTRAL)
        time.sleep(0.05)

    # --------------------------
    #  Acciones de dispensado
    # --------------------------

    def push_A(self):
        us = self.angle_to_us(+90)
        self.fast_move(SERVO_A, us)
        time.sleep(DWELL_PUSH)
        self.fast_move(SERVO_A, NEUTRAL)

    def push_B(self):
        us = self.angle_to_us(-90)
        self.fast_move(SERVO_A, us)
        time.sleep(DWELL_PUSH)
        self.fast_move(SERVO_A, NEUTRAL)

    def push_C(self):
        us = self.angle_to_us(+90)
        self.fast_move(SERVO_B, us)
        time.sleep(DWELL_PUSH)
        self.fast_move(SERVO_B, NEUTRAL)

    def push_D(self):
        us = self.angle_to_us(-90)
        self.fast_move(SERVO_B, us)
        time.sleep(DWELL_PUSH)
        self.fast_move(SERVO_B, NEUTRAL)

    # --------------------------
    #  Callback ROS
    # --------------------------

    def cb_dispense(self, msg):
        i = msg.data
        rospy.loginfo("Solicitud recibida: %d", i)

        ok = True
        if   i == 1: self.push_A()
        elif i == 2: self.push_B()
        elif i == 3: self.push_C()
        elif i == 4: self.push_D()
        else:
            rospy.logwarn("ID no válido: %d", i)
            ok = False

        # 🔔 Publicar READY si se hizo acción válida
        if ok:
            rospy.loginfo("Publicando ready=True en /tirgo/dispense/ready")
            self.pub_ready.publish(Bool(data=True))

    def shutdown(self):
        rospy.loginfo("Apagando: servos a neutro y liberados.")
        self.set_neutral()
        time.sleep(0.1)
        self.pi.set_servo_pulsewidth(SERVO_A, 0)
        self.pi.set_servo_pulsewidth(SERVO_B, 0)
        self.pi.stop()

# --------------------------
#  MAIN
# --------------------------

if __name__ == "__main__":
    rospy.init_node("servo_dispenser_node")
    sd = None
    try:
        sd = ServoDispenser()
        rospy.on_shutdown(sd.shutdown)
        rospy.spin()
    except Exception as e:
        rospy.logerr("Error al iniciar: %s", e)
        if sd is not None:
            sd.shutdown()
