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

# ==============================
#  LÓGICA PURA DE MAPEADO
# ==============================

def compute_servo_command(bin_id):
    """
    Lógica pura de mapeo bin_id → (gpio, angle_deg).

    Devuelve:
      (gpio, angle_deg) si el bin_id es válido
      None si el bin_id es inválido.
    """
    try:
        i = int(bin_id)
    except (TypeError, ValueError):
        return None

    if   i == 1:
        return SERVO_A, +90.0
    elif i == 2:
        return SERVO_A, -90.0
    elif i == 3:
        return SERVO_B, +90.0
    elif i == 4:
        return SERVO_B, -90.0

    return None


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

        # 👉 TOPIC READY
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
    #  Acción genérica de dispensado
    # --------------------------

    def do_push_for_bin(self, bin_id):
        """
        Usa compute_servo_command para decidir qué servo mover y cuánto.

        Devuelve:
          True  -> si el bin_id es válido y se ha ejecutado movimiento
          False -> si el bin_id es inválido (no se mueve nada)
        """
        cmd = compute_servo_command(bin_id)
        if cmd is None:
            return False

        gpio, angle = cmd
        us = self.angle_to_us(angle)

        self.fast_move(gpio, us)
        time.sleep(DWELL_PUSH)
        self.fast_move(gpio, NEUTRAL)
        return True

    # --------------------------
    #  Callback ROS
    # --------------------------

    def cb_dispense(self, msg):
        i = msg.data
        rospy.loginfo("Solicitud recibida: %d", i)

        ok = self.do_push_for_bin(i)

        if not ok:
            rospy.logwarn("ID no válido: %d", i)
            return

        # 🔔 Publicar READY si se hizo acción válida
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
