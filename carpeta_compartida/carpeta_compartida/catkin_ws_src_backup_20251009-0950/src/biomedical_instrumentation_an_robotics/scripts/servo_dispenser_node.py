#!/usr/bin/env python3
import rospy
import pigpio
import time
from std_msgs.msg import UInt8

# Pines BCM
SERVO_GPIO2 = 2   # Bote A/B
SERVO_GPIO3 = 3   # Bote C/D

# Calibración
MIN_US = 500
MAX_US = 2500
NEUTRAL = 1500

DWELL_PUSH = 0.30
MOVE_TIME  = 0.20

class ServoDispenser:
    def __init__(self):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            rospy.logerr("No conectado a pigpiod. Lanza 'sudo pigpiod' antes.")
            raise RuntimeError("pigpio not connected")

        self.set_neutral()

        rospy.Subscriber(
            "/dispense_id",
            UInt8,
            self.dispense_callback,
            queue_size=10
        )

        rospy.loginfo("ServoDispenser listo. IDs válidos 1-4 en /dispense_id")

    def clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def angle_to_us(self, angle_deg):
        span = (MAX_US - MIN_US) / 2.0
        us = NEUTRAL + (angle_deg / 90.0) * span
        return int(self.clamp(us, MIN_US, MAX_US))

    def smooth_move(self, gpio, us_target, duration=MOVE_TIME, steps=20):
        us0 = self.pi.get_servo_pulsewidth(gpio)
        if us0 < 500 or us0 > 2500:
            us0 = NEUTRAL
            self.pi.set_servo_pulsewidth(gpio, us0)
        du = float(us_target - us0) / steps
        dt = float(duration) / steps
        for _ in range(steps):
            us0 += du
            self.pi.set_servo_pulsewidth(gpio, int(us0))
            time.sleep(dt)

    def set_neutral(self):
        self.pi.set_servo_pulsewidth(SERVO_GPIO2, NEUTRAL)
        self.pi.set_servo_pulsewidth(SERVO_GPIO3, NEUTRAL)

    def push_A(self):
        rospy.loginfo("Dispensando bote A")
        self.smooth_move(SERVO_GPIO2, self.angle_to_us(+90))
        time.sleep(DWELL_PUSH)
        self.smooth_move(SERVO_GPIO2, NEUTRAL)

    def push_B(self):
        rospy.loginfo("Dispensando bote B")
        self.smooth_move(SERVO_GPIO2, self.angle_to_us(-90))
        time.sleep(DWELL_PUSH)
        self.smooth_move(SERVO_GPIO2, NEUTRAL)

    def push_C(self):
        rospy.loginfo("Dispensando bote C")
        self.smooth_move(SERVO_GPIO3, self.angle_to_us(+90))
        time.sleep(DWELL_PUSH)
        self.smooth_move(SERVO_GPIO3, NEUTRAL)

    def push_D(self):
        rospy.loginfo("Dispensando bote D")
        self.smooth_move(SERVO_GPIO3, self.angle_to_us(-90))
        time.sleep(DWELL_PUSH)
        self.smooth_move(SERVO_GPIO3, NEUTRAL)

    def dispense_callback(self, msg):
        i = msg.data
        rospy.loginfo(f"ID recibido: {i}")
        if i == 1:   self.push_A()
        elif i == 2: self.push_B()
        elif i == 3: self.push_C()
        elif i == 4: self.push_D()
        else:
            rospy.logwarn(f"ID {i} inválido. Solo 1-4.")

    def shutdown(self):
        rospy.loginfo("Apagando ServoDispenser...")
        self.set_neutral()
        time.sleep(0.3)
        self.pi.set_servo_pulsewidth(SERVO_GPIO2, 0)
        self.pi.set_servo_pulsewidth(SERVO_GPIO3, 0)
        self.pi.stop()

if __name__ == "__main__":
    rospy.init_node("servo_dispenser_node")
    sd = None
    try:
        sd = ServoDispenser()
        rospy.on_shutdown(sd.shutdown)
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"Error al iniciar ServoDispenser: {e}")
        if sd is not None:
            sd.shutdown()
