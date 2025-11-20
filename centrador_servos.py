#!/usr/bin/env python3
import pigpio

# Pines de los servos
SERVO_GPIO2 = 2
SERVO_GPIO3 = 3
NEUTRAL = 1500  # microsegundos (~90° o posición 0)

pi = pigpio.pi()
if not pi.connected:
    print("No conectado a pigpio. Ejecuta: sudo pigpiod")
    exit(1)

# Centrar los servos
pi.set_servo_pulsewidth(SERVO_GPIO2, NEUTRAL)
pi.set_servo_pulsewidth(SERVO_GPIO3, NEUTRAL)

print("Servos centrados en posición 0.")

# Dejar la posición 0 un segundo y luego apagar la señal
import time
time.sleep(1)

pi.set_servo_pulsewidth(SERVO_GPIO2, 0)
pi.set_servo_pulsewidth(SERVO_GPIO3, 0)
pi.stop()
