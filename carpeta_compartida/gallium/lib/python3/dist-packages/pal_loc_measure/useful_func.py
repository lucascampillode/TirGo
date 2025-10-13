#!/usr/bin/env python
# -*- coding: utf-8 -*-

from geometry_msgs.msg import Pose2D
import tf
from tf.transformations import euler_from_quaternion
import math


def compute_inc_values (currentPos, prevPos, currentTheta):
    
    inc = Pose2D (0.0, 0.0, 0.0)
    inc.x = currentPos.x - prevPos.x
    inc.y = currentPos.y - prevPos.y
    inc.theta = currentTheta - prevPos.theta

    return inc


def getTheta (orientation):
    q = [orientation.x, orientation.y, orientation.z, orientation.w]

    roll, pitch, yaw = tf.transformations.euler_from_quaternion(q)

    return yaw
    
def norm_angle (angle):
    
    while angle >  math.pi:
        angle = angle - 2*math.pi
    while angle < - math.pi:
        angle= angle + 2*math.pi
    
    return angle
