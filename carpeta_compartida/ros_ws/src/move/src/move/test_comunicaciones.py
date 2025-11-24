#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from checkpointfollower import Follower  # <--- importas la clase

def main():
    # OJO: ahora mismo Follower ya hace init_node en su __init__
    follower = Follower()

    # Lista de checkpoints que quieras usar
    checkpoints = [
        [0.75413808767915,    0.6935195599805307,   0.8738231258256374,  0.48624391489489344],
        [1.4998722751648397, -0.7247556435570256, 0.989510915323257, 0.14445812007682451],
    ]

    follower.enviar_puntos(checkpoints)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
