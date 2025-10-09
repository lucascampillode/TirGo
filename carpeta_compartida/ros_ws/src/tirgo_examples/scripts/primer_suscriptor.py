#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32


def callback(msg: Int32) -> None:
    rospy.loginfo(f"Recibido: {msg.data}")

def listener():
    rospy.init_node('primer_suscriptor', anonymous=True)
    rospy.Subscriber('topic_entero', Int32, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()