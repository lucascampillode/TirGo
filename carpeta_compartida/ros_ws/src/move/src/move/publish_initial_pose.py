#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from tf.transformations import quaternion_from_euler
from math import radians


def make_msg(x, y, qz, qw):
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"

    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = 0.0

    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw

    # default small planar covariance: [x, y, yaw]
    cov = [0.21065809247261713, 0.0015867823238011614, 0.0, 0.0, 0.0, 0.0, 0.0015867823238009393, 0.2165335154259953, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06603954552177965]
    return msg

def publish_spin(duration=4.0, switch_interval=2.0, angular_speed=1.0, topic='/mobile_base_controller/cmd_vel', rate_hz=10.0):
    pub = rospy.Publisher(topic, Twist, queue_size=1)
    rospy.sleep(0.1)

    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0

    total_steps = max(1, int(duration * rate_hz))
    steps_per_switch = max(1, int(switch_interval * rate_hz))
    rate = rospy.Rate(rate_hz)

    for i in range(total_steps):
        sign = -1 if ((i // steps_per_switch) % 2) == 0 else 1  
        twist.angular.z = sign * float(angular_speed)
        pub.publish(twist)
        rate.sleep()

    # stop rotation
    twist.angular.z = 0.0
    pub.publish(twist)

def main(x=0.47278890013694763, y=-1.169088363647461, qz=0.32774684904650947, qw=0.9447655809459214):
    rospy.init_node('set_initial_pose', anonymous=False)
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
    rospy.sleep(0.1)
    rate = rospy.Rate(5)

    msg = make_msg(x, y, qz, qw)

    for i in range(5):
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()
    publish_spin(duration=8.0, switch_interval=4.0, angular_speed=1.0, topic='/mobile_base_controller/cmd_vel', rate_hz=10.0)


if __name__ == '__main__':
    try:
        main(x=1.4998722751648397, y=-0.7247556435570256, qz=0.989510915323257, qw=0.14445812007682451)
    except:
        pass