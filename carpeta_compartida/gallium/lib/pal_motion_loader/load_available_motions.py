#!/usr/bin/env python
import os
import yaml
import rospy
from sensor_msgs.msg import JointState


def is_motion_file(basepath, motion):
    return os.path.isfile(
        os.path.join(basepath, motion)) and motion.endswith('.yaml')


def main():
    rospy.init_node('load_motions_node')
    basepath = os.path.expanduser('~/.pal/motions')
    js = rospy.wait_for_message('/joint_states', JointState)
    current_joints = js.name
    if os.path.isdir(basepath):
        for motion in os.listdir(basepath):
            if is_motion_file(basepath, motion):
                with open(os.path.join(basepath, motion), 'r') as f:
                    try:
                        motion_param = yaml.load(f, Loader=yaml.FullLoader)
                    except:
                        motion_param = yaml.load(f)
                    for m in motion_param['play_motion']['motions']:
                        m_param = motion_param['play_motion']['motions'][m]
                        motion_ready = True
                        for joint in m_param['joints']:
                            if joint not in current_joints:
                                motion_ready = False
                                break
                        if motion_ready:
                            ns = "/play_motion/motions/" + m
                            rospy.set_param(ns, m_param)


if __name__ == "__main__":
    main()
