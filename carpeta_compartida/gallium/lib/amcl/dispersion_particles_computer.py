#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2015 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND ISC DISCLAIMS ALL WARRANTIES WITH
# REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL ISC BE LIABLE FOR ANY
# SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
# OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Authors:
# * Sammy Pfeiffer
# * Enrique Fernandez

'''
@author: Sammy Pfeiffer

Calculates the dispersion of the particles from /particlecloud

'''
from __future__ import division

import rospy
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'
OKBLUE = '\033[94m'


PARTICLE_TOPIC = '/particlecloud'
CENTROID_TOPIC = '/particlecloud_centroid'


class DispersionComputer:
    def __init__(self):
        rospy.loginfo("Setting subscriber to: " + PARTICLE_TOPIC)
        self.particle_subs = rospy.Subscriber(PARTICLE_TOPIC, PoseArray, self.particle_cb)
        rospy.loginfo("Setting publisher to: " + CENTROID_TOPIC )
        self.particle_centroid_pub = rospy.Publisher(CENTROID_TOPIC, PoseStamped)
        rospy.loginfo("Running DispersionComputer.")
        
    def particle_cb(self, data):
        #rospy.loginfo("Got cb: " + str(data)")
# [geometry_msgs/PoseArray]:
# std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id
# geometry_msgs/Pose[] poses
#   geometry_msgs/Point position
#     float64 x
#     float64 y
#     float64 z
#   geometry_msgs/Quaternion orientation
#     float64 x
#     float64 y
#     float64 z
#     float64 w
        center_pose = self.compute_center(data.poses)
        ps = PoseStamped()
        ps.pose = center_pose
        ps.header.frame_id = '/map'
        self.particle_centroid_pub.publish(ps)
        rospy.loginfo("Published centroid at: " + str(ps))
        
        # Given this center, should i calculate the dispersion
        # like the sum of all the distances (x,y)
        # in reference to the center pose?
        # maybe a refrence could be the diameter of the robot? 
        # if the dispersion is bigger than the diameter of the robot
        # we are probably lost
        dispersion_val = self.compute_dispersion(center_pose, data.poses)
        rospy.loginfo("Dispersion: " + str(dispersion_val))
        
        
    def compute_center(self, pose_array):
        center = Pose()
        x_coords = []
        y_coords = []
        yaws = []
        for pose in pose_array:
            x_coords.append(pose.position.x)
            y_coords.append(pose.position.y)
            q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            roll, pitch, yaw = euler_from_quaternion(q)
            yaws.append(yaw)
        center.position.x = sum(x_coords) / len(x_coords)
        center.position.y = sum(y_coords) / len(y_coords)
        avg_yaws = sum(yaws) / len(yaws)
        center.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, avg_yaws))
        
        return center
        
    def compute_dispersion(self, center_pose, pose_array):
        x_dists = []
        y_dists = []
        for pose in pose_array:
            x_dists.append(abs(pose.position.x - center_pose.position.x))
            y_dists.append(abs(pose.position.y - center_pose.position.y))
        avg_x_dists = sum(x_dists) / len(x_dists)
        avg_y_dists = sum(y_dists) / len(y_dists)
        total_avg = (avg_x_dists + avg_y_dists ) /  2.0
        return total_avg

    
if __name__ == '__main__':
    rospy.init_node("dispersion_computer_node")
    w_node = DispersionComputer()
    rospy.spin()
