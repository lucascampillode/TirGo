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

Node that uses the covariance matrix from /amcl_pose topic
to compute a confidence value on how well we are localized.

'''
import numpy as np

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64

AMCL_TOPIC = '/amcl_pose'
LOC_CONFIDENCE_TOPIC = '/amcl_loc_confidence'


class LocalizationConfidence:
    def __init__(self):
        rospy.loginfo("Subscribing to: " + AMCL_TOPIC)
        self.amcl_sub = rospy.Subscriber(AMCL_TOPIC, PoseWithCovarianceStamped, self.amcl_cb)
        rospy.loginfo("Setting publisher to: " + LOC_CONFIDENCE_TOPIC)
        self.loc_conf_pub = rospy.Publisher(LOC_CONFIDENCE_TOPIC, Float64)
        rospy.loginfo("Running LocalizationConfidence.")
        
    def amcl_cb(self, data):
        #rospy.loginfo("Got cb: " + str(data) + "\nComputing confidence.")
        self.compute_confidence(data.pose.covariance)
        
    def compute_confidence(self, covariance):
        # Reshape from array of 36 values to submatrix 3x3 with the only filled values in the covariance
        # First, second and last row with first, second and last column
        covar_3x3 = np.array(((covariance[0],  covariance[1],  covariance[5]),
                                 (covariance[6],  covariance[7],  covariance[12]),
                                 (covariance[29], covariance[30], covariance[35])))
        confidence = np.linalg.det(covar_3x3)
        rospy.loginfo("Confidence is: " + str(confidence))
        self.loc_conf_pub.publish(Float64(confidence))
    
    
if __name__ == '__main__':
    rospy.init_node("localization_confidence_node")
    w_node = LocalizationConfidence()
    rospy.spin()
    
    
# rostopic echo /amcl_pose 
# header: 
#   seq: 7
#   stamp: 
#     secs: 70
#     nsecs: 938000000
#   frame_id: map
# pose: 
#   pose: 
#     position: 
#       x: -0.0212574093508
#       y: 0.0294912095109
#       z: 0.0
#     orientation: 
#       x: 0.0
#       y: 0.0
#       z: -1.23960155565e-05
#       w: 0.999999999923
#   covariance: [0.004862490732600364, 0.0019068450494778098, 0.0, 0.0, 0.0, 0.0, 0.0019068450494778098, 0.0026295314348148747, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.00017141707547008755]
