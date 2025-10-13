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


import rospy
import rosparam
from std_msgs.msg import Float64MultiArray
from nav_by_odom.srv import NavigationTurn, NavigationTurnResponse, NavigationTurnRequest
from std_srvs.srv import Empty, EmptyRequest
import numpy as np

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

TURN_BY_ODOM_SRV= '/turn'
WEIGHTS_TOPIC = '/particle_laser_match_weight'
CLEAR_COSTMAP_SRV = '/move_base/clear_costmaps'
NO_MOTION_UPDATE_SRV = '/request_nomotion_update'
MIN_PARTICLES_PARAM = '/amcl/min_particles'

LOC_WAIT_TIME = 0.1
WELL_LOC_MAX_AVG_WEIGHT = 1.0 - 0.18

class AreWeWellLoc:
    def __init__(self):
        rospy.sleep(0.3)
        rospy.loginfo("Subscribing to: " + WEIGHTS_TOPIC)
        self.last_weights = []
        self.store_weights = False
        self.new_weights = False
        self.max_weights= []
        self.weights_sub = rospy.Subscriber(WEIGHTS_TOPIC, Float64MultiArray, self.weights_array_cb)
        rospy.loginfo("Connecting to service: " + TURN_BY_ODOM_SRV)
        self.turn_srv = rospy.ServiceProxy(TURN_BY_ODOM_SRV, NavigationTurn)
        self.turn_srv.wait_for_service()
        rospy.loginfo("Connecting to service: " + CLEAR_COSTMAP_SRV)
        self.clear_costmap_srv = rospy.ServiceProxy(CLEAR_COSTMAP_SRV, Empty)
        self.clear_costmap_srv.wait_for_service()
        rospy.loginfo("Connecting to service: " + NO_MOTION_UPDATE_SRV)
        self.req_no_mot_up = rospy.ServiceProxy(NO_MOTION_UPDATE_SRV, Empty)
        self.req_no_mot_up.wait_for_service()
        rospy.loginfo("Getting param " + MIN_PARTICLES_PARAM)
        self.min_particles = int(rosparam.get_param(MIN_PARTICLES_PARAM))
        rospy.loginfo("Got " + str(self.min_particles))
        rospy.loginfo("AreWeWellLoc init done.")
        
    def weights_array_cb(self, data):
        self.last_weights = data.data
        self.new_weights = True
        maxval = max(data.data)
        if self.store_weights:
            self.max_weights.append(maxval)
        if maxval >= 0.7:
            print OKGREEN + str(maxval) + ENDC
        else:
            print FAIL + str(maxval) + ENDC

    def compute_localization_confidence(self):
        # First let's converge the particles
        rospy.loginfo("Converging particles with request_no_motion_update calls")
        self.request_no_motion_update_until_minimum_particles()
        rospy.loginfo("Done, now we rotate 360 deg and store the best weights")
        # Start storing the best values of localization best particle weight
        self.max_weights = []
        self.store_weights = True
        # Rotate half a turn
        self.rotate_robot(180)
        # Rotate the other half
        self.rotate_robot(180)
        self.store_weights = False
        avg_weight = np.average(self.max_weights)
        rospy.loginfo("\n\nAverage weight of the 360 turn: " + str(avg_weight) + "\n\n")
        
        if avg_weight > WELL_LOC_MAX_AVG_WEIGHT:
            print OKGREEN + "WELL LOCALIZED" + ENDC
        else:
            print FAIL + "WRONG LOCALIZED" + ENDC
        
    def wait_for_new_weights(self):
        while not self.new_weights:
            #rospy.loginfo("waiting for new_weights...")
            rospy.sleep(LOC_WAIT_TIME)
        
    def request_no_motion_update_until_minimum_particles(self):
        while self.get_particle_count() > self.min_particles + 1:
            self.request_no_motion_update_call()
            
        # Extra calls to converge
        # 5% of the min particles:
        num_extra_calls = int( self.min_particles * 0.05 )
        for i in range(num_extra_calls):
            self.request_no_motion_update_call()
            
    def request_no_motion_update_call(self):
        self.new_weights = False
        self.req_no_mot_up.call(EmptyRequest())
        self.wait_for_new_weights()
        
    def clear_costmap(self):
        self.clear_costmap_srv.call(EmptyRequest())
        
    def rotate_robot(self, degrees_of_rot):
        # We need to clear costmap before moving or we can get stuck
        self.clear_costmap()
        req = NavigationTurnRequest()
        req.enable = True
        req.degree = degrees_of_rot
        rospy.loginfo("Sending goal: " + str(degrees_of_rot))
        self.turn_srv.call(req)
        rospy.loginfo("Done rotating " + str(degrees_of_rot) + " degrees")
        
    def get_particle_count(self):
        return len(self.last_weights)

if __name__ == '__main__':
    rospy.init_node("are_we_well_loc_node")
    rospy.sleep(0.3)
    awwl = AreWeWellLoc()
    awwl.compute_localization_confidence()
    rospy.spin()

    