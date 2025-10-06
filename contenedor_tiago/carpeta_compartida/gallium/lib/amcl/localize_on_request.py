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


'''
@author: Sammy Pfeiffer

Node that tries to better the current 
localization of the robot based on the
best particle weight.

'''
import math
import numpy as np
import rospy
import rosparam
from amcl.srv import Randomize, RandomizeRequest
from std_srvs.srv import Empty, EmptyRequest
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import MapMetaData

from nav_by_odom.srv import NavigationTurn, NavigationTurnResponse, NavigationTurnRequest

from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from tf.transformations import quaternion_from_euler

from collections import deque

MOVE_BASE_GOAL_TOPIC = '/move_base_simple/goal'
MOVE_BASE_AS = '/move_base'


#WEIGHTS_TOPIC = '/weights_array'
INTERNAL_WEIGHTS_TOPIC = '/particle_laser_match_weight'
WEIGHTS_TOPIC = INTERNAL_WEIGHTS_TOPIC
MAP_METADATA_TOPIC = '/map_metadata'
LOCAL_LOC_SRV = '/local_localization'
GLOBAL_LOC_SRV = '/global_localization'
CLEAR_COSTMAP_SRV = '/move_base/clear_costmaps'
NO_MOTION_UPDATE_SRV = '/request_nomotion_update'
TURN_BY_ODOM_SRV= '/turn'

MIN_PARTICLES_PARAM = '/amcl/min_particles'

GLOBAL_TH = 1.0 - 0.85
MINIMUM_USEFUL_TH = 1.0 - 0.6
LOCAL_TH = 1.0 - 0.3
REFINED_TH = 1.0 - 0.05

MINIMUM_WELL_LOC_TH = 1.0 - 0.6

WELL_LOC_MAX_AVG_WEIGHT = 1.0 - 0.2

# TODO: Add Max tries and stuff
MINIMUM_RADIUS = 0.3 # relate to the robot base radius?
LOC_WAIT_TIME = 0.1

class LocalizeNode:
    def __init__(self):
        rospy.loginfo("Connecting to service: " + GLOBAL_LOC_SRV)
        self.global_loc_srv = rospy.ServiceProxy(GLOBAL_LOC_SRV, Empty)
        self.global_loc_srv.wait_for_service()
        rospy.loginfo("Connecting to service: " + LOCAL_LOC_SRV)
        self.local_loc_srv = rospy.ServiceProxy(LOCAL_LOC_SRV, Randomize)
        self.local_loc_srv.wait_for_service()
        rospy.loginfo("Connecting to service: " + CLEAR_COSTMAP_SRV)
        self.clear_costmap_srv = rospy.ServiceProxy(CLEAR_COSTMAP_SRV, Empty)
        self.clear_costmap_srv.wait_for_service()
        rospy.loginfo("Connecting to service: " + NO_MOTION_UPDATE_SRV)
        self.req_no_mot_up = rospy.ServiceProxy(NO_MOTION_UPDATE_SRV, Empty)
        self.req_no_mot_up.wait_for_service()
        rospy.loginfo("Connecting to service: " + TURN_BY_ODOM_SRV)
        self.turn_srv = rospy.ServiceProxy(TURN_BY_ODOM_SRV, NavigationTurn)
        self.turn_srv.wait_for_service()
        rospy.loginfo("Subscribing to: " + WEIGHTS_TOPIC)
        rospy.loginfo("Connecting to AS: " + MOVE_BASE_AS)
        self.move_as = SimpleActionClient(MOVE_BASE_AS, MoveBaseAction)
        self.move_as.wait_for_server()
        rospy.loginfo("Connected.")
        rospy.loginfo("Getting param " + MIN_PARTICLES_PARAM)
        self.min_particles = int(rosparam.get_param(MIN_PARTICLES_PARAM))
        rospy.loginfo("Got " + str(self.min_particles))
        self.last_weights = None
        self.last_weights_best_log = deque(maxlen=100)
        self.new_weights = False
        self.store_weights = False
        self.max_weights= []
        self.weights_sub = rospy.Subscriber(WEIGHTS_TOPIC, Float64MultiArray, self.weights_array_cb)
        self.radius_array = self.create_radius_array()
        self.max_tries_global_calls = 10
        rospy.loginfo("Running LocalizeNode.")

    def get_map_max_dist(self):
        """Get the biggest distance possible of this map from the map metadata"""
        mmd = rospy.wait_for_message(MAP_METADATA_TOPIC, MapMetaData)
        # Diagonal of the rectangle that represents the map is the longest distance
        diagonal = math.sqrt((mmd.width * mmd.width + mmd.height * mmd.height))
        # Map from pixels to map resolution (meters)
        map_max_dist = diagonal * mmd.resolution
        rospy.loginfo("Maximum map distance is: " + str(map_max_dist))
        return map_max_dist
    
    def create_radius_array(self):
        """Create an array of radius to do local calls based on the size of the map"""
        map_max_dist = self.get_map_max_dist()
        radius_array = []
        curr_radius = float(map_max_dist)
        while curr_radius > MINIMUM_RADIUS:
            radius_array.append(curr_radius)
            curr_radius /= 2.0
        rospy.loginfo("radius_array is: " + str(radius_array))
        return radius_array
    
    def weights_array_cb(self, data):
        self.last_weights = data.data # Array of Float64
        self.new_weights = True
        maxval = max(data.data)
        # store a log of the best weights
        self.last_weights_best_log.append(maxval)
        if self.store_weights:
            self.max_weights.append(maxval)
        
    def run(self):
        initial_time = rospy.Time.now()
        rospy.loginfo("Trying to (better) localize!")
        #global_calls_counter = self.do_global_calls()
        global_calls_counter = 0
        local_calls_counter, total_req_no_mot_calls_counter = self.do_local_calls_with_max_tries() #self.do_local_calls()
        rospy.loginfo("Local localization got us to a particle with the best weight: " + str(self.get_best_weight()))
        final_time = rospy.Time.now()
        used_time = final_time - initial_time
        rospy.loginfo("Clearing costmap.")
        self.clear_costmap()
        #rospy.sleep(1.0)
        rospy.loginfo("We took: " + str(float(used_time.secs) + (float(used_time.nsecs) / 1000000000.0)) + "s to relocalize.\n"
                      + "We did " + str(global_calls_counter) + " Global calls and " + str(local_calls_counter) + " Local calls  and " 
                      + str(total_req_no_mot_calls_counter) + " request no motion update calls.\n"
                      + "Final best weight is: " + str(self.get_best_weight()))
        
        
    def run_with_movements(self, initial_time=None):
        if initial_time == None:
            initial_time = rospy.Time.now()
        global_converge_tries = 0
        while global_converge_tries < self.max_tries_global_calls:
            rospy.loginfo("Try #" + str(global_converge_tries) + " / " + str(self.max_tries_global_calls) + " of converging global calls")
            best_accomplished_weight = self.try_converge_global_call()
            global_converge_tries += 1
            if best_accomplished_weight > REFINED_TH:
                rospy.loginfo("We got a great weight (" + str(best_accomplished_weight) + ") better than refined threshold: "
                              + str(REFINED_TH) + "! Stopping, we should be awesomely localized!")
                final_time = rospy.Time.now()
                used_time = final_time - initial_time
                rospy.loginfo("We took: " + str(float(used_time.secs) + (float(used_time.nsecs) / 1000000000.0)) + "s to relocalize.\n")
                return
        
        rospy.loginfo("We couldn't converge to a location from here, rotating the robot 90 degrees and trying again.")
        self.rotate_robot(90)
        # TODO: max tries on this... this is an infinite loop now
        self.run_with_movements(initial_time)
        
    def run_with_movements_averaged_score(self, initial_time=None):
        if initial_time == None:
            initial_time = rospy.Time.now()
        global_converge_tries = 0
        while global_converge_tries < self.max_tries_global_calls:
            rospy.loginfo("Try #" + str(global_converge_tries) + " / " + str(self.max_tries_global_calls) + " of converging global calls")
            avg_accomplished_weight = self.try_converge_global_call_avg_scored()
            global_converge_tries += 1
            if avg_accomplished_weight > WELL_LOC_MAX_AVG_WEIGHT:
                rospy.loginfo("We got a great weight (" + str(avg_accomplished_weight) + ") better than WELL_LOC_MAX_AVG_WEIGHT threshold: "
                              + str(WELL_LOC_MAX_AVG_WEIGHT) + "! Stopping, we should be awesomely localized!")
                final_time = rospy.Time.now()
                used_time = final_time - initial_time
                rospy.loginfo("We took: " + str(float(used_time.secs) + (float(used_time.nsecs) / 1000000000.0)) + "s to relocalize.\n")
                return
        
        rospy.loginfo("We couldn't converge to a location from here, rotating the robot 90 degrees and trying again.")
        self.rotate_robot(90)
        # TODO: max tries on this... this is an infinite loop now
        self.run_with_movements_averaged_score(initial_time)
        
    def try_converge_global_call(self):
        # Do global call
        rospy.loginfo("Initial 'global' loc call with radius: " + str(self.radius_array[0]))
        self.local_call(self.radius_array[0]) # biggest radius
        # Loop request update until minimum particles
        rospy.loginfo("Converging using no_motion_update calls")
        self.request_no_motion_update_until_minimum_particles()
        # Check best weight if > MINIMUM_WELL_LOC_TH and store pose (X,Y) and weight
        initial_weight = self.get_best_weight()
        rospy.loginfo("Converged to a weight of " + str(initial_weight))
        if self.get_best_weight() > MINIMUM_WELL_LOC_TH:
            rospy.loginfo("We surpassed " + str(MINIMUM_WELL_LOC_TH) + " threshold of minimum to believe we are localized, rotating 180 degrees...")
            # Rotate robot 180 degrees (so we have, in two shots, the 360deg of the robot)
            self.rotate_robot(180)
            # Call local loc with not very big range
            rospy.loginfo("Calling a local localization with radius: " + str(self.radius_array[-3]))
            self.local_call(self.radius_array[-3]) # almost tiniest option
            # Loop request update until minimum particles
            rospy.loginfo("Converging again using no_motion_update calls")
            self.request_no_motion_update_until_minimum_particles()
            

            # Check best weight still ok, if so, we are done
            rospy.loginfo("We got to weight: " + str(self.get_best_weight()))
        after_turn_weight = self.get_best_weight()
        if initial_weight > after_turn_weight:
            return initial_weight
        return after_turn_weight
    
    def try_converge_global_call_avg_scored(self):
        # Do global call
        rospy.loginfo("Initial 'global' loc call with radius: " + str(self.radius_array[0]))
        self.local_call(self.radius_array[0]) # biggest radius
        # Loop request update until minimum particles
        rospy.loginfo("Converging using no_motion_update calls")
        self.request_no_motion_update_until_minimum_particles()
        # Check best weight if > MINIMUM_WELL_LOC_TH and store pose (X,Y) and weight
        initial_weight = self.get_best_weight()
        rospy.loginfo("Converged to a weight of " + str(initial_weight))
        if self.get_best_weight() > MINIMUM_WELL_LOC_TH:
            rospy.loginfo("We surpassed " + str(MINIMUM_WELL_LOC_TH) + " threshold of minimum to believe we are localized") #, rotating 180 degrees...")

            # Call local loc with not very big range
            rospy.loginfo("Calling a local localization with radius: " + str(self.radius_array[-3]))
            self.local_call(self.radius_array[-3]) # almost tiniest option
            # Loop request update until minimum particles
            rospy.loginfo("Converging again using no_motion_update calls")
            self.request_no_motion_update_until_minimum_particles()
            
            # Rotate robot 180 degrees (so we have, in two shots, the 360deg of the robot)
            avg_weight = self.rotate_and_score_loc(180)
            if avg_weight > WELL_LOC_MAX_AVG_WEIGHT:
                rospy.loginfo("   [(spoiler) avg weight is: "+ str(avg_weight) +" ] we surpassed WELL_LOC_MAX_AVG_WEIGHT! (" + str(WELL_LOC_MAX_AVG_WEIGHT) + ")")
            # Check best weight still ok, if so, we are done
            rospy.loginfo("We got to  avg weight: " + str(avg_weight))
        return avg_weight
        
    def rotate_and_score_loc(self, degrees_of_rot):
        # Start storing the best values of localization best particle weight
        self.max_weights = []
        self.store_weights = True
        self.rotate_robot(degrees_of_rot)
        self.store_weights = False
        avg_weight = np.average(self.max_weights)
        return avg_weight
        
        
    def rotate_robot(self, degrees_of_rot):
        # We need to clear costmap before moving or we can get stuck
        self.clear_costmap()
        req = NavigationTurnRequest()
        req.enable = True
        req.degree = degrees_of_rot
        rospy.loginfo("Sending goal: " + str(degrees_of_rot))
        self.turn_srv.call(req)
        rospy.loginfo("Done rotating " + str(degrees_of_rot) + " degrees")
        
#         rot_g = MoveBaseGoal()
#         rot_g.target_pose.header.frame_id = "base_footprint"
#         q = Quaternion(*quaternion_from_euler(0.0, 0.0, math.radians(degrees_of_rot)))
#         rot_g.target_pose.pose.orientation = q
#         
#         rospy.loginfo(" == Waiting for new weights before moving == ")
#         self.wait_for_new_weights()
#         
#         rospy.loginfo("Sending goal: " + str(degrees_of_rot))
#         self.move_as.send_goal_and_wait(rot_g)
#         rospy.loginfo("Done rotating " + str(degrees_of_rot) + " degrees")
#         
#         rospy.loginfo(" == Waiting for new weights after moving == ")
        
    def do_global_calls(self):
        global_calls_counter = 1
        self.global_call()
        while self.get_best_weight() > GLOBAL_TH:
            global_calls_counter += 1
            self.global_call()
        rospy.loginfo("Global localization got us to a particle with the best weight: " + str(self.get_best_weight()))
        rospy.loginfo("Refining with local localization calls.")
        return global_calls_counter
        
    def do_local_calls(self):
        local_calls_counter = 1
        for radius in self.radius_array: # Reduce the radius each time to try to get a more accurate position
            this_radius_calls_counter = 1
            self.local_call(radius)
            while self.get_best_weight() < LOCAL_TH:
                this_radius_calls_counter += 1
                local_calls_counter += 1
                self.local_call(radius)
                self.request_no_motion_update_call()
            rospy.loginfo("Radius: " + str(radius) + " -> " + str(this_radius_calls_counter) + " calls.")
                
        return local_calls_counter
    
    
    def do_local_calls_with_max_tries(self):
        local_calls_counter = 1
        total_req_no_mot_calls_counter = 0
        for radius in self.radius_array: # Reduce the radius each time to try to get a more accurate position
            this_radius_calls_counter = 1
            self.local_call(radius)
            while self.get_best_weight() < LOCAL_TH:#REFINED_TH:
                this_radius_calls_counter += 1
                local_calls_counter += 1
                self.local_call(radius)
                max_tries = 30
                initial_tries = 10
                curr_tries = 0
                req_no_mot_up_calls_counter = 1
                self.request_no_motion_update_call()
                while self.get_best_weight() > REFINED_TH and curr_tries < max_tries:
                    if curr_tries > initial_tries and self.get_best_weight() > MINIMUM_USEFUL_TH:
                        break
                    self.request_no_motion_update_call()
                    req_no_mot_up_calls_counter += 1
                    curr_tries += 1
                rospy.loginfo("Req no mot up " + str(req_no_mot_up_calls_counter) + " calls.")
                total_req_no_mot_calls_counter += req_no_mot_up_calls_counter
            rospy.loginfo("Radius: " + str(radius) + " -> " + str(this_radius_calls_counter) + " calls.")
                
        return local_calls_counter, total_req_no_mot_calls_counter
        
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
    
    def global_call(self):
        #rospy.loginfo("Global localization call!")
        self.new_weights = False
        self.global_loc_srv.call(EmptyRequest())
        self.wait_for_new_weights()
        #rospy.loginfo("Global localization call done.")
        
    def local_call(self, radius):
        #rospy.loginfo("Local localization call!")
        self.new_weights = False
        self.local_loc_srv.call(RandomizeRequest(radius))
        self.wait_for_new_weights()
        #rospy.loginfo("Local localization call done.")
        
    def clear_costmap(self):
        self.clear_costmap_srv.call(EmptyRequest())
        
    def get_best_weight(self):
        if len(self.last_weights) < 1:
            rospy.logwarn("Weights array was empty!")
            return 0.0
        return max(self.last_weights)
    
    def get_particle_count(self):
        return len(self.last_weights)

    def refine_current_pose(self, radius):
        self.wait_for_new_weights()
        initial_weight = self.get_best_weight()
        rospy.loginfo("Current best weigth: " + str(initial_weight))
        self.local_call(1.0)
        self.request_no_motion_update_until_minimum_particles()
        final_weight = self.get_best_weight()
        rospy.loginfo("After refining pose best weight:" + str(final_weight))

    def get_last_best_weights_avg(self, avg_qtty):
        """Get the avg of the last best weights in the log"""
        import itertools
        print "len(self.last_weights_best_log) - 1 - avg_qtty:" + str(len(self.last_weights_best_log) - 1 - avg_qtty)
        print "len(self.last_weights_best_log) - 1):" + str(len(self.last_weights_best_log) - 1)
        list_to_avg = list(itertools.islice(self.last_weights_best_log,
                                            len(self.last_weights_best_log) - avg_qtty,
                                            len(self.last_weights_best_log) - 1)) # to treat deque as a list, well, using slices
        #list_to_avg = self.last_weights_best_log[len(self.last_weights_best_log) - avg_qtty:] # cant use slice
        print "Averaging: " + str(list_to_avg)
        avg = np.average(list_to_avg)
        return avg
    
if __name__ == '__main__':
    rospy.init_node("localization_node_internal_w")
    rospy.sleep(0.3)
    l_node = LocalizeNode()
    #l_node.run_with_movements()
    l_node.run_with_movements_averaged_score()
    
