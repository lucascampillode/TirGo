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

Plot current weights

'''
import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt

WEIGHTS_TOPIC = '/particle_laser_match_weight'


class PlotWeights:
    def __init__(self):
        #plt.axis([0, MAX_PARTICLES, 0, MAX_WEIGHT_VAL])
        plt.ion()
        plt.show()
        rospy.sleep(0.3)
        self.last_weights = []
        self.new_weights = False
        self.weights_sub = rospy.Subscriber(WEIGHTS_TOPIC, Float64MultiArray, self.weights_array_cb)
        
    def weights_array_cb(self, data):
        self.last_weights = data.data
        self.new_weights = True

        
    def run(self):
        while not rospy.is_shutdown():
            if self.new_weights:
                self.new_weights = False
                plt.clf()
                plt.title("Histogram of weights")
                plt.xlabel("bins")
                plt.ylabel("Laser match confidence")
                # update plot with the new data
                x_data = range(len(self.last_weights) + 1) 
                y_data = list(self.last_weights)
                y_data.sort()
                y_data.extend([0.0] * (len(x_data) - len(y_data)))
                plt.hist(y_data, 100,  range=[0.0, 1.0])
                #plt.axis([0.0, 0.1, 0, 500])
                
            plt.draw()
            #print "drawing!"
            rospy.sleep(0.05)

if __name__ == '__main__':
    rospy.init_node("weightshist_node")
    rospy.sleep(0.3)
    pw = PlotWeights()
    pw.run()

    