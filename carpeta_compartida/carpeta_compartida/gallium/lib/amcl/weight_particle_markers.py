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

# This node attemps to print the weights over the poses
# of the particles with lines
# I couldn't use message filters as Float64MultiArray hasn't
# header. So I made this poor people version of it. Kind of works!


from __future__ import division
from copy import deepcopy
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float64MultiArray, ColorRGBA
from geometry_msgs.msg import PoseArray, Vector3, Point, Pose


WEIGHTS_TOPIC = '/particle_laser_match_weight'
PARTICLES_TOPIC = '/particlecloud'
MARKERS_PUB = '/weight_markers'

# Publishing 500 to 5000 cylinders killed Rviz
# def get_cylinder_weight_marker(weight, id, pose, frame_id="map", color=ColorRGBA(0.7, 0.0, 0.0, 0.5)):
#     m = Marker()
#     m.action = m.ADD
#     m.header.frame_id = frame_id
#     m.id = id
#     m.type = m.CYLINDER
#     m.pose = pose
#     m.scale = Vector3(0.05, 0.05, weight)
#     m.color = color
#     return m

def create_line_list_marker(weights, particles, frame_id="map", marker_id=555, color=ColorRGBA(0.7, 0.0, 0.0, 0.2)):
    m = Marker()
    m.action = m.ADD
    m.header.frame_id = frame_id
    m.header.stamp = rospy.Time.now()
    m.id = marker_id
    m.type = m.LINE_LIST
    m.color = color
    m.pose = Pose()
    m.scale.x = 0.01
    m.lifetime = rospy.Duration(1.0)

    # normalize weights to exagerate the difference between particle weights for easier visualization
    # This can be done more efficiently for sure, sorry!
    max_w = max(weights.data)
    min_w = min(weights.data)
    tmp_weights = map(lambda x: x - min_w, weights.data) # we get rid of the minimum
    new_max = max(tmp_weights)
    weights_for_visualization = map(lambda x: x/new_max, tmp_weights)


    for weight, pose in zip(weights_for_visualization, particles.poses):
        # Create a marker for every particle, it's height represents the weight normalized
        p1 = deepcopy(pose.position)
        p2 = deepcopy(pose.position)
        p2.z = weight
        m.points.append(p1)
        m.points.append(p2)

    return m


# Using lifetime parameter I got rid of this...
# def create_markers_array_to_delete_previous(marker_array_to_delete):
#     ma = MarkerArray()
#     if len(marker_array_to_delete.markers) > 0:
#         for marker in marker_array_to_delete.markers:
#             marker.action = Marker.DELETE
#             ma.markers.append(marker)
#     return ma


class ParticleMarkers():
    def __init__(self):
        self.markers_id = 555
        self.markers_array = MarkerArray()
        self.markers_pub = rospy.Publisher(MARKERS_PUB, MarkerArray)
        self.last_weights = Float64MultiArray()
        self.weights_sub = rospy.Subscriber(WEIGHTS_TOPIC, Float64MultiArray, self.weights_cb)
        self.last_particles = PoseArray()
        self.particles_sub = rospy.Subscriber(PARTICLES_TOPIC, PoseArray, self.particles_cb)
        rospy.loginfo("ParticleMarkers ready.")


    def weights_cb(self, data):
        self.last_weights = data

    def particles_cb(self, data):
        self.last_particles = data

    def run(self):

        while not rospy.is_shutdown():
            weights = self.last_weights
            particles = self.last_particles

            if not len(weights.data) == len(particles.poses):
                rospy.logwarn("Weights array has different size from particle poses! " + str(len(weights.data)) + " != " + str(len(particles.poses)))
                rospy.sleep(0.05)
                continue

            # Create markers array to publish the weights
            self.markers_array = MarkerArray()
            line_list_marker = create_line_list_marker(weights, particles, marker_id=self.markers_id)
            self.markers_id += 1
            self.markers_array.markers.append(line_list_marker)
            self.markers_pub.publish(self.markers_array)
            rospy.sleep(0.1) # In practice this is 7Hz...


if __name__ == '__main__':
    rospy.init_node("weight_markers")
    pm = ParticleMarkers()
    rospy.sleep(0.3)
    pm.run()