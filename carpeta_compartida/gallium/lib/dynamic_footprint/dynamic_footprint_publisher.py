#!/usr/bin/env python

from math import sqrt, pow
import rospy

from dynamic_footprint.srv import ChangeParams, ChangeParamsResponse
from geometry_msgs.msg import Polygon, PolygonStamped, Point32

from dynamic_footprint import generate_convex_polygon
from tf_lookup import TfStreamClient

def transform_eq(a, b):
  """ StampedTransform comparison.

  Checks whether the transformations in two StampedTransforms are almost equal.
  """
  a = a.transform
  b = b.transform

  def floats_eq(t):
    """ Checks if two floats in a 2-tuple are almost equal. """
    return abs(t[0] - t[1]) < 1e-5

  values = (
      (a.translation.x, b.translation.x),
      (a.translation.y, b.translation.y),
      (a.translation.z, b.translation.z),
      # we don't care about rotation
  )
  return all(map(floats_eq, values))

class Node:

  def __init__(self, footprint):
    self.link_list = rospy.get_param('~link_list', [])
    self.padding    = rospy.get_param('~padding', 0.0)
    self.tf_stream  = TfStreamClient()
    self.pubp = rospy.Publisher('~footprint', Polygon, queue_size=10)
    self.pubs = rospy.Publisher('~footprint_stamped', PolygonStamped, queue_size=10)
    self.serviceServer = rospy.Service('~change_params', ChangeParams, self.changeParamsCb)
    self.footprint = footprint
    rospy.loginfo(footprint)
    self.rate = rospy.Rate(10)

    self.tf_cache = {}
    self.tf_cache_dirty = True
    for link_name in self.link_list:
      self.tf_cache[link_name] = None
      self.tf_stream.add_transform('base_footprint', link_name, self.tfCb)

  def tfCb(self, transform):
    frame = transform.child_frame_id
    if self.tf_cache[frame]:
      if transform_eq(transform, self.tf_cache[frame]):
        self.tf_cache[frame].header = transform.header
        return
    self.tf_cache[frame] = transform
    self.tf_cache_dirty = True

  def tf_available(self):
    link_name = self.link_list[0]
    transform = self.tf_cache[link_name]
    if transform:
      return True
    else:
      self.tf_stream.add_transform('base_footprint', link_name, self.tfCb)
      return False

  def get_dynamic_points(self):
    points = []
    for link_name in self.link_list:
      transform = self.tf_cache[link_name]
      if transform:
        # TODO: check transform age?
        position = [transform.transform.translation.x, transform.transform.translation.y]
        vect_length = sqrt(pow(position[0],2) + pow(position[1],2))
        if self.padding > 0.0 and vect_length != 0:
          position[0] = position[0] * (1.0 + (self.padding/vect_length))
          position[1] = position[1] * (1.0 + (self.padding/vect_length))
        points.append(position)
      else:
        rospy.logerr('Link ' + link_name + ' does not exist.')
    return points

  def loop(self):
    while not rospy.Time.now():
      pass  # Waiting for clock

    while not self.tf_available() and not rospy.is_shutdown():
     rospy.sleep(0.1) 

    polygon = None
    while not rospy.is_shutdown():
      if not polygon or self.tf_cache_dirty:
        self.tf_cache_dirty = False
        points = self.get_dynamic_points()
        if len(points) != 0:
          # Fill in polygon
          points = generate_convex_polygon.get_convex_hull(points + self.footprint)
        else:
          # If there are no links for some reason, go with the basic footprint
          points = self.footprint
        polygon = [Point32(p[0], p[1], 0) for p in points]

      for link_name in self.link_list:
        transform = self.tf_cache[link_name]
        if not transform:
          self.tf_stream.add_transform('base_footprint', link_name, self.tfCb)

      polygon_stamped = PolygonStamped()
      polygon_stamped.header.frame_id = 'base_footprint'
      polygon_stamped.header.stamp = rospy.Time.now()
      polygon_stamped.polygon.points = polygon

      self.pubs.publish(polygon_stamped)
      self.pubp.publish(polygon_stamped.polygon)
      self.rate.sleep()

  def changeParamsCb(self, req):
    if req.rate != 0:
      self.rate = rospy.Rate(req.rate)
      rospy.loginfo('Changed rate to ' + str(req.rate) + ' Hz.')
    if req.padding >= 0.0:
      self.padding = req.padding
      rospy.loginfo('Changed padding to ' + str(req.padding))
    return ChangeParamsResponse()

if __name__ == '__main__':
  rospy.init_node('dynamic_footprint_publisher')
  footprint = rospy.get_param('~footprint', [])
  node = Node(footprint)
  node.loop()
