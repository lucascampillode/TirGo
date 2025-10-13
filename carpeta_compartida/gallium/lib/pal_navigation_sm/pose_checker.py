#!/usr/bin/env python3

import os
import sys
import time

import rospy

def main(command):
  rospy.init_node("pose_checker_node", anonymous=True)

  rospy.get_param('/pose_manager/check_pose', 'False')
  while rospy.get_param('/pose_manager/check_pose', 'False') == 'False':
    if rospy.is_shutdown() :
      sys.exit()
    time.sleep(1)

  os.system( command )

if __name__ == "__main__":
  try:
    if len(sys.argv) < 2:
      print("usage: pose_checker_node.py 'cmd command'")
    else :
      main(sys.argv[1])
  except rospy.ROSInterruptException:
    pass