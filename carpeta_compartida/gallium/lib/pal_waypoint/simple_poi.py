#! /usr/bin/env python
import rospy
import actionlib
import sys

from pal_navigation_msgs.msg import GoToPOIAction, GoToPOIGoal

class SimplePOINavigation():
    def __init__(self):

        rospy.init_node('simple_poi_navigation')

    def wp_nav_client(self, target_poi):

        self.poi_client = actionlib.SimpleActionClient(
            "poi_navigation_server/go_to_poi", GoToPOIAction)
        self.poi_client.wait_for_server()

        action_goal = GoToPOIGoal()
        action_goal.poi.data = target_poi

        self.poi_client.send_goal_and_wait(action_goal)
        rospy.loginfo("goal sent to POI navigation.")

    def shutdown(self):
        self.poi_client.cancel_all_goals()

if __name__ == '__main__':
    try:
        if len(sys.argv) != 2:
            print("usage is: ./simple_poi 'poi name'")
            exit()

        target_poi = str(sys.argv[1])
        spn = SimplePOINavigation()
        spn.wp_nav_client(target_poi)
        spn.shutdown()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
