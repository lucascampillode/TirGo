#! /usr/bin/env python
import rospy
import actionlib
import sys

from pal_waypoint_msgs.msg import DoWaypointNavigationAction, DoWaypointNavigationGoal


class SimpleWaypointNavigation():
    def __init__(self):

        rospy.init_node('simple_waypoint_navigation')

    def wp_nav_client(self, target_group):

        self.waypoint_client = actionlib.SimpleActionClient(
            "pal_waypoint/navigate", DoWaypointNavigationAction)
        self.waypoint_client.wait_for_server()

        action_goal = DoWaypointNavigationGoal()
        action_goal.group = target_group

        self.waypoint_client.send_goal_and_wait(action_goal)
        rospy.loginfo("goal sent to waypoint group navigation.")

    def shutdown(self):
        self.waypoint_client.cancel_all_goals()

if __name__ == '__main__':
    try:
        if len(sys.argv) != 2:
            print("usage is: ./simple_wp 'waypoing group'")
            exit()

        target_group = str(sys.argv[1])
        swp = SimpleWaypointNavigation()
        swp.wp_nav_client(target_group)
        swp.shutdown()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
