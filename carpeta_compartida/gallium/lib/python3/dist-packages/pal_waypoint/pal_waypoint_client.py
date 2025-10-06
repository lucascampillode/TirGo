#!/usr/bin/env python

import rospy
import pal_waypoint_msgs.msg as wp
import actionlib
import threading
import math


class RobotControl:

    def __init__(self, client="navigate"):
        waypoint_namespace = "pal_waypoint/"
        self.navigate_client = actionlib.SimpleActionClient(
            waypoint_namespace + client, wp.DoWaypointNavigationAction)

        rospy.loginfo("Waiting for servers to be available")
        self.navigate_client.wait_for_server()
        self.active = False
        self.last_wp_group = None
        self.waypoint_group = None

    def __del__(self):
        self.navigate_client.cancel_all_goals()

    def go_to_initial_wp(self, group, async=False):
        result = self.do_wp_navigation(group, 0, 1, async)
        if async:
            return False
        if result:
            rospy.loginfo("Initial waypoint reached")
            return True
        else:
            rospy.logerr("Initial waypoint not reached")
            return False

    def nav_feedback_cb(self, feedback):
        self.waypoint_group = feedback.waypoints

    def do_wp_navigation(self, group, first_wp=0, num_wp=0, async=False,
                         continue_on_error=False):
        """
        Navigate through a group of POIs

        Args:
            group (String): name of group of POIs to navigate
            first_wp (int, optional): index of the first POI to visit
            num_wp (int, optional): number of waypoints to visit
            async (bool, optional): if do navigation in a thread or not
            continue_on_error (bool, optional): continue if fails a POI

        Returns:
            bool: True if the last waypoint was reached, False otherwise
        """
        rospy.loginfo("Proceeding with waypoint navigation")
        goal = wp.DoWaypointNavigationGoal()
        goal.group = group
        goal.first_wp = first_wp
        goal.num_wp = num_wp
        goal.continue_on_error = continue_on_error
        if async:
            self.thread = threading.Thread(
                target=self.do_wp_navigation_core, args=(goal,))
            self.thread.start()
        else:
            return self.do_wp_navigation_core(goal)

    def do_wp_navigation_core(self, goal):
        self.active = True
        if not self.navigate_client.wait_for_server(rospy.Duration(2)):
            rospy.logerr("Could not connect to waypoint navigation server")
            return False
        self.goal = self.navigate_client.send_goal(
            goal, feedback_cb=self.nav_feedback_cb, done_cb=self.done_cb)
        self.navigate_client.wait_for_result()

        goal_state = self.navigate_client.get_state()
        if goal_state == actionlib.GoalStatus.SUCCEEDED:
            return True
        else:
            return False

    def done_cb(self, status, result):
        rospy.loginfo("Navigation finished in state {}"
                      .format(actionlib.GoalStatus.to_string(
                              self.navigate_client.get_state())))
        self.active = False

    def wait_for_activation(self, timeout):
        """
        Safety for avoid checkin if navigator "is_done" (inactive) before
        it had the time to processes the request and becomes active

        Args:
            timeout (double): How long to wait for activation

        Returns:
            bool: True if the client becomes active in the given time,
                  False if it reaches the timeout without becoming active
        """
        max_duration = rospy.Duration(timeout)
        start_time = rospy.Time.now()
        while self.is_done():
            if rospy.Time.now() - start_time > max_duration:
                rospy.logerr("Navigator is not active after {}s."
                             .format(timeout))
                return False
            else:
                rospy.logwarn("Waiting for navigator to become active.")
                rospy.sleep(0.5)
        return True

    def is_done(self):
        return not self.active

    def is_successful(self):
        return (self.navigate_client.get_state() ==
                actionlib.GoalStatus.SUCCEEDED)

    def abort(self):
        if self.is_done():
            rospy.loginfo("Pal Waypoint abort requested, "
                          "but no goal is active.")
            return

        rospy.loginfo("Pal Waypoint abort requested. Cancelling goal.")
        self.navigate_client.cancel_goal()

    def wait_for_result(self, timeout=rospy.Duration()):
        return self.navigate_client.wait_for_result(timeout)

    def is_max_failures_reached(self, ratio, min_failures, max_failures=20):
        """
        Checks if max number of CONSECUTIVE waypoint failures is reached

        Args:
            ratio (double): proportion of wp group POIs need to reach a failure
            min_failures (int): min number of consecutive failures

        Returns:
            bool: True if more then max consecutive failures is reached
        """
        consecutive_failures = 0

        # if goals state has not changed, there is nothing to check
        if self.last_wp_group == self.waypoint_group:
            return False
        self.last_wp_group = self.waypoint_group

        # correct group size to discard PODs
        true_size = (len(self.waypoint_group) -
                     self.waypoint_group.count(waypoint_status.SPECIAL))

        max_failed_wps = max(math.ceil(true_size * ratio), min_failures)
        max_failed_wps = min(max_failed_wps, max_failures)

        for element in self.waypoint_group:
            if element == waypoint_status.NOT_VISITED:
                break
            elif element == waypoint_status.SPECIAL:
                pass
            elif element == waypoint_status.SUCCESS:
                consecutive_failures = 0
                pass
            elif element == waypoint_status.FAILURE:
                consecutive_failures += 1

        if consecutive_failures:
            rospy.logwarn(
                "consecutive failed waypoints so far: {}, max acceptable: {}"
                .format(consecutive_failures, max_failed_wps))

        if(consecutive_failures > max_failed_wps):
            return True
        else:
            return False


class waypoint_status:
    """
    Waypoint status code
    """
    NOT_VISITED = -1
    SUCCESS = 0
    FAILURE = 1
    SPECIAL = 2
