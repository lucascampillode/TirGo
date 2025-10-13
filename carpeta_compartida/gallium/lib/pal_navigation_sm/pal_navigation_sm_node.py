#!/usr/bin/env python3
import rospy
from pal_navigation_sm.state_machine import StateMachine
from pal_navigation_sm.pose_manager import PoseManager

from pal_navigation_msgs.srv import ChangeBuilding
from std_srvs.srv import Empty

def relay_service(old_name, new_name, service_class):
    rospy.logerr("Using Deprecated Service name! Please use %s instead!" % new_name)
    proxy = rospy.ServiceProxy(new_name, service_class)
    return rospy.Service(old_name, service_class, proxy)

if __name__ == "__main__":
    rospy.init_node("navigation_sm")

    sm = StateMachine()

    r1 = relay_service("/pal_map_manager/change_building",
                       "/pal_building_manager/change_building",
                       ChangeBuilding)

    r2 = relay_service("/pal_map_manager/save_curr_building_conf",
                       "/pal_building_manager/save_curr_building_conf",
                       Empty)
    rospy.spin()
