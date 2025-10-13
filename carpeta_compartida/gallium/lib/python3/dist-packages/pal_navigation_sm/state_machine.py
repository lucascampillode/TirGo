#!/usr/bin/env python
import rospy
from enum import Enum

from std_msgs.msg import String

from pal_navigation_msgs.msg import NavigationStatus
from pal_navigation_msgs.srv import Acknowledgment, AcknowledgmentResponse
from pal_common_msgs.srv import GetString, GetStringResponse

from pal_navigation_sm.app_manager import SimulationAppManager, RobotAppManager


class State(Enum):
    LOC = 1
    MAP = 2
    STOP = 3

DEFAULT_STATE = State.STOP


class StateMachine():
    
    def __init__(self):
        initial_state = rospy.get_param("~state", DEFAULT_STATE.name)
        if initial_state == "localization":
            self.current_state_ = State.LOC
        elif initial_state == "mapping":
            self.current_state_ = State.MAP
        elif initial_state == "stop":
            self.current_state_ = State.STOP
        else:
            rospy.logerr("[pal_navigation_sm] - Initial state {} is not in the list of possible states {}, Using default state {}"
                        .format(initial_state, ["localization", "mapping", "stop"], DEFAULT_STATE.name))
            self.current_state_ = DEFAULT_STATE

        rospy.loginfo("[pal_navigation_sm] - Starting Navigation SM in state {}".format(self.current_state_.name))

        is_sim = rospy.get_param("~is_sim", False)
        if is_sim:
            self.app_manager_ = SimulationAppManager(self.current_state_.name)
        else:
            self.app_manager_ = RobotAppManager()    

        # ROS Interface
        self.pub_ = rospy.Publisher("pal_navigation_sm/state", NavigationStatus, queue_size=1, latch=True)
        self.srv_ = rospy.Service("pal_navigation_sm", Acknowledgment, self.requestCallback)
        self.srv_ = rospy.Service("pal_navigation_sm/state", GetString, self.stateCallback)
        
        self.state_msg_ = NavigationStatus(status=String(self.current_state_.name), msg=String(""))

        # Publish the Initial State
        self.pub_.publish(self.state_msg_)


    def requestCallback(self, req):
        if req.input == "SAVE":
            if self.current_state_.name == "MAP":
                result = self.app_manager_.saveMap()
                if result:
                    return AcknowledgmentResponse(True, "OK")
                else:
                    return AcknowledgmentResponse(False, "Cannot save map.")
            else:
                return AcknowledgmentResponse(False, "Cannot save map, not in mapping mode.")

        rospy.loginfo("[pal_navigation_sm] - Requested to change State from {} to {}".format(self.current_state_.name, req.input))

        if req.input not in [state.name for state in State]:
            return AcknowledgmentResponse(False, "Received unrecognized command {}".format(req.input))

        if req.input == self.current_state_.name:
            return AcknowledgmentResponse(True, "Already in mode {}".format(req.input))

        # Automatically save the map before changing state
        if self.current_state_.name == "MAP":
            result = self.app_manager_.saveMap()

        if req.input == "LOC":
            # Change to Localization
            self.app_manager_.startLocalization()
            self.app_manager_.stopMapping()
            self.current_state_ = State.LOC

        elif req.input == "MAP":
            # Change to Mapping
            self.app_manager_.stopLocalization()
            self.app_manager_.startMapping()
            self.current_state_ = State.MAP

        elif req.input == "STOP":
            # Change to Stopped
            self.app_manager_.stopLocalization()
            self.app_manager_.stopMapping()
            self.current_state_ = State.STOP

        self.state_msg_.status.data = self.current_state_.name
        self.pub_.publish(self.state_msg_)

        return AcknowledgmentResponse(True, "Navigation mode successfully changed.")


    def stateCallback(self, req):
        rospy.loginfo("[pal_navigation_sm] - Requested current State {}".format(self.current_state_.name))
        return GetStringResponse(data=self.current_state_.name, success=True)
