#!/usr/bin/env python
import rospy
import rosnode

import subprocess

from abc import ABCMeta, abstractmethod

from std_msgs.msg import Bool
from std_srvs.srv import Empty

from pal_navigation_msgs.srv import SaveMap

from pal_startup_msgs.srv import StartupStart, StartupStop
from pal_python import pal_environ


class AppManager(object):
    __metaclass__ = ABCMeta
    
    def __init__(self):
        if rospy.has_param("pal_robot_info/type"):
            self.robot = rospy.get_param("pal_robot_info/type")
        elif rospy.has_param("~robot"):
            self.robot = rospy.get_param("~robot")
        else:
            rospy.logerr("[pal_navigation_sm] - Cannot detect robot type! Exit!")
            return
        
        self.mapping_algorithm = rospy.get_param("~mapping", "karto")
        self.localization_algorithm = rospy.get_param("~localization", "amcl")

        self.pub = rospy.Publisher("stop_closing_loop", Bool, queue_size=1)
        self.save_map_srv = rospy.ServiceProxy("pal_map_manager/save_map", SaveMap)
        self.stop_map_srv = rospy.ServiceProxy("pal_map_manager/stop_map", Empty)
        self.start_map_srv = rospy.ServiceProxy("pal_map_manager/start_map", Empty)


    def saveMap(self):
        expectSrv(self.save_map_srv)


    def stopLocalization(self):
        if self.robot == "ari":
            if self.mapping_algorithm == "orb":
                self.stopApp("localizer")
                nodes = ["/loc_orb"]
            else:
                self.stopApp("localizer_laser")
                nodes = ["/amcl"]
        else:
            self.stopApp("localizer")
            nodes = ["/amcl"]

        # Stop the pal_map_server
        expectSrv(self.stop_map_srv)

        rosnode.kill_nodes(nodes)

    
    def stopMapping(self):
        self.stopApp('mapping_' + self.mapping_algorithm)
        nodes = [self.mapping_algorithm if 'slam' in self.mapping_algorithm else '/slam_' + self.mapping_algorithm]

        # Make sure mapping doesn't hold on twist_mux lock
        self.pub.publish(False)

        rosnode.kill_nodes(nodes)


    def startLocalization(self):

        if self.robot == "ari":
            if self.mapping_algorithm == "orb":
                self.startApp("localizer")
                nodes = ["/loc_orb"]
            else:
                self.startApp("localizer_laser")
                nodes = ["/amcl"]
        else:
            self.startApp("localizer")
            nodes = ["/amcl"]

        # Start the pal_map_server
        expectSrv(self.start_map_srv)

        if not pal_environ.wait_for_nodes(nodes, 10.0):
            rospy.logerr("[pal_navigation_sm] - Failed to start localization properly!")


    def startMapping(self):
        self.startApp("mapping_" + self.mapping_algorithm)

        nodes = [self.mapping_algorithm if 'slam' in self.mapping_algorithm else '/slam_' + self.mapping_algorithm]
        if not pal_environ.wait_for_nodes(nodes, 10.0):
            rospy.logerr("[pal_navigation_sm] - Failed to start mapping properly!")

    @abstractmethod
    def startApp(self, app_name):
        # Run an app by its name
        pass

    @abstractmethod
    def stopApp(self, app_name):
        # Stop an app by its name
        pass


class RobotAppManager(AppManager):
    def __init__(self):
        AppManager.__init__(self)
        self.startup_start_srv = rospy.ServiceProxy("pal_startup_control/start", StartupStart)
        self.startup_stop_srv = rospy.ServiceProxy("pal_startup_control/stop", StartupStop)


    def startApp(self, app_name):
        rospy.loginfo("[pal_navigation_sm] - Starting app {}".format(app_name))
        try:
            self.startup_start_srv.wait_for_service(2)
            self.startup_start_srv(app_name, "")
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logerr("[pal_navigation_sm] - Exception when starting {} : {}".format(app_name, e.message))
            return False

        return True


    def stopApp(self, app_name):
        rospy.loginfo("[pal_navigation_sm] - Stopping app {}".format(app_name))
        try:
            self.startup_stop_srv.wait_for_service(2)
            self.startup_stop_srv(app_name)
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logerr("[pal_navigation_sm] - Exception when stopping {} : {}".format(app_name, e.message))
            return False

        return True


class SimulationAppManager(AppManager):
    def __init__(self, current_state):
        AppManager.__init__(self)
        self.multiple = rospy.get_param("~multiple", False)
        self.robot_namespace = rospy.get_param("~robot_namespace", "")
        self.launched = dict()
        self.apps = {
            "localizer": "roslaunch {}_2dnav localization.launch localization:={} multiple:={} robot_namespace:={}"
                         .format(self.robot, self.localization_algorithm, self.multiple, self.robot_namespace),
            "mapping_{}".format(self.mapping_algorithm): "roslaunch {}_2dnav mapping.launch mapping:={}".format(self.robot, self.mapping_algorithm)
        }

        if self.robot == "ari":
            # Adds the localizer_laser to the list of applications
            self.apps.update({
                "localizer_laser": "roslaunch {}_2dnav localization.launch localization:={}".format(self.robot, self.localization_algorithm)
            })

        if self.mapping_algorithm == "orb":
            # Adds the _sim suffix to the mapping and localization launch files
            self.apps.update({
                "mapping_{}".format(self.mapping_algorithm): "roslaunch {}_2dnav mapping.launch mapping:={}_sim".format(self.robot, self.mapping_algorithm),
                "localizer": "roslaunch {}_2dnav localization.launch localization:={}_sim".format(self.robot, self.localization_algorithm)
            })

        if current_state == "LOC":
            self.startApp("localizer")

        elif current_state == "MAP":
            self.startApp("mapping_{}".format(self.mapping_algorithm))
            # Stops the pal_map_server to publish the map (done by the mapping)
            expectSrv(self.stop_map_srv)


    def start(self, app_name, config):
        if app_name in self.launched:
            rospy.loginfo("[pal_navigation_sm] - Node {} is already running".format(app_name))
            return

        self.launched[app_name] = subprocess.Popen(config, shell=True)


    def startApp(self, app_name):
        rospy.loginfo("[pal_navigation_sm] - Starting app {}".format(app_name))
        config = self.apps.get(app_name, None)
        if config is None:
            rospy.loginfo("[pal_navigation_sm] - There is no config for {}".format(app_name))
            return
        self.start(app_name, config)

        
    def stopApp(self, app_name):
        rospy.loginfo("[pal_navigation_sm] - Stopping app {}".format(app_name))
        process = self.launched.pop(app_name, None)
        if process is None:
            rospy.loginfo("[pal_navigation_sm] - app {} is not running".format(app_name))
            return
        process.terminate()
        process.wait()
        rospy.loginfo("[pal_navigation_sm] - app {} stopped".format(app_name))


def expectSrv(srv):
    '''
    Checks whether a Service exists and if so calls it
    '''
    try:
        srv.wait_for_service(5.0)
    except rospy.ROSException as e:
        rospy.logerr("Service {} is not available! {}".format(srv.resolved_name, e))
        return False

    try:
        srv()
    except rospy.ServiceException as e:
        rospy.logerr("There was an error while calling the service {}! {}".format(srv.resolved_name, e))
        return False

    return True
