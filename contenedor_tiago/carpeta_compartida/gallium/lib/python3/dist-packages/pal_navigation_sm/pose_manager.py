#!/usr/bin/env python
import rospy
import yaml
import os
from std_msgs.msg import Bool
import rospkg
import shutil

class PoseManager(object):
    
    def __init__(self):
        try:
            self.path = os.path.expanduser("~/.pal/pose.yaml")
        except Exception as e:
            rospy.logerr("[pal_navigation_sm] - Pose file not found. Cannot check initial pose: {}".format(e))

    def checkIntegrityPoseFile(file_path):
        rospack = rospkg.RosPack()
        mydir = rospack.get_path('pal_navigation_sm')
        default_pose_file_path = os.path.join(mydir, 'config/pose.yaml')
        with open(file_path) as f:
            with open(default_pose_file_path) as file_default:
                default_pose = yaml.safe_load(file_default) or {}
                pose = yaml.safe_load(f) or {}
                
                for key, value in default_pose.items():
                    if not pose.has_key(key) or type(pose[key]) != type(default_pose[key]) :
                        raise Exception()

    def checkInitialPose(self):
        try:
            checkIntegrityPoseFile(self.path)
            rospy.loginfo("[pal_navigation_sm] - Valid pose.yaml")
            return
        except Exception:
            rospy.logerr("[pal_navigation_sm] - Invalid pose.yaml")

        try:
            checkIntegrityPoseFile(self.path + '.bkp')                 
            shutil.copyfile(self.path + '.bkp', self.path)
            rospy.loginfo("[pal_navigation_sm] - Successful recovery from backup pose.yaml")
            return
        except Exception:
            rospy.logerr("[pal_navigation_sm] - Invalid pose.yaml.bkp, trying reset with dock")

        docked = Bool()
        docked.data = False
        try:
            docked = rospy.wait_for_message("/power/is_docked", Bool, timeout=1)
        except Exception:
            rospy.logerr("[pal_navigation_sm] - No messages received on topic /power/is_docked! Cannot check if robot is docked")

        if docked.data:
            try:
                floor = rospy.get_param("/current_floor")
                poi = rospy.get_param("/building/floors/{}/docks/dock_1/docked_poi".format(floor))
                docked_pose = rospy.get_param("/mmap/poi/submap_0/{}".format(poi))
                
                pose = {}
                pose["initial_pose_x"] = docked_pose[2]
                pose["initial_pose_y"] = docked_pose[3]
                pose["initial_pose_a"] = docked_pose[4]
                pose["initial_cov_xx"] = 0.01
                pose["initial_cov_yy"] = 0.01
                pose["initial_cov_aa"] = 0.02

                with open(self.path, "w") as f:
                    yaml.safe_dump(pose, f)
                rospy.loginfo("[pal_navigation_sm] - Reset successful, robot in the dock!")
                return
                
            except Exception as e:
                rospy.logerr("[pal_navigation_sm] - Something went wrong during the pose recovery using the dock: {}".format(e))
               
        # Try to fix the corrupted file anyway with an empty one
        try:
            rospack = rospkg.RosPack()
            mydir = rospack.get_path('pal_navigation_sm')
            file_path = os.path.join(mydir, 'config/pose.yaml')

            shutil.copyfile(file_path, self.path)
            rospy.loginfo("[pal_navigation_sm] - Reset failed, robot out of the dock! Localize manually")
        except Exception as e:
            rospy.logerr("[pal_navigation_sm] - Something went wrong during the pose reset: {}".format(e))