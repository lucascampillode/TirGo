#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
from __future__ import division
import math
import threading
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped, PoseArray
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from .anomaly import *
from .useful_func import *

MAX_DIST_IN_LOST_STATE = 20.0  # distance in meters that the robot can move in LOST state
lastOdomX = 0.0
lastOdomY = 0.0

class Listener:

    def __init__(self, training):
        self.prevOdo = Pose2D(0.0, 0.0, 0.0)
        self.prevLoc = Pose2D(0.0, 0.0, 0.0)
        self.prevScanMsg = Pose2D(0.0, 0.0, 0.0)

        self.training = training
        print(self.training)
        self.status = 'LOCALIZED' if self.training else 'LOST'

        self.minVal = 100

        self.scan_time = -1.0
        self.start_recording = False
        self.currentOdoMsg = Odometry()
        self.prevTime = rospy.get_time()

        self.spaceMovedUnlocalized = 0.0

        if not self.training:
            print("STARTING IN RUNNING MODE")
            """ Load params for gaussians """
            mX = rospy.get_param('/pal_loc_measure/loc_mes_mX')
            vX = rospy.get_param('/pal_loc_measure/loc_mes_vX')
            mY = rospy.get_param('/pal_loc_measure/loc_mes_mY')
            vY = rospy.get_param('/pal_loc_measure/loc_mes_vY')
            mT = rospy.get_param('/pal_loc_measure/loc_mes_mT')
            vT = rospy.get_param('/pal_loc_measure/loc_mes_vT')
            th1 = rospy.get_param('/pal_loc_measure/loc_mes_th1')
            th2 = rospy.get_param('/pal_loc_measure/loc_mes_th2')

            """ Values of covars for SIMULATION """
            #           (mX = 0.013, vX = 0.006,
            #           mY = 0.012, vY = 0.0036,
            #           mT = 0.032, vT = 0.174,
            #           th1 = 0.1, th2 = 0.001)
        else:
            print("STARTING IN TRAINING MODE")
            """ in case we are training we create the arrays that will contain the data """
            self.covarXs = []
            self.covarYs = []
            self.covarTs = []

            self.incScansOdomDiffXs = []
            self.incScansOdomDiffYs = []
            self.incScansOdomDiffTs = []
            mX = mY = mT = vX = vY = vT = th1 = th2 = 0.0

        self.covarGauss = gauss_array(mX, mY, mT, vX, vY, vT, th1, th2)
        self._lock = threading.Lock()

        print("Creating the publishers")
        self.publishers_init()
        print("subscribing to topics")
        self.subscribers_init()

    def new_scan_match(self, msg):
        self._lock.acquire()   ### =========

        if self.start_recording == False:
            self.scan_time = rospy.Time.now().to_sec() / 1e-6
            self.prevScanMsg = msg
            self.prevOdoMsg = self.currentOdoMsg
            self.start_recording = True
        else:
            if rospy.Time.now().to_sec() / 1e-6 > self.scan_time + 100:
                self.start_recording = False

                """ Compute increment in scan odom """
                lastScanInc = Pose2D (0.0, 0.0, 0.0)
                lastScanInc.x = msg.x - self.prevScanMsg.x
                lastScanInc.y = msg.y - self.prevScanMsg.y
                lastScanInc.theta = msg.theta - self.prevScanMsg.theta

                """ Compute increment in odometry """
                self.prevOdo = Pose2D (self.prevOdoMsg.pose.pose.position.x, self.prevOdoMsg.pose.pose.position.y, getTheta(self.prevOdoMsg.pose.pose.orientation))
                lastOdoInc = compute_inc_values (self.currentOdoMsg.pose.pose.position, self.prevOdo, getTheta(self.currentOdoMsg.pose.pose.orientation))

                """ Compute differences between both increments (in theory they should be equal) """
                x = math.fabs (lastOdoInc.x - lastScanInc.x)
                y = math.fabs (lastOdoInc.y - lastScanInc.y)
                theta = norm_angle (math.fabs (lastOdoInc.theta - lastScanInc.theta))

                if self.training:
                    self.incScansOdomDiffXs += (x,)
                    self.incScansOdomDiffYs += (y,)
                    self.incScansOdomDiffTs += (theta,)
                    self.covarGauss.train_anomaly_funcs (self.incScansOdomDiffXs, self.incScansOdomDiffYs, self.incScansOdomDiffTs)
                    print("With diff values (%f,%f,%f) incX_scan=%f  incX_odo=%f" % (x,y,theta,lastScanInc.x, lastOdoInc.x ))
                else:
                    status = self.covarGauss.is_anomalous( x, y, theta)
                    #print "SCAN MATCH SAYS ROBOT IS " + status
                    if status != "NORMAL":
                        print("\nScan match status is ", status)
                        print("With diff values (%f,%f,%f)" % (x,y,theta ))
                        print("incX_scan=%f  incX_odo=%f" % (lastScanInc.x, lastOdoInc.x ))
                        print("incY_scan=%f  incY_odo=%f" % (lastScanInc.y, lastOdoInc.y ))
                        print("incTheta_scan=%f  incTheta_odo=%f" % (lastScanInc.theta, lastOdoInc.theta ))

                """ Publish the latest increments """
                self.scan_match_pub.publish (lastScanInc)
                self.odom_pub.publish (lastOdoInc)

        self._lock.release()   ### =========

    def new_odom(self, msg):
        global lastOdomX
        global lastOdomY

        self._lock.acquire()   ### =========
        self.currentOdoMsg = msg
        status = self.status
        self._lock.release()   ### =========

        # Calculates the space moved by the robot in DOUBTFUL state
        if status == 'DOUBTFUL':
            incX = msg.pose.pose.position.x - lastOdomX
            incY = msg.pose.pose.position.y - lastOdomY
            d = math.sqrt ( math.pow(incX,2) + math.pow(incY,2))
            self.spaceMovedUnlocalized += d
            #print "Space moved as unlocalized %f" % self.spaceMovedUnlocalized

        lastOdomX = msg.pose.pose.position.x
        lastOdomY = msg.pose.pose.position.y

        # publish the status once every second
        now = rospy.get_time()
        if now - self.prevTime > 1.0:
            self.publish_marker (status)
            self.loc_st_pub.publish (String(status))
            self.prevTime = now

        #print "NEW ODOM: (%f,%f,%f)" % (msg.pose.pose.position.x, msg.pose.pose.position.y, currentTheta)
        #print "Last odometry inc = ", lastOdoInc

    def new_loc(self,msg):
        currentTheta = getTheta (msg.pose.pose.orientation)
        lastLocInc  = compute_inc_values (msg.pose.pose.position, self.prevLoc, currentTheta)

        self.loc_pub.publish (lastLocInc)

        self.prevLoc = Pose2D (msg.pose.pose.position.x, msg.pose.pose.position.y, currentTheta)

        if self.training:
            self.covarXs += (msg.pose.covariance[0],)    # covariance in X
            self.covarYs += (msg.pose.covariance[7],)    # covariance in Y
            self.covarTs += (msg.pose.covariance[35],)    # covariance in Theta
            self.covarGauss.train_anomaly_funcs(self.covarXs, self.covarYs, self.covarTs)
        else:
            status = self.covarGauss.is_anomalous( msg.pose.covariance[0], msg.pose.covariance[7], msg.pose.covariance[35])
            if status == "NORMAL":
                status = "LOCALIZED"
                self.spaceMovedUnlocalized = 0.0
            elif status == "ANOMALOUS":
                if self.spaceMovedUnlocalized > MAX_DIST_IN_LOST_STATE or msg.pose.covariance[0] > 0.4 or msg.pose.covariance[7] > 0.4: # if the space moved in 'ANOMALOUS/DOUBTFUL' state is large then we assume LOST
                    status = "LOST"
                    self.spaceMovedUnlocalized = MAX_DIST_IN_LOST_STATE + 1.0
                else:
                    status ="DOUBTFUL"

            self._lock.acquire()   ### =========
            self.status = status
            self._lock.release()   ### =========
            #print "COVAR SAYS ROBOT IS " + status

    def new_particles(self,msg):
        n = len (msg.poses)
        #print "NUMBER OF PARTICLES ============= ", n

    def subscribers_init(self):
        self.odom_subscriber = rospy.Subscriber("odom", Odometry,callback=self.new_odom, queue_size=1)

        self.scan_match_subs = rospy.Subscriber("pose2D", Pose2D,
            callback=self.new_scan_match, queue_size=1)
        self.loc_subscriber = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped,
            callback=self.new_loc, queue_size=1)
        self.part_subscriber = rospy.Subscriber("particlecloud", PoseArray,
            callback=self.new_particles, queue_size=1)

    def publishers_init(self):
        self.scan_match_pub = rospy.Publisher('/loc_measure/scanmatch_inc', Pose2D, queue_size=10)
        self.odom_pub = rospy.Publisher('/loc_measure/odom_inc', Pose2D, queue_size=10)
        self.part_pub = rospy.Publisher('/loc_measure/part_num', Pose2D, queue_size=10)
        self.loc_pub = rospy.Publisher('/loc_measure/loc_inc', Pose2D, queue_size=10)
        self.loc_st_pub = rospy.Publisher('/loc_measure/loc_status', String, queue_size=10)
        self.loc_st_marker = rospy.Publisher('/loc_measure/loc_status_marker', Marker, queue_size=10)

    def publish_marker(self, text):
        marker = Marker()
        marker.header.frame_id = "/base_link"
        marker.type = marker.TEXT_VIEW_FACING
        marker.text = text
        marker.action = marker.ADD
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 0.9
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.0
        self.loc_st_marker.publish(marker)
