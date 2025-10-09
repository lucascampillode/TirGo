#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import rospy
import sys
from pal_loc_measure import Listener


if __name__ == '__main__':

    rospy.init_node('loc_measurer', anonymous=True)

    """ Parse arguments """
    print("Parsing arguments")
    if sys.argv[1] == 'True' or sys.argv[1] == 'true':
        training = True
    else:
        training = False

    print("Creating the listener")
    lst = Listener(training)

    print("Entering the loop")
    while not rospy.is_shutdown():
        rospy.sleep(1.0)

    del lst
