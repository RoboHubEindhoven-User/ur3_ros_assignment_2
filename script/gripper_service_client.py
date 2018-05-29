#! /usr/bin/env python

import rospy
from faraday_gripper.srv import *

def gripper_client(size_to_open):
    rospy.init_node('gripper_service_client')
    rospy.wait_for_service('gripper_control')
    try:
        gripper = rospy.ServiceProxy('gripper_control', GripperControl)
        gripper_control = gripper(size_to_open)
        print "Send state: ", gripper_control.completed

    except rospy.ServiceException, e:
        print "Service call failed: %s" %e


if __name__ == "__main__":

    print 'Client node ready'
    user_input = raw_input('Please give in the size to open (0mm - 185mm). ')
    size_to_open = float(int(user_input)/10)
    print "Requesting %s cm" %size_to_open
    print gripper_client(size_to_open)