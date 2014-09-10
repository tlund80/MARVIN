#!/usr/bin/env python

import roslib; roslib.load_manifest('calibration_verify')
import sys
import rospy

from calibration_verify.srv import *

def pose_client():
    print "It is here starts."
    rospy.wait_for_service('/kinect_chessboard_detector/getcorner')
    pose = rospy.ServiceProxy('/kinect_chessboard_detector/getcorner', chessboard_detector)
    re = pose(9,6)
    print "Left top corner P:", re.left_top
    print "Left top corner P:", re.right_top
    print "Left top corner P:", re.left_bottom
    print "Left top corner P:", re.right_bottom
    
    
if __name__ == "__main__":
    pose_client()
    print "Done!"