#!/usr/bin/env python

import roslib; roslib.load_manifest('projector_select_image')
import sys
import rospy

from projector_select_image.srv import *

def show():
    print "Start."
    rospy.wait_for_service('/projector_select_image_left/ChooseImage')
    imageL = rospy.ServiceProxy('/projector_select_image_left/ChooseImage', projector_select_image)
    
    name = '1280X800_01.png'
    #name = 'black.png'
    
    clientL = imageL(name)
    
    if(clientL.success == True):
	print 'Success to change the left projector image'

    rospy.wait_for_service('/projector_select_image_right/ChooseImage')
    imageR = rospy.ServiceProxy('/projector_select_image_right/ChooseImage', projector_select_image)
    
    
    clientR = imageR(name)
    
    if(clientR.success == True):
	print 'Success to change the right projector image'
    
if __name__ == '__main__':
    show()
    print "Done!"
