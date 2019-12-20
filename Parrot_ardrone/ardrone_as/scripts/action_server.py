#! /usr/bin/python

import rospy

import actionlib

from ardrone_as.msg import ArdroneFeedback, ArdroneResult, ArdroneAction
from sensor_msgs.msg import CompressedImage


class ArdroneAS(object):
    _feedback = ArdroneFeedback()
    _result = ArdroneResult()
    _lastImage = CompressedImage()
    
    def __init__(self):
        
        # init the action server
        self._as = actionlib.SimpleActionServer("~/ardrone_action_server", ArdroneAction, self.arCallback, False)
        self._as.start()
        
        # connect to the drone front camera
        self._camera = rospy.Subscriber("/drone/front_camera/image_raw/compressed", CompressedImage, self.cameraCallback)
        
        self._result.allPictures = []
        
    def cameraCallback(self, msg):
        
        self._lastImage = msg
        
        
    def arCallback(self, goal):
        r = rospy.Rate(1)
        
        success = True
        
        for i in xrange(1, goal.nseconds):
            
            # check if there are a preemption request
            if self._as.is_preempt_requested():
                rospy.loginfo('Cancelling image taking action server')
                self._as.set_preempted()
                success = False
                break
            
            self._feedback.lastImage = self._lastImage
            self._result.allPictures.append(self._lastImage)
            
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            
            r.sleep()
            
        if success:
            rospy.loginfo('Finishing.All the images have been taken')
            self._as.set_succeeded(self._result)
            
if __name__ == '__main__':
    
    rospy.init_node('ardrone_action_server')
    ArdroneAS()
    rospy.spin()
            