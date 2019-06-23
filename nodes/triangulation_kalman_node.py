#!/usr/bin/env python

#Alex Brown
#2018

import roslib
roslib.load_manifest('triangulation_test')
import sys
import rospy
#from cv2 import cv
from fishtracker.msg import Float32Stamped
from std_msgs.msg import *
from geometry_msgs.msg import *
#from preview_filter.msg import * #this is very important! we have custom message types defined in this package!!
from sensor_msgs.msg import Image,CameraInfo,CompressedImage
from visualization_msgs.msg import Marker #we will use this message for the perceived fish. then pop it into Rviz
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from numpy import *
import math
import cv2
import tf
import rospkg

class KalmanOneFish:

    def __init__(self):
        #overall timestep of this node:
        self.dt = 0.1

        #Kalman Filter setup
        self.KF = cv2.KalmanFilter(6,3,0)
        self.KF.transitionMatrix = array([[1., self.dt,0.,0.,0.,0.], [0., 1.,0.,0.,0.,0.],[0.,0.,1.,self.dt,0.,0.],[0.,0.,0.,1.,0.,0.,],[0.,0.,0.,0.,1.,self.dt],[0.,0.,0.,0.,0.,1.]])
        self.KF.measurementMatrix = 1. * array([[1.,0.,0.,0.,0.,0.],[0.,0.,1.,0.,0.,0.],[0.,0.,0.,0.,1.,0.]])
        self.KF.processNoiseCov = 1e-3 * eye(6)
        self.KF.measurementNoiseCov = 1e-1 * eye(3)
        self.KF.errorCovPost = 1. * ones((6, 6))
        self.KF.statePost = 0.0 * random.randn(6, 1)

        #subscribe to the 3D pose of the single fish:
        self.measuresub = rospy.Subscriber("/fishtracker/measuredfishpose",PoseStamped,self.measurecallback)
        self.KFpub = rospy.Publisher("/fishtracker/kalmanfishpose",PoseStamped,queue_size=1)

        #timer for the prediction step(s)
        rospy.Timer(rospy.Duration(0.1),self.timercallback,oneshot=False)

        #initialize the measurement
        self.measurement = array([[0.],[0.],[0.]])
        self.oldmeasurement = array([[0.],[0.],[0.]])
        self.oldstamp = rospy.Time.now()
        self.newstamp = rospy.Time.now()

        self.state_estimate = array([[0],[0],[0],[0]])

    def measurecallback(self,data):
        self.measurement = array([[data.pose.position.x],[data.pose.position.y],[data.pose.position.z]])
        self.newstamp = data.header.stamp
    def timercallback(self,data):
        #prediction step
        self.KF.predict()
        self.state_estimate = self.KF.statePre
        #now check to see if we have a new measurement
        if(self.newstamp!=self.oldstamp):
            self.oldstamp=self.newstamp
            self.KF.correct(self.measurement)
            self.state_estimate = self.KF.statePost
        br = tf.TransformBroadcaster()
        br.sendTransform([self.state_estimate[0,0],self.state_estimate[2,0],self.state_estimate[4,0]],[0.,0.,0.,1],rospy.Time.now(),'/fishkalman','/world')
        KFPose = PoseStamped()
        KFPose.header.stamp = rospy.Time.now()
        KFPose.pose.position.x = self.state_estimate[0,0]
        KFPose.pose.position.y = self.state_estimate[2,0]
        KFPose.pose.position.z = self.state_estimate[4,0]
        self.KFpub.publish(KFPose)


def main(args):
  
  rospy.init_node('kalman_fish', anonymous=True)
  ic = KalmanOneFish()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)