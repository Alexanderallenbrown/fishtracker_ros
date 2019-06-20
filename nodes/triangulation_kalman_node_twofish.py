#!/usr/bin/env python

#Alex Brown
#2018

import roslib
roslib.load_manifest('triangulation_test')
import sys
import rospy
#from cv2 import cv
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
        self.KF1 = cv2.KalmanFilter(6,3,0)
        self.KF1.transitionMatrix = array([[1., self.dt,0.,0.,0.,0.], [0., 1.,0.,0.,0.,0.],[0.,0.,1.,self.dt,0.,0.],[0.,0.,0.,1.,0.,0.,],[0.,0.,0.,0.,1.,self.dt],[0.,0.,0.,0.,0.,1.]])
        self.KF1.measurementMatrix = 1. * array([[1.,0.,0.,0.,0.,0.],[0.,0.,1.,0.,0.,0.],[0.,0.,0.,0.,1.,0.]])
        self.KF1.processNoiseCov = 1e-3 * eye(6)
        self.KF1.measurementNoiseCov = 1e-1 * eye(3)
        self.KF1.errorCovPost = 1. * ones((6, 6))
        self.KF1.statePost = 0.1 * random.randn(6, 1)
        print "initial KF1 state: "
        print self.KF1.statePost

        self.KF2 = cv2.KalmanFilter(6,3,0)
        self.KF2.transitionMatrix = array([[1., self.dt,0.,0.,0.,0.], [0., 1.,0.,0.,0.,0.],[0.,0.,1.,self.dt,0.,0.],[0.,0.,0.,1.,0.,0.,],[0.,0.,0.,0.,1.,self.dt],[0.,0.,0.,0.,0.,1.]])
        self.KF2.measurementMatrix = 1. * array([[1.,0.,0.,0.,0.,0.],[0.,0.,1.,0.,0.,0.],[0.,0.,0.,0.,1.,0.]])
        self.KF2.processNoiseCov = 1e-3 * eye(6)
        self.KF2.measurementNoiseCov = 1e-1 * eye(3)
        self.KF2.errorCovPost = 1. * ones((6, 6))
        self.KF2.statePost = 0.1 * random.randn(6, 1)
        print "initial KF2 state: "
        print self.KF2.statePost

        #create bounds for the predictions
        self.tankminx = -26*.0254
        self.tankminy = -33*.0254
        self.tankmaxx = 0
        self.tankmaxy = 0
        self.tankminz = -.8
        self.tankmaxz = -.8+6*.0254 #GUESS


        #subscribe to the 3D pose of the single fish:
        self.measuresub1 = rospy.Subscriber("/fishtracker/measuredfishpose0",PoseStamped,self.measure1callback)
        self.measuresub2 = rospy.Subscriber("/fishtracker/measuredfishpose1",PoseStamped,self.measure2callback)

        #timer for the prediction step(s)
        rospy.Timer(rospy.Duration(0.1),self.timercallback,oneshot=False)

        #initialize the measurement
        self.measurement1 = array([[0.],[0.],[0.]])
        self.oldmeasurement1 = array([[0.],[0.],[0.]])
        self.oldstamp1 = rospy.Time.now()
        self.newstamp1 = rospy.Time.now()

        self.measurement2 = array([[0.],[0.],[0.]])
        self.oldmeasurement2 = array([[0.],[0.],[0.]])
        self.oldstamp2 = rospy.Time.now()
        self.newstamp2 = rospy.Time.now()


        self.state_estimate1 = self.KF1.statePost
        self.state_estimate2 = self.KF2.statePost

    def measure1callback(self,data):
        self.measurement1 = array([[data.pose.position.x],[data.pose.position.y],[data.pose.position.z]])
        self.newstamp1 = data.header.stamp
    def measure2callback(self,data):
        self.measurement2 = array([[data.pose.position.x],[data.pose.position.y],[data.pose.position.z]])
        self.newstamp2 = data.header.stamp
    def timercallback(self,data):
        #prediction step
        
        print "before bounds:"
        print self.KF1.statePre
        print self.KF2.statePre
        # now bound the predictions to make sure they don't go out of the tank!!
        dolims = True
        if(dolims):
            if(self.KF1.statePost[0,0]<=self.tankminx):
                # print "BOUNDING 1 X"
                self.KF1.statePost[0,0]=self.tankminx
                if(self.KF1.statePost[1,0]<0):
                    self.KF1.statePost[1,0]=0
            if(self.KF1.statePost[0,0]>=self.tankmaxx):
                # print "BOUNDING 1 X"
                self.KF1.statePost[0,0]=self.tankmaxx
                if(self.KF1.statePost[1,0])>0:
                    self.KF1.statePost[1,0]=0
            if(self.KF1.statePost[2,0]<=self.tankminy):
                # print "BOUNDING 1 Y"
                self.KF1.statePost[2,0]=self.tankminy
                if(self.KF1.statePost[3,0]<0):
                    self.KF1.statePost[3,0]=0
            if(self.KF1.statePost[2,0]>=self.tankmaxy):
                # print "BOUNDING 1 Y"
                self.KF1.statePost[2,0]=self.tankmaxy
                if(self.KF1.statePost[3,0])>0:
                    self.KF1.statePost[3,0]=0
            if(self.KF1.statePost[4,0]<=self.tankminz):
                # print "BOUNDING 1 Z"
                self.KF1.statePost[4,0]=self.tankminz
                if(self.KF1.statePost[5]<0):
                    self.KF1.statePost[5]=0
            if(self.KF1.statePost[4,0]>=self.tankmaxz):
                # print "BOUNDING 1 Z"
                self.KF1.statePost[4,0]=self.tankmaxz
                if(self.KF1.statePost[5,0])>0:
                    self.KF1.statePost[5,0]=0

            if(self.KF2.statePost[0,0]<=self.tankminx):
                # print "BOUNDING 2 X"
                self.KF2.statePost[0,0]=self.tankminx
                if(self.KF2.statePost[1,0]<0):
                    self.KF2.statePost[1,0]=0
            if(self.KF2.statePost[0,0]>=self.tankmaxx):
                # print "BOUNDING 2 X"
                self.KF2.statePost[0,0]=self.tankmaxx
                if(self.KF2.statePost[1,0])>0:
                    self.KF2.statePost[1,0]=0
            if(self.KF2.statePost[2,0]<=self.tankminy):
                # print "BOUNDING 2 Y"
                self.KF2.statePost[2,0]=self.tankminy
                if(self.KF2.statePost[3,0]<0):
                    self.KF2.statePost[3,0]=0
            if(self.KF2.statePost[2,0]>=self.tankmaxy):
                # print "BOUNDING 2 Y"
                self.KF2.statePost[2,0]=self.tankmaxy
                if(self.KF2.statePost[3,0])>0:
                    self.KF2.statePost[3,0]=0
            if(self.KF2.statePost[4,0]<=self.tankminz):
                # print "BOUNDING 2 Z"
                self.KF2.statePost[4,0]=self.tankminz
                if(self.KF2.statePost[5,0]<0):
                    self.KF2.statePost[5,0]=0
            if(self.KF2.statePost[4,0]>=self.tankmaxz):
                # print "BOUNDING 2 Z"
                self.KF2.statePost[4,0]=self.tankmaxz
                if(self.KF2.statePost[5,0])>0:
                    self.KF2.statePost[5,0]=0
        

        print "after bounds:"
        print self.KF1.statePre
        print self.KF2.statePre
        self.KF1.predict()
        self.KF2.predict()

        self.state_estimate1 = self.KF1.statePre
        self.state_estimate2 = self.KF2.statePre    
        #now check to see if we have a new measurement

        if ((self.newstamp1!=self.oldstamp1) and (self.newstamp2!=self.oldstamp2)):
            self.oldstamp1 = self.newstamp1
            self.oldstamp2 = self.newstamp2

            #which KF is this for?
            dist1 = sqrt((self.measurement1[0]-self.state_estimate1[0])**2+(self.measurement1[1]-self.state_estimate1[2])**2+(self.measurement1[2]-self.state_estimate1[4])**2)
            dist2 = sqrt((self.measurement1[0]-self.state_estimate2[0])**2+(self.measurement1[1]-self.state_estimate2[2])**2+(self.measurement1[2]-self.state_estimate2[4])**2)
            if(dist1<=dist2):
                    print "correcting 1 and 2"
                    self.KF1.correct(self.measurement1)
                    self.KF2.correct(self.measurement2)
                    self.state_estimate1 = self.KF1.statePost
                    self.state_estimate2 = self.KF2.statePost
            else:
                print "correcting 1 and 2"
                self.KF2.correct(self.measurement1)
                self.KF1.correct(self.measurement2)
                self.state_estimate2=self.KF2.statePost
                self.state_estimate1=self.KF1.statePost

        else:
            if(self.newstamp1!=self.oldstamp1):
                self.oldstamp1=self.newstamp1
                #which KF is this for?
                dist1 = sqrt((self.measurement1[0]-self.state_estimate1[0])**2+(self.measurement1[1]-self.state_estimate1[2])**2+(self.measurement1[2]-self.state_estimate1[4])**2)
                dist2 = sqrt((self.measurement1[0]-self.state_estimate2[0])**2+(self.measurement1[1]-self.state_estimate2[2])**2+(self.measurement1[2]-self.state_estimate2[4])**2)

                if(dist1<=dist2):
                    print "correcting 1"
                    self.KF1.correct(self.measurement1)
                    self.state_estimate1 = self.KF1.statePost
                else:
                    print "correcting 2"
                    self.KF2.correct(self.measurement1)
                    self.state_estimate2=self.KF2.statePost

            if(self.newstamp2!=self.oldstamp2):
                self.oldstamp2=self.newstamp2
                #which KF is this for?
                dist1 = sqrt((self.measurement2[0]-self.state_estimate1[0])**2+(self.measurement2[1]-self.state_estimate1[2])**2+(self.measurement2[2]-self.state_estimate1[4])**2)
                dist2 = sqrt((self.measurement2[0]-self.state_estimate2[0])**2+(self.measurement2[1]-self.state_estimate2[2])**2+(self.measurement2[2]-self.state_estimate2[4])**2)

                if(dist1<=dist2):
                    print "correcting 1"
                    self.KF1.correct(self.measurement2)
                    self.state_estimate1 = self.KF1.statePost
                else:
                    print "correcting 2"
                    self.KF2.correct(self.measurement2)
                    self.state_estimate2=self.KF2.statePost
        
        br = tf.TransformBroadcaster()
        br.sendTransform([self.state_estimate1[0,0],self.state_estimate1[2,0],self.state_estimate1[4,0]],[0.,0.,0.,1],rospy.Time.now(),'/fishkalman1','/world')
        br.sendTransform([self.state_estimate2[0,0],self.state_estimate2[2,0],self.state_estimate2[4,0]],[0.,0.,0.,1],rospy.Time.now(),'/fishkalman2','/world')



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