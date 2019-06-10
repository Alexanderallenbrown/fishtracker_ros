#!/usr/bin/env python

#Alex Brown
#2019

import roslib
roslib.load_manifest('fishtracker')
import sys
import rospy
#from cv2 import cv
from std_msgs.msg import *
from geometry_msgs.msg import *
#from preview_filter.msg import * #this is very important! we have custom message types defined in this package!!
from sensor_msgs.msg import Image,CameraInfo
from visualization_msgs.msg import Marker #we will use this message for the perceived fish. then pop it into Rviz
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from numpy import *
import math
import cv2
import tf
import rospkg


def generateCamMats(x,y,z,roll,pitch,yaw,fx,fy,cx,cy):
    # x,y,z = 0.5,-2,0
    # roll,pitch,yaw = 0,pi/2,pi/2
    K = array([[fx,0,cx,0],[0,fy,cy,0],[0,0,1,0],[0,0,0,1]])
    Rx = array([[1,0,0,0],[0,cos(roll),sin(roll),0],[0,-sin(roll),cos(roll),0],[0,0,0,1]])
    Ry = array([[cos(pitch),0,-sin(pitch),0],[0,1,0,0],[sin(pitch),0,cos(pitch),0],[0,0,0,1]])
    Rz = array([[cos(yaw),sin(yaw),0,0],[-sin(yaw),cos(yaw),0,0],[0,0,1,0],[0,0,0,1]])
    R = dot(Rx,dot(Ry,Rz))
    T = array([[1,0,0,x],[0,1,0,y],[0,0,1,z],[0,0,0,1]])

    return K,R,T


class SimCam:
    def __init__(self,K,R,T,w,h):
        self.K=K
        self.R=R
        self.T=T
        self.E = dot(self.R,self.T)
        self.w=w
        self.h=h
        self.P=dot(self.K,self.E)
        #print self.P
    def renderSphere(self,position,radius,depth):
        im = zeros((self.h,self.w,3),dtype=uint8)

        #position will come in as a list or array
        pos = vstack(position)
        pos = vstack((pos,1))
        centroid = dot(self.P,pos)
        #print centroid[0]/centroid[2],centroid[1]/centroid[2]
        #real copout on the radius... but for now... TODO
        prad = self.K[0,0]*abs(radius/(centroid[2]+.000001))
        #draw our sphere at the correct location
        # cv2.circle(im,(int(centroid[0]),int(centroid[1])), int(prad), (0,0,255), -1)
        cv2.circle(im,(int(centroid[0]/centroid[2]),int(centroid[1]/centroid[2])), prad, (0,0,255), -1)

        return im

class Cams:
    def __init__(self):
        #images 720 rows 1280 columns

        #first camera info: side view
        x1,y1,z1 = -1.54,-.45,-0.79
        tx1,ty1,tz1 = x1,y1,z1
        roll1,pitch1,yaw1 = -pi/2,0,-pi/2
        fx1,fy1,cx1,cy1 = 1018.49,1018.05,634.54,382.02
        self.K1,self.R1,self.T1= generateCamMats(tx1,ty1,tz1,roll1,pitch1,yaw1,fx1,fy1,cx1,cy1)
        self.pos1 = [x1,y1,z1]
        self.quat1 = tf.transformations.quaternion_from_euler(roll1,pitch1,yaw1)
        self.simcam1 = SimCam(self.K1,self.R1,self.T1,1280,720)

        #second camera info: Top view
        x2,y2,z2 = -.38,-.48,-.05
        tx2,ty2,tz2 = x2,y2,z2
        roll2,pitch2,yaw2 = -pi,0,-pi/2
        fx2,fy2,cx2,cy2 = 1103.32,1108.68,591.74,313
        self.K2,self.R2,self.T2= generateCamMats(tx2,ty2,tz2,roll2,pitch2,yaw2,fx2,fy2,cx2,cy2)
        self.pos2 = [x2,y2,z2]
        self.quat2 = tf.transformations.quaternion_from_euler(roll2,pitch2,yaw2)
        self.simcam2 = SimCam(self.K2,self.R2,self.T2,1280,720)

        #timed loop that will generate two synced images
        rospy.Timer(rospy.Duration(0.033),self.camcallback,oneshot=False)

        #transform listener
        self.listener = tf.TransformListener()
        self.bridge = CvBridge()

        #set up image publishers for synthetic images
        # self.im1_pub = rospy.Publisher("/cam1/image_raw",Image,queue_size=5)
        # self.im2_pub = rospy.Publisher("/cam2/image_raw",Image,queue_size=5)
        self.cam2infopub = rospy.Publisher("/camera1/usb_cam1/camera_info",CameraInfo,queue_size=5)
        self.cam1infopub = rospy.Publisher("/camera2/usb_cam2/camera_info",CameraInfo,queue_size=5)
        rospack = rospkg.RosPack()
        self.package_path=rospack.get_path('fishtracker')
        self.meshpath = self.package_path+'/meshes/camera_rig.dae'
        #tank mesh marker publisher
        self.tankpub = rospy.Publisher('/tankrig',Marker,queue_size=1)


    def camcallback(self,data):
        now = rospy.Time.now()

        #send transforms to show each camera's coordinate system
        br = tf.TransformBroadcaster()
        br.sendTransform(self.pos1,self.quat1,now,'/camera2','world')
        br.sendTransform(self.pos2,self.quat2,now,'/camera1','world')

        #now prep the camera info msgs
        CI1 = CameraInfo()
        CI2 = CameraInfo()

        CI1.header.stamp = rospy.Time.now()
        CI1.header.frame_id = '/camera2'
        CI1.height=self.simcam1.h
        CI1.width = self.simcam1.w
        CI1.K = self.simcam1.K[0:3,0:3].reshape(1,9)[0]
        CI1.R = self.simcam1.R[0:3,0:3].reshape(1,9)[0]
        CI1.P = self.simcam1.P[0:3,:].reshape(1,12)[0]

        CI2.header.stamp = rospy.Time.now()
        CI2.header.frame_id = '/camera1'
        CI2.height=self.simcam2.h
        CI2.width = self.simcam2.w
        CI2.K = self.simcam2.K[0:3,0:3].reshape(1,9)[0]
        CI2.R = self.simcam2.R[0:3,0:3].reshape(1,9)[0]
        CI2.P = self.simcam2.P[0:3,:].reshape(1,12)[0]

        self.cam1infopub.publish(CI2)
        self.cam2infopub.publish(CI1)

        #marker showing tank
        #publish a marker representing the fish body position
        tankmarker = Marker()
        tankmarker.header.frame_id='/world'
        tankmarker.header.stamp = rospy.Time.now()
        tankmarker.type = tankmarker.MESH_RESOURCE
        tankmarker.mesh_resource = 'package://fishtracker/meshes/camera_rig.dae'
        tankmarker.mesh_use_embedded_materials = True
        tankmarker.action = tankmarker.MODIFY
        tankmarker.scale.x = .0254
        tankmarker.scale.y = .0254
        tankmarker.scale.z = .0254
        tempquat = tf.transformations.quaternion_from_euler(0,0,0)#this is RELATIVE TO FISH ORIENTATION IN TF (does the mesh have a rotation?)
        tankmarker.pose.orientation.w = tempquat[3]
        tankmarker.pose.orientation.x = tempquat[0]
        tankmarker.pose.orientation.y = tempquat[1]
        tankmarker.pose.orientation.z = tempquat[2]
        tankmarker.pose.position.x = 0
        tankmarker.pose.position.y = 0
        tankmarker.pose.position.z = 0
        # tankmarker.color.r = .5
        # tankmarker.color.g = .8
        # tankmarker.color.b = .5
        tankmarker.color.a = 0.3#transparency
        self.tankpub.publish(tankmarker)

def main(args):
  
  rospy.init_node('cam_info_publisher', anonymous=True)
  cameras = Cams()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  #cv2.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)