#!/usr/bin/env python

#Alex Brown
#2018

import roslib
roslib.load_manifest('fishtracker')
import sys
import rospy
#from cv2 import cv
from std_msgs.msg import *
from geometry_msgs.msg import *
#from preview_filter.msg import * #this is very important! we have custom message types defined in this package!!
from sensor_msgs.msg import Image, CompressedImage
#import tetherish message so that this node knows where the robot is
from tetherfish_ros.msg import TetherFishMsg

from visualization_msgs.msg import Marker #we will use this message for the perceived fish. then pop it into Rviz
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from numpy import *
import math
import cv2
import tf
import rospkg

class measure_fish:

    def __init__(self):
        #self.D = np.array([-0.40541413163196455, 0.09621547958919903, 0.029070017586547533, 0.005280797822816339, 0.0])
        #self.K = np.array([[529.8714858851022, 0.0, 836.4563887311622], [0.0, 1547.2605077363528, 83.19276259345895], [0.0, 0.0, 1.0]])
        # get the file path for rospy_tutorials
        self.cam_pos = (0,0,18*.0254)
        self.cam_quat = tf.transformations.quaternion_from_euler(pi,0,0)
        self.robotsize = [140,65]
        self.robotOffsetY = 40;
        self.robotOffsetX = 100;
        self.robotPixelX = 0
        self.robotPixelY = 0
        self.robotScale = 16.75
        self.robotX = 0

        rospack = rospkg.RosPack()
        self.package_path=rospack.get_path('fishtracker')

        #this is how we get our image in to use openCV
        self.cascade = cv2.CascadeClassifier(self.package_path+'/cascade/cascade_mar232018.xml')#'package://fishtracker/meshes/fishbody.dae'
        self.top_crop = rospy.get_param('top_crop',100)
        self.bottom_crop = rospy.get_param('bottom_crop',150)
        self.bridge = CvBridge()
        self.fishmarkerpub = rospy.Publisher('/measured_fishmarker',Marker,queue_size=1)
        self.image_pub = rospy.Publisher('/fishtracker/overlay_image',Image,queue_size=1)
        self.fishlist = array([])
        self.robotlist = array([])
        self.robotPixelPub = rospy.Publisher('/fishtracker/robot_pixel_pos',Int32MultiArray,queue_size=1)
        self.fishPosPub = rospy.Publisher('/fishtracker/fish_positions',Int32MultiArray,queue_size=1)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw/compressed",CompressedImage,self.callback,queue_size=1)#change this to proper name!
        self.robot_sub = rospy.Subscriber("/tetherfish/robotinfo",TetherFishMsg,self.robotCallback)

        self.timenow = rospy.Time.now()
        self.imscale = 1.0





        

        
    def robotCallback(self,data):
        #print "updating robot pos"
        self.robotX = data.robotpos
        self.robotPixelY = int(self.robotOffsetY)
        self.robotPixelX = int(self.robotX*self.robotScale+self.robotOffsetX)
        #self.robotlist = [self.robotPixelX,self.robotPixelY]

    def detect(self,img):
        #print cascade
        #rejectLevels??
        #maxSize=(200,100),
        # rects = cascade.detectMultiScale(img, scaleFactor=1.6, minNeighbors=24,  minSize=(20,20),maxSize=(200,100),flags=cv2.CASCADE_SCALE_IMAGE)
        # rects = self.cascade.detectMultiScale(img, scaleFactor=1.6, minNeighbors=7,  minSize=(20,20),maxSize=(200,100),flags=cv2.CASCADE_SCALE_IMAGE)
        rects = self.cascade.detectMultiScale(img, scaleFactor=1.01, minNeighbors=5,  minSize=(1,1),maxSize=(50,80),flags=cv2.CASCADE_SCALE_IMAGE)

        if len(rects) == 0:
            return [], img
        rects2=self.cleanRects(rects)
        rects2[:, 2:] += rects2[:, :2]

        return rects2, img

    def cleanRects(self,rects):
        #gets rid of any rects that are fully contained within another
        rects = array(rects)
        rectsout=rects.copy()
        badrects = array([],dtype=int32)
        for rectnum in range(0,len(rects[:,0])):
            #what rect are we looking at?
            rect_tlx,rect_tly,rect_brx,rect_bry = rects[rectnum,0],rects[rectnum,1],rects[rectnum,0]+rects[rectnum,2],rects[rectnum,1]+rects[rectnum,3]
            #now see if any others are contained within it
            for testnum in range(0,len(rects[:,0])):
                testrect_tlx,testrect_tly,testrect_brx,testrect_bry = rects[testnum,0],rects[testnum,1],rects[testnum,0]+rects[testnum,2],rects[testnum,1]+rects[testnum,3]
                if ((rect_tlx-testrect_tlx)<0 and (rect_tly-testrect_tly)<0):
                    #this means that the TL corner is inside the rect
                    if ((rect_brx-testrect_brx)>0 and (rect_bry-testrect_bry)>0):
                        #this means that testrect is fully enclosed in rect, so delete it
                        badrects = append(badrects,testnum)
                        print "found bad rect at index "+str(testnum)+" of "+str(len(rects[:,0]))
        rectsout=delete(rectsout,badrects,0)
        return rectsout

    def box(self,rects, img):
        
        for x1, y1, x2, y2 in rects:
            if((abs(x2-x1)>self.robotsize[1]) and (abs(y2-y1)>self.robotsize[0])):
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 255), 2)
                
            else:
                cv2.rectangle(img, (x1, y1), (x2, y2), (127, 255, 0), 2)
        #cv2.imwrite('one.jpg', img);

    def fillFishPositions(self,rects,img):
        self.fishlist = []
        self.robotlist = [self.robotPixelX,self.robotPixelY]
        for x1,y1,x2,y2 in rects:
            cx = int((x1+x2)/2.0)
            cy = int((y1+y2)/2.0)
            if ((cx>(self.robotPixelX-self.robotsize[0]/2)) and (cx<self.robotPixelX+self.robotsize[0]/2) and (cy<self.robotPixelX-self.robotsize[0]/2) and (cy<(self.robotPixelY+self.robotsize[1]/2))) :
                #print "found rect inside robot"
                pass
            else:
                self.fishlist.append(int(cx))
                self.fishlist.append(int(cy))
                cv2.circle(img,(cx,cy),10,(0,255,0),2)

        cv2.circle(img,(int(self.robotPixelX),int(self.robotPixelY)),10,(0,0,255),2)

        fishpos_msg = Int32MultiArray(data=self.fishlist)
        #fishpos_msg.header.stamp = rospy.Time.now()
        robotpos_msg = Int32MultiArray(data=self.robotlist)
        #robotpos_msg.header.stamp = rospy.Time.now()
        self.fishPosPub.publish(fishpos_msg)
        self.robotPixelPub.publish(robotpos_msg)




    def boxBW(self,rects, img):
        for x1, y1, x2, y2 in rects:
            cv2.rectangle(img, (x1, y1), (x2, y2), ( 255,255,255), 2)
        #cv2.imwrite('one.jpg', img);

    def drawRobot(self,img):
        x1 = self.robotPixelX - self.robotsize[0]/2
        x2 = self.robotPixelX + self.robotsize[0]/2
        y1 = self.robotPixelY-self.robotsize[1]/2
        y2 = self.robotPixelY+self.robotsize[1]/2
        cv2.rectangle(img,(x1,y1),(x2,y2),(0,0,255),2)

  #this function fires whenever a new image_raw is available. it is our "main loop"
    def callback(self,data):
        try:
            np_arr = np.fromstring(data.data, np.uint8)
            # Decode to cv2 image and store
            frame= cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            he,wi,de = frame.shape
            frame = frame[self.top_crop:he-self.bottom_crop,:]
            frame_orig = frame
            frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            #frame = cv2.resize(frame,None,fx=self.imscale, fy=self.imscale, interpolation = cv2.INTER_CUBIC)
        
        except CvBridgeError, e:
            print e
        self.timenow = rospy.Time.now()
        rows,cols = frame.shape
        if rows>0:
            rects,frame = self.detect(frame)
            #if rects is not None:
            #    rects = rects.sort()
            #self.box(rects, frame_orig)
            #self.drawRobot(frame_orig)
            self.fillFishPositions(rects,frame_orig)

        img_out = self.bridge.cv2_to_imgmsg(frame_orig, "bgr8")
        img_out.header.stamp = rospy.Time.now()
        try:
            self.image_pub.publish(img_out)
        except CvBridgeError as e:
            print(e)

            # fishquat = tf.transformations.quaternion_from_euler(rvecs[0][0][0],rvecs[0][0][1],rvecs[0][0][2])
            # br = tf.TransformBroadcaster()
            # br.sendTransform((tvecs[0][0][0],tvecs[0][0][1],tvecs[0][0][2]),fishquat,self.timenow,'/fish_measured','/camera1')
            # br.sendTransform(self.cam_pos,self.cam_quat,self.timenow,'/camera1','world')
            # #publish a marker representing the fish body position
            # fishmarker = Marker()
            # fishmarker.header.frame_id='/fish_measured'
            # fishmarker.header.stamp = self.timenow
            # fishmarker.type = fishmarker.MESH_RESOURCE
            # fishmarker.mesh_resource = 'package://fishtracker/meshes/fishbody.dae'
            # fishmarker.mesh_use_embedded_materials = True
            # fishmarker.action = fishmarker.MODIFY
            # fishmarker.scale.x = 1
            # fishmarker.scale.y = 1
            # fishmarker.scale.z = 1
            # tempquat = tf.transformations.quaternion_from_euler(0,0,0)#this is RELATIVE TO FISH ORIENTATION IN TF (does the mesh have a rotation?)
            # fishmarker.pose.orientation.w = tempquat[3]
            # fishmarker.pose.orientation.x = tempquat[0]
            # fishmarker.pose.orientation.y = tempquat[1]
            # fishmarker.pose.orientation.z = tempquat[2]
            # fishmarker.pose.position.x = 0
            # fishmarker.pose.position.y = 0
            # fishmarker.pose.position.z = 0
            # fishmarker.color.r = .8
            # fishmarker.color.g = .5
            # fishmarker.color.b = .5
            # fishmarker.color.a = 1.0#transparency
            # self.fishmarkerpub.publish(fishmarker)

def main(args):
  
  rospy.init_node('measure_fish', anonymous=True)
  ic = measure_fish()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
