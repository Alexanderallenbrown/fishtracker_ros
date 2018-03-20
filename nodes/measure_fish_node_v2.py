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

        #this is how we get our image in to use openCV
        self.top_crop = rospy.get_param('top_crop',100)
        self.bottom_crop = rospy.get_param('bottom_crop',150)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw/compressed",CompressedImage,self.callback,queue_size=1)#change this to proper name!
        self.fishmarkerpub = rospy.Publisher('/measured_fishmarker',Marker,queue_size=1)
        self.image_pub = rospy.Publisher('/fishtracker/overlay_image',Image,queue_size=1)
        self.timenow = rospy.Time.now()
        self.imscale = 1.0

        self.cam_pos = (0,0,18*.0254)
        self.cam_quat = tf.transformations.quaternion_from_euler(pi,0,0)
        self.fgbg = cv2.createBackgroundSubtractorMOG2()
        self.minw = 10
        self.maxw = 100
        self.minh = 10
        self.maxh = 100
        self.kernel = np.ones((401,401),np.uint8)
        rospack = rospkg.RosPack()
        # get the file path for rospy_tutorials
        self.package_path=rospack.get_path('fishtracker')

        self.cascade = cv2.CascadeClassifier(self.package_path+'/cascade/fish_sideview_1.xml')#'package://fishtracker/meshes/fishbody.dae'

    def detect(self,img):
        #print cascade
        #rejectLevels??
        #maxSize=(200,100),
        # rects = cascade.detectMultiScale(img, scaleFactor=1.6, minNeighbors=24,  minSize=(20,20),maxSize=(200,100),flags=cv2.CASCADE_SCALE_IMAGE)
        rects = self.cascade.detectMultiScale(img, scaleFactor=1.6, minNeighbors=7,  minSize=(20,20),maxSize=(200,100),flags=cv2.CASCADE_SCALE_IMAGE)
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

    def cleanRects2(self,rects):
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
        print rects.shape
        for x1, y1, x2, y2 in rects:
            cv2.rectangle(img, (x1, y1), (x2, y2), (127, 255, 0), 2)
        #cv2.imwrite('one.jpg', img);

    def boxBW(self,rects, img):
        for x1, y1, x2, y2 in rects:
            cv2.rectangle(img, (x1, y1), (x2, y2), ( 255,255,255), 2)
        #cv2.imwrite('one.jpg', img);

  #this function fires whenever a new image_raw is available. it is our "main loop"
    def callback(self,data):
        #print "in callback"
        try:
            #use the np_arr thing if subscribing to compressed image
            #frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            np_arr = np.fromstring(data.data, np.uint8)
            # Decode to cv2 image and store
            frame= cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            he,wi,de = frame.shape
            frame = frame[self.top_crop:he-self.bottom_crop,:]
            frame_orig = frame

        except CvBridgeError, e:
            print e
        
        self.timenow = rospy.Time.now()
        rows,cols,depth = frame.shape

        rects = None
        if rows>0:
            #fgmask = self.fgbg.apply(frame)
            gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            gray = cv2.bilateralFilter(gray, 11, 17, 17)
            fgmask = self.fgbg.apply(gray)
            
            #cv2.dilate(fgmask,self.kernel,iterations=1)
            cv2.imshow('pre canny',fgmask)
            #fgmask = cv2.Canny(fgmask, 30, 200)
            #fgmask = cv2.Canny(fgmask, 30, 200)
            #http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contour_features/py_contour_features.html
            #http://opencvpython.blogspot.com/2012/06/hi-this-article-is-tutorial-which-try.html
            im,contours,hier = cv2.findContours(fgmask.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            #contours = sorted(contours, key=cv2.contourArea,reverse=True)[:10]
            #print "countour shape"
            #contours = contours[0]
            
            #cv2.drawContours(frame,np.array(contours[0]),-1,(0,255,0),5)
            if(len(contours)>0):
                for k in range(0,len(contours)):
                    cnt = contours[k]
                    #print cnt.shape
                    #print cnt
                    x,y,w,h = cv2.boundingRect(array(cnt))
                    if ((w<self.maxw) and (w>self.minw) and (h<self.maxh) and (h>self.minh)):
                        #print k,x,y,w,h
                        #frame = cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),2)
                        if rects is not None:
                            rects = np.vstack((rects,np.array([x,y,w+x,h+y])))
                            #print rects
                        else:
                            rects = np.array([[x,y,w+x,h+y]])
            
            if rects is not None:
                rectsout = self.cleanRects(rects)
                #print rects.shape
                self.box(rectsout,frame)
            cv2.imshow('frame',frame)
            cv2.imshow('mask',fgmask)

            cv2.waitKey(1)
            #rects,frame = self.detect(frame)
            #self.box(rects, frame_orig)
        img_out = self.bridge.cv2_to_imgmsg(frame, "bgr8")
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
