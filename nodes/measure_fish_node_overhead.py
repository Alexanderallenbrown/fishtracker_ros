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
from skimage import data, color, img_as_ubyte
from skimage.feature import canny
from skimage.transform import hough_ellipse
from skimage.draw import ellipse_perimeter

class measure_fish:

    def __init__(self):
        #self.D = np.array([-0.40541413163196455, 0.09621547958919903, 0.029070017586547533, 0.005280797822816339, 0.0])
        #self.K = np.array([[529.8714858851022, 0.0, 836.4563887311622], [0.0, 1547.2605077363528, 83.19276259345895], [0.0, 0.0, 1.0]])

        #this is how we get our image in to use openCV
        self.top_crop = rospy.get_param('top_crop',0)
        self.bottom_crop = rospy.get_param('bottom_crop',0)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera1/usb_cam1/image_raw/compressed",CompressedImage,self.callback,queue_size=1)#change this to proper name!
        self.fishmarkerpub = rospy.Publisher('/measured_fishmarker',Marker,queue_size=1)
        self.image_pub = rospy.Publisher('/fishtracker/overlay_image_overhead',Image,queue_size=1)
        self.timenow = rospy.Time.now()
        self.imscale = 1.0

        self.cam_pos = (0,0,18*.0254)
        self.cam_quat = tf.transformations.quaternion_from_euler(pi,0,0)
        self.fgbg = cv2.createBackgroundSubtractorMOG2()
        self.fgbg.setDetectShadows(True)
        self.fgbg.setShadowValue(0)
        

        self.minw = 50
        self.maxw = 250
        self.minh = 50
        self.maxh = 250
        self.kernel = np.ones((9,9),np.uint8)
        rospack = rospkg.RosPack()
        # get the file path for rospy_tutorials
        self.package_path=rospack.get_path('fishtracker')


    def cleanRects(self,rects):
        #gets rid of any rects that are fully contained within another
        rects = array(rects)
        rectsout=rects.copy()
        badrects = array([],dtype=int32)
        for rectnum in range(0,len(rects[:,0])):
            #what rect are we looking at?
            rect_tlx,rect_tly,rect_brx,rect_bry = rects[rectnum,0],rects[rectnum,1],rects[rectnum,0]+rects[rectnum,2],rects[rectnum,1]+rects[rectnum,3]
            #now see if any others are contained within it
            for testnum in range(1,len(rects[:,0])):
                testrect_tlx,testrect_tly,testrect_brx,testrect_bry = rects[testnum,0],rects[testnum,1],rects[testnum,0]+rects[testnum,2],rects[testnum,1]+rects[testnum,3]
                if ((rect_tlx-testrect_tlx)<=0 and (rect_tly-testrect_tly)<=0):
                    #this means that the TL corner is inside the rect
                    if ((rect_brx-testrect_brx)>=0 and (rect_bry-testrect_bry)>=0):
                        #this means that testrect is fully enclosed in rect, so delete it
                        badrects = append(badrects,testnum)
                        #print "found bad rect at index "+str(testnum)+" of "+str(len(rects[:,0]))
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
            # frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            # frame = cv2.resize(frame,None,fx=self.imscale, fy=self.imscale, interpolation = cv2.INTER_CUBIC)

        except CvBridgeError, e:
            print e
        
        self.timenow = rospy.Time.now()
        rows,cols,depth = frame.shape

        rects = None
        if rows>0:
            
            gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            fgmask = self.fgbg.apply(gray)
            canny = cv2.Canny(fgmask,100,200)
            canny = cv2.dilate(canny,self.kernel,iterations=1)
            cannycolor = cv2.cvtColor(canny,cv2.COLOR_GRAY2BGR)

            im,contours,hier = cv2.findContours(canny.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)          
            
            if(len(contours)>0):
                cv2.drawContours(cannycolor,contours,-1,(0,255,0),5)
                for k in range(0,len(contours)):
                    cnt = contours[k]
                    #print cnt.shape
                    #print cnt
                    x,y,w,h = cv2.boundingRect(array(cnt))
                    #print x,y,w,h
                    if ((w<self.maxw) and (w>self.minw) and (h<self.maxh) and (h>self.minh)):
                        #print k,x,y,w,h
                        #frame = cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),2)
                        if rects is not None:
                            rects = np.vstack((rects,np.array([x,y,w+x,h+y])))
                            #print rects
                        else:
                            rects = np.array([[x,y,w+x,h+y]])
            
            #print rects
            if rects is not None:
                rectsout = self.cleanRects(rects)
                #print rects.shape
                self.box(rectsout,frame)
            #cv2.imshow('frame',frame)
            #cv2.imshow('canny',cannycolor)

            #cv2.waitKey(1)
            #rects,frame = self.detect(frame)
            #self.box(rects, frame_orig)
        img_out = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        img_out.header.stamp = rospy.Time.now()
        try:
            self.image_pub.publish(img_out)
        except CvBridgeError as e:
            print(e)


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
