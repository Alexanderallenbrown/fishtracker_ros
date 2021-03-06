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
import os


class measure_fish:

    def __init__(self):
        #self.D = np.array([-0.40541413163196455, 0.09621547958919903, 0.029070017586547533, 0.005280797822816339, 0.0])
        #self.K = np.array([[529.8714858851022, 0.0, 836.4563887311622], [0.0, 1547.2605077363528, 83.19276259345895], [0.0, 0.0, 1.0]])
        self.detectedangle=0
        self.kernel = np.ones((9,9),np.uint8)
        self.fgbg = cv2.createBackgroundSubtractorMOG2()
        self.fgbg.setDetectShadows(True)
        self.fgbg.setShadowValue(0)
        #this is how we get our image in to use openCV
        self.manx = None
        self.many = None
        direc = rospy.get_param('output_dir','')
        bagname = rospy.get_param('bag','')
        self.fishnum = str(rospy.get_param('fishnum','1'))
        self.bagchop = bagname[0:-4]
        self.image_directory = direc+'/'+self.bagchop+'/fish_overhead/'
        if not os.path.exists(self.image_directory):
            os.makedirs(self.image_directory)

        self.file_path = direc+'/'+self.bagchop+'/'+self.bagchop+'_'+'output_fish'+self.fishnum+'overhead.txt'
        self.anglefile_path=direc+'/'+self.bagchop+'/'+self.bagchop+'_'+'output_fishangle'+self.fishnum+'.txt'
        
        print self.file_path
        
        self.f = open(self.file_path,'wb')
        self.anglef = open(self.anglefile_path,'wb')
        self.top_crop = rospy.get_param('top_crop',0)
        self.bottom_crop = rospy.get_param('bottom_crop',0)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera1/usb_cam1/image_raw/compressed",CompressedImage,self.callback,queue_size=1)#change this to proper name!
        self.timenow = rospy.Time.now()
        self.imscale = 1.0

        rospack = rospkg.RosPack()
        # get the file path for rospy_tutorials
        self.package_path=rospack.get_path('fishtracker')
        self.manrect = 200#size for manual square patch for image saving
        self.mandia = 15#size for the manually encoded fish center
        #update the current fish position
        
        self.drawing = False
        # cv2.namedWindow('fish')
        # cv2.waitKey(10)
        # cv2.setMouseCallback('fish',self.update_fish)
        self.frame = None




  #this function fires whenever a new image_raw is available. it is our "main loop"
    def callback(self,data):
        cv2.namedWindow('fish')
        cv2.setMouseCallback('fish',self.update_fish)
        #print "in callback"
        try:
            #use the np_arr thing if subscribing to compressed image
            #frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            np_arr = np.fromstring(data.data, np.uint8)
            # Decode to cv2 image and store
            frame= cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            he,wi,de = frame.shape

            frame = frame[self.top_crop:he-self.bottom_crop,:]
            frame_orig = frame.copy()

        except CvBridgeError, e:
            print e
        
        timenow = data.header.stamp
        now = timenow.secs+timenow.nsecs/1.0000e9
        rows,cols,depth = frame.shape
        #print now,self.manx,self.many

        rects = None
        if rows>0:
            if self.manx is not None:
                # pass
                
                
                gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
                fgmask = self.fgbg.apply(gray)
                if (self.manx-self.manrect/2)<0:
                    self.manx=self.manrect/2
                if (self.manx+self.manrect/2)>cols:
                    self.manx=cols-self.manrect/2
                if (self.many-self.manrect/2)<0:
                    self.many=self.manrect/2
                if (self.many+self.manrect/2)>rows:
                    self.many=rows-self.manrect/2
                framecrop = fgmask[self.many-self.manrect/2:self.many+self.manrect/2,self.manx-self.manrect/2:self.manx+self.manrect/2]
                cropgrey = framecrop#cv2.cvtColor(framecrop,cv2.COLOR_BGR2GRAY)
                cropcanny = cv2.Canny(cropgrey,100,200)
                cropcanny = cv2.dilate(cropcanny,self.kernel,iterations=1)
                im,contours,hier = cv2.findContours(cropcanny.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                if(len(contours))>0:
                    cropcannycolor = cv2.cvtColor(cropcanny,cv2.COLOR_GRAY2BGR)
                    cv2.drawContours(cropcannycolor,contours,-1,(0,255,0),5)
                    # cv2.imshow("contours",cropcannycolor)
                    # cv2.waitKey(10)
                    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:5]
                    cnt = contours[0]
                    
                    if len(cnt)>5:
                        ellipse = cv2.fitEllipse(cnt)
                        # print ellipse
                        ellipse2 = ((self.manx-self.manrect/2+ellipse[0][0],self.many-self.manrect/2+ellipse[0][1]),(ellipse[1][0],ellipse[1][1]),ellipse[2])
                        angle = -(ellipse2[2]-90.0)
                        # if (abs(self.detectedangle-angle)<abs(180-self.detectedangle-angle)):
                        #     self.detectedangle=angle
                        # else:
                        #     self.detectedangle=180-angle
                        self.detectedangle = angle
                        print self.detectedangle
                        # ellipse[0]=(ellipse[0][0]+self.manx,ellipse[0][1]+self.many)
                        # ellipse[0][1]+=self.many
                        cv2.ellipse(frame,ellipse2,(255,0,0),2)
                cv2.rectangle(frame,(self.manx-self.manrect/2,self.many-self.manrect/2),(self.manx+self.manrect/2,self.many+self.manrect/2),(0,255,0),1)
                cv2.circle(frame,(self.manx,self.many),self.mandia,(0,255,0),1)

                if ((self.manx-self.manrect/2)>0 and (self.manx+self.manrect/2)<cols):
                    if((self.many-self.manrect/2)>0 and (self.many+self.manrect/2)<rows):
                        crop = frame_orig[self.many-self.manrect/2:self.many+self.manrect/2,self.manx-self.manrect/2:self.manx+self.manrect/2,:]
                        cropfile = str(data.header.seq)+'_fish'+self.fishnum+'.jpg'
                        cv2.imwrite(self.image_directory+self.bagchop+'_'+cropfile,crop)

                if self.drawing==True:
                    self.f.write(str(rospy.get_time())+'\t'+str(data.header.seq)+'\t'+str(self.manx)+'\t'+str(self.many)+'\r\n')
                    self.anglef.write(str(rospy.get_time())+'\t'+str(data.header.seq)+'\t'+str(self.detectedangle)+'\r\n')

            cv2.imshow('fish',frame)
            cv2.waitKey(1)

    
    def update_fish(self,event,x,y,flags,param):
        if event==cv2.EVENT_LBUTTONDOWN:
            if(self.drawing==False):
                self.drawing = True
            else:
                self.drawing = False
        # elif event==cv2.EVENT_LBUTTONUP:
        #     self.drawing=False
        elif event==cv2.EVENT_MOUSEMOVE:
            if self.drawing==True:
                self.manx,self.many = x,y

        



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
