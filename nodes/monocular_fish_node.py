#!/usr/bin/env python

#Alex Brown
#2018

import roslib
roslib.load_manifest('fishtracker')
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
# from cv2 import cv
import tf
import rospkg
from skimage import data, color, img_as_ubyte
from skimage.feature import canny
from skimage.transform import hough_ellipse
from skimage.draw import ellipse_perimeter

class Measurefish:

    def __init__(self):
        #cv bridge for handling image transport
        self.bridge = CvBridge()
        
        #side view detection specific
        self.svminx,self.svmaxx,self.svminy,self.svmaxy = rospy.get_param('svminx',220),rospy.get_param('svmaxx',1100),rospy.get_param('svminy',230),rospy.get_param('svmaxy',370)
        rospack = rospkg.RosPack()
        self.package_path=rospack.get_path('fishtracker')
        #this is how we get our image in to use openCV
        self.cascade = cv2.CascadeClassifier(self.package_path+'/cascade/fish_sideview_1.xml')#'package://fishtracker/meshes/fishbody.dae'
    

        #top view detection specific
        self.fgbg = cv2.createBackgroundSubtractorMOG2()
        self.fgbg.setDetectShadows(True)
        self.fgbg.setShadowValue(0)
        self.minw = 50
        self.maxw = 250
        self.minh = 50
        self.maxh = 250
        self.kernel = np.ones((9,9),np.uint8)

        #pub and sub for camera 1
        self.imside_sub = rospy.Subscriber("/camera2/usb_cam2/image_raw/compressed",CompressedImage,self.sideviewcallback,queue_size=1)#change this to proper name!
        self.CIside_sub = rospy.Subscriber("/camera2/usb_cam2/camera_info",CameraInfo,self.CItopcallback,queue_size=1)
        self.imside_pub = rospy.Publisher('/fishtracker/side/overlay_imside',Image,queue_size=1)
        #placeholder for cam1 parameters
        self.CIside = CameraInfo()
        #placeholder for the ball row and column
        self.fishuvside = array([0,0])


        #ball marker publisher
        self.markerpub = rospy.Publisher('/measured_fish',Marker,queue_size=1)

        #publish a pose message for the KF
        self.posepub = rospy.Publisher('/fishtracker/measuredfishpose',PoseStamped,queue_size=1)

        self.topcounter=1
        self.sidecounter=1
        self.every=3

        self.lasttopstamp = rospy.Time.now()
        self.lastsidestamp = rospy.Time.now()

        self.currenttopstamp = rospy.Time.now()
        self.currentsidestamp = rospy.Time.now()
        self.angle = 0
        
        #timed loop that will generate two synced images
        rospy.Timer(rospy.Duration(0.1),self.triangulate,oneshot=False)

    def cleanRects(self,rects):
        #gets rid of any rects that are fully contained within another
        rects = array(rects)
        rectsout=rects.copy()
        badrects = array([],dtype=int32)
        for rectnum in range(1,len(rects[:,0])):
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

    def discardROI(self,rects,minx,miny,maxx,maxy):
        rects = array(rects)
        rectsout = rects.copy()
        badrects = array([],dtype=int32)
        for rectnum in range(0,len(rects[:,0])):
            #what rect are we looking at?
            rect_tlx,rect_tly,rect_brx,rect_bry = rects[rectnum,0],rects[rectnum,1],rects[rectnum,0]+rects[rectnum,2],rects[rectnum,1]+rects[rectnum,3]
            if (((rect_tlx-minx)>0 and (rect_tly-miny)>0) and  ((rect_brx-maxx)<0 and (rect_bry-maxy)<0)):
                pass
            else:
                badrects = append(badrects,rectnum)
                #print "found bad rect at index "+str(rectnum)+" of "+str(len(rects[:,0]))
        rectsout=delete(rectsout,badrects,0)
        return rectsout

    def box(self,rects, img):
        #print rects.shape
        for x1, y1, x2, y2 in rects:
            cv2.rectangle(img, (x1, y1), (x2, y2), (127, 255, 0), 2)
        #cv2.imwrite('one.jpg', img);

    def detectSideview(self,frame):
        #print cascade
        #rejectLevels??
        #maxSize=(200,100),
        # rects = cascade.detectMultiScale(img, scaleFactor=1.6, minNeighbors=24,  minSize=(20,20),maxSize=(200,100),flags=cv2.CASCADE_SCALE_IMAGE)
        # rects = self.cascade.detectMultiScale(img, scaleFactor=1.6, minNeighbors=7,  minSize=(20,20),maxSize=(200,100),flags=cv2.CASCADE_SCALE_IMAGE)
        frame = frame.copy()
        cv2.rectangle(frame,(self.svminx,self.svminy),(self.svmaxx,self.svmaxy),(0,0,255),2)

        img = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            # frame = cv2.resize(frame,None,fx=self.imscale, fy=self.imscale, interpolation = cv2.INTER_CUBIC)
        
        rects = self.cascade.detectMultiScale(img, scaleFactor=1.01, minNeighbors=7,  minSize=(1,1),maxSize=(70,100),flags=cv2.CASCADE_SCALE_IMAGE)
        if(len(rects))>0:
            rects2=self.cleanRects(rects)
            # rects3= rects2
            rects3 = self.discardROI(rects,self.svminx,self.svminy,self.svmaxx,self.svmaxy)
            rects3[:, 2:] += rects3[:, :2]
            self.box(rects3,frame)

            if len(rects3)>0:
                return frame,array([(rects3[0,0]+rects3[0,2])/2,(rects3[0,1]+rects3[0,3])/2])
            else:
                return frame,array([])
        else:
            return frame,array([])


    def detectGeneric(self,img,thisname):
        frame = img.copy()
        rows,cols, = frame.shape

        rects = None
        if rows>0:
            #fgmask = self.fgbg.apply(frame)
            gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            canny = cv2.Canny(gray,100,200)
            # canny = cv2.dilate(canny,self.kernel,iterations=1)
            cannycolor = cv2.cvtColor(canny,cv2.COLOR_GRAY2BGR)
            im,contours,hier = cv2.findContours(canny.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            
            if(len(contours)>0):
                cv2.drawContours(cannycolor,contours,-1,(0,255,0),5)
                for k in range(0,len(contours)):
                    cnt = contours[k]
                    x,y,w,h = cv2.boundingRect(array(cnt))
                    #print x,y,w,h
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
            cv2.rectangle(frame,(self.svminx,self.svminy),(self.svmaxx,self.svmaxy),(0,0,255),2)
            # cv2.imshow('frame',frame)
            #cv2.imshow(thisname,frame)

            #cv2.waitKey(1)

        if rects is not None:
            return frame,array([(rects[0,0]+rects[0,2])/2,(rects[0,1]+rects[0,3])/2])
        else:
            return frame,array([])


    #this function fires whenever a new image_raw is available. it is our "main loop"
    def sideviewcallback(self,data):
        
        if self.sidecounter<self.every:
            self.sidecounter+=1
        else:
            self.sidecounter=1
            self.currentsidestamp=data.header.stamp
            # rospy.logwarn("side now "+str(self.currentsidestamp)+" was "+str(self.lastsidestamp))
            try:
                np_arr = np.fromstring(data.data, np.uint8)
                # Decode to cv2 image and store
                frame= cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                he,wi,de = frame.shape
                #print frame.shape
                if frame is not None:
                    print "processing side frame"
                    #frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
                    frameside_out,self.fishuvside = self.detectSideview(frame)
                    img_out = self.bridge.cv2_to_imgmsg(frameside_out, "8UC3")
                    img_out.header.stamp = rospy.Time.now()
                    img_out.header.frame_id = '/camera2'
                    self.imside_pub.publish(img_out)
                else:
                    print "side frame is none"
            except CvBridgeError as e:
                print(e)

    def CIsidecallback(self,data):
        self.CIside = data

    def Unproject(self,points, Z, intrinsic, distortion):
        f_x = intrinsic[0, 0]
        f_y = intrinsic[1, 1]
        c_x = intrinsic[0, 2]
        c_y = intrinsic[1, 2]
        # This was an error before
        # c_x = intrinsic[0, 3]
        # c_y = intrinsic[1, 3]

        # Step 1. Undistort.
        points_undistorted = np.array([])
        if len(points) > 0:
            points_undistorted = cv2.undistortPoints(np.expand_dims(points, axis=1), intrinsic, distortion, P=intrinsic)
        points_undistorted = np.squeeze(points_undistorted, axis=1)

        # Step 2. Reproject.
        result = []
        for idx in range(points_undistorted.shape[0]):
            z = Z[0] if len(Z) == 1 else Z[idx]
            x = (points_undistorted[idx, 0] - c_x) / f_x * z
            y = (points_undistorted[idx, 1] - c_y) / f_y * z
            result.append([x, y, z])
        return result


    def triangulate(self,data):
        # rospy.logwarn("triangulating: sidelen = "+str(len(self.fishuvside))+" toplen: "+str(len(self.fishuvtop)))
        if((len(self.fishuvside)>0):
            if (1):#( (self.currentsidestamp != self.lastsidestamp) and (self.currenttopstamp!=self.lasttopstamp)  ):
                self.lastsidestamp = self.currentsidestamp
                #print "triangulating!"
                D2 = array([0.0, 0.0, 0.0, 0.0])  # This works!

                P2 = array([self.CIside.P]).reshape(3,4)
                x2 = vstack(self.fishuvside)
                x2 = vstack((x2,1))
                #use the camera information and an estimate of fish plane depth to unproject
                rospy.logwarn("broadcasting measured fish!")
                try:
                    fishpos = self.unproject(x2,0.5,P2,D2)
                    # fishpos/=fishpos[3]
                    #print ballpos

                    #send transforms to show ball's coordinate system
                    br = tf.TransformBroadcaster()
                    fishquat = tf.transformations.quaternion_from_euler(0,0,self.angle)
                    #I think the ball position needs to be inverted... why? Not sure but I think
                    #it may be because there is a confusion in the T matrix between camera->object vs. object->camera
                    br.sendTransform(-fishpos[:,0],fishquat,rospy.Time.now(),'/fishmeasured','world')

                    #create a marker
                    fishmarker = Marker()
                    fishmarker.header.frame_id='/fishmeasured'
                    fishmarker.header.stamp = rospy.Time.now()
                    fishmarker.type = fishmarker.SPHERE
                    fishmarker.action = fishmarker.MODIFY
                    fishmarker.scale.x = .12
                    fishmarker.scale.y = .12
                    fishmarker.scale.z = .12
                    fishmarker.pose.orientation.w=1
                    fishmarker.pose.orientation.x = 0
                    fishmarker.pose.orientation.y=0
                    fishmarker.pose.orientation.z=0
                    fishmarker.pose.position.x = 0
                    fishmarker.pose.position.y=0
                    fishmarker.pose.position.z=0
                    fishmarker.color.r=0.0
                    fishmarker.color.g=0
                    fishmarker.color.b=1.0
                    fishmarker.color.a=0.5

                    self.markerpub.publish(fishmarker)

                    posemsg = PoseStamped()
                    posemsg.header.stamp = rospy.Time.now()
                    posemsg.pose.position.x = 0.-fishpos[0,0]
                    posemsg.pose.position.y = 0.-fishpos[1,0]
                    posemsg.pose.position.z = 0.-fishpos[2,0]
                    self.posepub.publish(posemsg)
                except:
                    rospy.logwarn("triangulation failed")


def main(args):
  
  rospy.init_node('measure_fish', anonymous=True)
  ic = Measurefish()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
