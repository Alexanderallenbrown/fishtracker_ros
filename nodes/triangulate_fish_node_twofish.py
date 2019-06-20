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
from skimage import data, color, img_as_ubyte
from skimage.feature import canny
from skimage.transform import hough_ellipse
from skimage.draw import ellipse_perimeter


# def nearestColumn(uv1,uv2):
#     #uv1 and uv2 come in as [x11 x2 ...; y1 y2 ...] 
#     #our job here is to sort these so that the x coordinates are the best matches.
#     #NOTE THIS IS SPECIFIC TO OUR CASE WHERE 
#     x1 = uv1[0,:]
#     x2 = uv2[0,:]

class Measurefish:

    def __init__(self):
        #cv bridge for handling image transport
        self.bridge = CvBridge()
        
        #side view detection specific
        self.svminx,self.svmaxx,self.svminy,self.svmaxy = rospy.get_param('svminx',220),rospy.get_param('svmaxx',1080),rospy.get_param('svminy',260),rospy.get_param('svmaxy',400)
        rospack = rospkg.RosPack()
        self.package_path=rospack.get_path('fishtracker')
        #this is how we get our image in to use openCV
        self.cascade = cv2.CascadeClassifier(self.package_path+'/cascade/fish_sideview_1.xml')#'package://fishtracker/meshes/fishbody.dae'
    

        #top view detection specific
        self.fgbg = cv2.createBackgroundSubtractorMOG2()
        self.fgbg.setDetectShadows(True)
        self.fgbg.setShadowValue(0)
        # self.fgbg.setHistory(10)
        self.minw = 60
        self.maxw = 250
        self.minh = 60
        self.maxh = 250
        self.kernel = np.ones((9,9),np.uint8)

        #pub and sub for camera 1
        self.imside_sub = rospy.Subscriber("/camera2/usb_cam2/image_raw/compressed",CompressedImage,self.sideviewcallback,queue_size=1)#change this to proper name!
        self.CIside_sub = rospy.Subscriber("/camera2/usb_cam2/camera_info",CameraInfo,self.CItopcallback,queue_size=1)
        self.imside_pub = rospy.Publisher('/fishtracker/overlay_imside',Image,queue_size=1)
        #placeholder for cam1 parameters
        self.CIside = CameraInfo()
        #placeholder for the ball row and column
        self.fishuvside = array([[0],[0]])

        #pub and sub for camera 2
        self.imtop_sub = rospy.Subscriber("/camera1/usb_cam1/image_raw/compressed",CompressedImage,self.topviewcallback,queue_size=1)#change this to proper name!
        self.CItop_sub = rospy.Subscriber("/camera1/usb_cam1/camera_info",CameraInfo,self.CIsidecallback,queue_size=1)
        self.imtop_pub = rospy.Publisher('/fishtracker/overlay_imtop',Image,queue_size=1)
        #placeholder for cam1 parameters
        self.CItop = CameraInfo()
        #placeholder for the ball row and column
        self.fishuvtop = array([[0],[0]])

        #ball marker publisher
        self.markerpub0 = rospy.Publisher('/measured_fish0',Marker,queue_size=1)
        self.markerpub1 = rospy.Publisher('/measured_fish1',Marker,queue_size=1)

        #publish a pose message for the KF
        self.posepub0 = rospy.Publisher('/fishtracker/measuredfishpose0',PoseStamped,queue_size=1)
        self.posepub1 = rospy.Publisher('/fishtracker/measuredfishpose1',PoseStamped,queue_size=1)

        self.topcounter=1
        self.sidecounter=1
        self.every=3

        self.lasttopstamp = rospy.Time.now()
        self.lastsidestamp = rospy.Time.now()

        self.currenttopstamp = rospy.Time.now()
        self.currentsidestamp = rospy.Time.now()
        
        #timed loop that will generate two synced images
        rospy.Timer(rospy.Duration(0.1),self.triangulate,oneshot=False)

    # def cleanRects(self,rects):
    #     #gets rid of any rects that are fully contained within another
    #     rects = array(rects)
    #     rectsout=rects.copy()
    #     badrects = array([],dtype=int32)
    #     for rectnum in range(0,len(rects[:,0])):
    #         #what rect are we looking at?
    #         rect_tlx,rect_tly,rect_brx,rect_bry = rects[rectnum,0],rects[rectnum,1],rects[rectnum,0]+rects[rectnum,2],rects[rectnum,1]+rects[rectnum,3]
    #         #now see if any others are contained within it
    #         for testnum in range(1,len(rects[:,0])):
    #             testrect_tlx,testrect_tly,testrect_brx,testrect_bry = rects[testnum,0],rects[testnum,1],rects[testnum,0]+rects[testnum,2],rects[testnum,1]+rects[testnum,3]
    #             if ((rect_tlx-testrect_tlx)<=0 and (rect_tly-testrect_tly)<=0):
    #                 #this means that the TL corner is inside the rect
    #                 if ((rect_brx-testrect_brx)>=0 and (rect_bry-testrect_bry)>=0):
    #                     #this means that testrect is fully enclosed in rect, so delete it
    #                     badrects = append(badrects,testnum)
    #                     #print "found bad rect at index "+str(testnum)+" of "+str(len(rects[:,0]))
    #     rectsout=delete(rectsout,badrects,0)
    #     return rectsout

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
                if ((rect_tlx-testrect_tlx)<=0 and (rect_tly-testrect_tly)<=0):
                    #this means that the TL corner is inside the rect
                    if ((rect_brx-testrect_brx)>=0 and (rect_bry-testrect_bry)>=0):
                        #this means that testrect is fully enclosed in rect, so delete it
                        if testnum is not rectnum:
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
        
        rects = self.cascade.detectMultiScale(img, scaleFactor=1.01, minNeighbors=3,  minSize=(1,1),maxSize=(70,100),flags=cv2.CASCADE_SCALE_IMAGE)
        if(len(rects))>0:
            rects2=self.cleanRects(rects)
            # rects3= rects2
            rects3 = self.discardROI(rects,self.svminx,self.svminy,self.svmaxx,self.svmaxy)
            rects3[:, 2:] += rects3[:, :2]
            self.box(rects3,frame)

            if len(rects3)>0:
                nfish,npts = rects3.shape
                # print "side view fish: "+str(rects3.shape)
                out = zeros((2,nfish))
                for k in range(0,nfish):
                    # pass
                    out[:,k] = [(rects3[k,0]+rects3[k,2])/2,(rects3[k,1]+rects3[k,3])/2]
                return frame,out
            else:
                return frame,array([[]])
        else:
            return frame,array([[]])

    def detectTopview(self,frame):
        # self.timenow = rospy.Time.now()
        #print frame.shape
        rows,cols,depth = frame.shape

        rects = None
        rectsout = None
        if rows>0:
            gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            fgmask = self.fgbg.apply(gray)
            canny = cv2.Canny(fgmask,100,200)
            canny = cv2.dilate(canny,self.kernel,iterations=3)
            cannycolor = cv2.cvtColor(canny,cv2.COLOR_GRAY2BGR)

            im,contours,hier = cv2.findContours(canny.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)          
            
            if(len(contours)>0):
                cv2.drawContours(cannycolor,contours,-1,(0,255,0),5)
                # cv2.imshow("contours",cannycolor)
                # cv2.waitKey(1)
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
                # print rects.shape
                # print rectsout.shape
                self.box(rectsout,frame)

                
            if rectsout is not None:
                nfish,npts = rectsout.shape
                # if nfish>1:
                    # print "top view fish: "+str(rectsout.shape)
                out = zeros((2,nfish))
                for k in range(0,nfish):
                    # pass
                    out[:,k] = [(rectsout[k,0]+rectsout[k,2])/2,(rectsout[k,1]+rectsout[k,3])/2]
                return frame,out
            else:
                return frame,array([[]])


    #this function fires whenever a new image_raw is available. it is our "main loop"
    def sideviewcallback(self,data):
        
        if self.sidecounter<self.every:
            self.sidecounter+=1
        else:
            self.sidecounter=1
            self.currentsidestamp=data.header.stamp
            try:
                np_arr = np.fromstring(data.data, np.uint8)
                # Decode to cv2 image and store
                frame= cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                he,wi,de = frame.shape
                #print frame.shape
                if frame is not None:
                    # print "processing side frame"
                    #frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
                    frameside_out,self.fishuvside = self.detectSideview(frame)
                    img_out = self.bridge.cv2_to_imgmsg(frameside_out, "8UC3")
                    img_out.header.stamp = rospy.Time.now()
                    self.imside_pub.publish(img_out)
                else:
                    print "side frame is none"
            except CvBridgeError as e:
                print(e)

    def topviewcallback(self,data):
        
        #print "in callback"
        if self.topcounter>self.every:
            self.topcounter+=1
        else:
            self.topcounter=1
            self.currenttopstamp = data.header.stamp
            try:
                #use the np_arr thing if subscribing to compressed image
                #frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
                np_arr = np.fromstring(data.data, np.uint8)
                # Decode to cv2 image and store
                frame= cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                # cv2.imshow("top",frame)
                # cv2.waitKey(1)
                if frame is not None:
                    #print "detecting 1"
                    frametop_out,self.fishuvtop = self.detectTopview(frame)
                    #print "im1: "+str(self.balluv1)
                    img_out = self.bridge.cv2_to_imgmsg(frametop_out, "8UC3")
                    img_out.header.stamp = rospy.Time.now()
                    self.imtop_pub.publish(img_out)

                else:
                    print "frame is none"
            except CvBridgeError,e:
                print e

    def genericcallback(self,data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #if image topic is compressed, use following two lines instead
            # np_arr = fromstring(data.data,uint8)
            # frame = cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
            if frame is not None:
                #print "detecting 1"
                frame1_out,self.balluv1 = self.detectGeneric(frame,'cam1')
                #print "im1: "+str(self.balluv1)
                img_out = self.bridge.cv2_to_imgmsg(frame1_out, "bgr8")
                img_out.header.stamp = rospy.Time.now()
                self.im1_pub.publish(img_out)

            else:
                print "frame is none"
        except CvBridgeError,e:
            print e

    def CItopcallback(self,data):
        self.CItop = data
        #print self.CI1.P

    def CIsidecallback(self,data):
        self.CIside = data

    def triangulate(self,data):
        #now we have to sort so that the first fish in side image corresponds with the same fish in top image, etc.
        #how many fish are in each image right now?
        pts,nfishside = self.fishuvside.shape
        pts,nfishtop = self.fishuvtop.shape
        #first check to see if we have a measurement in both images
        if((nfishside>0) and (nfishtop)>0):
            #now check to see if both are new
            if ( (self.currentsidestamp != self.lastsidestamp) and (self.currenttopstamp!=self.lasttopstamp)  ):
                self.lasttopstamp = self.currenttopstamp
                self.lastsidestamp = self.currentsidestamp

                
                #how many fish do we have in BOTH images? Can only deal with that number for triangulation.
                nfish = min(nfishside,nfishtop)
                if nfish>2:
                    nfish=2
                # rospy.loginfo("num fish is: "+str(nfish))
                #sort fish in both top and bottom by their x-coordinate in the image (this only works for our case where columns should roughly align)
                inds_side = argsort(self.fishuvside[0,:])
                inds_top = argsort(self.fishuvtop[0,:])
                #now cut and prepare the x1 and x2 vectors for openCV
                # print nfishtop,nfishside
                x1 = self.fishuvtop[:,inds_top]
                x2 = self.fishuvside[:,inds_side]
                #make each homogeneous (add ones)
                # x1 = vstack((x1,ones((1,nfishtop))))
                # x2 = vstack((x2,ones((1,nfishside))))
                #now we still have the problem that x1 and x2 may not be the same number of fish.
                #but we have to be careful here because there may be 2 rectangles on one fish body... 
                #deal with this later, and for now just take nfish columns from each
                x1 = x1[:,0:nfish]
                x2 = x2[:,0:nfish]

                if nfish==2:
                    rospy.loginfo("x1: "+str(x1))
                    rospy.loginfo("x2: "+str(x2))

                #prepare the projection matrices
                P1 = array([self.CItop.P]).reshape(3,4)
                P2 = array([self.CIside.P]).reshape(3,4)
                print "top: "+str(P1)
                print "side: "+str(P2)
                
                #use the camera information and the current guess for the u,v
                #to triangulate the position of the ball.
                #print x1,x2
                #print P1,P2
                fishpos = cv2.triangulatePoints(P1, P2, x1, x2)
                fishpos/=fishpos[3,:]
                #print ballpos
                if nfish==2:
                    rospy.loginfo(str(fishpos))

                #send transforms to show ball's coordinate system
                br = tf.TransformBroadcaster()
                for k in range(0,nfish):
                    fishquat = tf.transformations.quaternion_from_euler(0,0,0)
                    #I think the ball position needs to be inverted... why? Not sure but I think
                    #it may be because there is a confusion in the T matrix between camera->object vs. object->camera
                    br.sendTransform(-fishpos[:,k],fishquat,rospy.Time.now(),'/fishmeasured_'+str(k),'world')

                    #create a marker
                    fishmarker = Marker()
                    fishmarker.header.frame_id='/fishmeasured_'+str(k)
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

                    if k==0:
                        self.markerpub0.publish(fishmarker)
                        posemsg = PoseStamped()
                        posemsg.header.stamp = rospy.Time.now()
                        posemsg.pose.position.x = 0.-fishpos[0,0]
                        posemsg.pose.position.y = 0.-fishpos[1,0]
                        posemsg.pose.position.z = 0.-fishpos[2,0]
                        self.posepub0.publish(posemsg)
                    elif k==1:
                        self.markerpub1.publish(fishmarker)
                        posemsg = PoseStamped()
                        posemsg.header.stamp = rospy.Time.now()
                        posemsg.pose.position.x = 0.-fishpos[0,0]
                        posemsg.pose.position.y = 0.-fishpos[1,0]
                        posemsg.pose.position.z = 0.-fishpos[2,0]
                        self.posepub1.publish(posemsg)

    def correspondence(self,uvside,uvtop):
        #ok, let's figure out how many fish we're dealing with
        pass


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