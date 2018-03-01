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
from sensor_msgs.msg import Image
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
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback,queue_size=1)#change this to proper name!
        self.fishmarkerpub = rospy.Publisher('/measured_fishmarker',Marker,queue_size=1)
        self.image_pub = rospy.Publisher('/fishtracker/overlay_image',Image,queue_size=1)
        self.timenow = rospy.Time.now()
        self.imscale = 1.0

        self.cam_pos = (0,0,18*.0254)
        self.cam_quat = tf.transformations.quaternion_from_euler(pi,0,0)
        self.fgbg = cv2.createBackgroundSubtractorMOG2()
        self.minw = 10
        self.maxw = 150
        self.minh = 10
        self.maxh = 150
        self.kernel = np.ones((401,401),np.uint8)
        rospack = rospkg.RosPack()
        # get the file path for rospy_tutorials
        self.package_path=rospack.get_path('fishtracker')

        self.cascade = cv2.CascadeClassifier(self.package_path+'/cascade/fish_sideview_1.xml')#'package://fishtracker/meshes/fishbody.dae'
        

        #kalman filter additions
        self.num_fish = rospy.get_param('num_fish',4)
        self.kalman_measurement = array([[1.,0.,0.,0.],[0.,0.,1.,0.]])
        self.kalman_transition = np.array([[1.,1.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,1.],[0.,0.,0.,1]])
        self.kalman_processnoise = .2 * np.eye(4)
        self.kalman_measurementnoise = 100. * np.eye(2)
        self.kalman_errorcovpost = 1. * np.ones((4, 4))
        self.kalman_statepost = 0.1 * np.random.randn(4, 1)
        #list of kalman filters (one for each fish)
        self.filters = []
        #list of raw measurements
        self.measurements = []
        #list of filtered fish pose estimates
        self.pose_estimates = []
        #list of positions only
        self.fish_positions = []
        #list of steps since last measurement (for destroying filter)
        self.steps_without_measurement = []
        #what is the max innovation (measurement error) allowed for one filter?
        self.meas_error_thresh = 100;

    def measurements_from_boxes(self,boxes):
        measurements = []
        #print boxes
        for k in range(0,len(boxes)):
            x = (boxes[k,0]+boxes[k,2])/2
            y = (boxes[k,1]+boxes[k,3])/2
            measurements.append([x,y])
        return measurements

    def runKalman(self,measurements):
        #clean up filters where fish are off screen.
        fishtodelete = []
        if len(self.pose_estimates)>0:
            for k in range(0,len(self.pose_estimates)):
                pose = self.pose_estimates[k]
                #print "pose shape is "+str(pose.shape)
                if(pose[0,0]<0 or pose[2,0]<0 or pose[0,0]>480 or pose[2,0]>480 or self.steps_without_measurement[k]>30):
                    fishtodelete.append(k)
                    "deleting fish..."
            # print fishtodelete
            # print "fishtodelete above"
            self.pose_estimates = [x for i,x in enumerate(self.pose_estimates) if i not in fishtodelete]
            self.filters = [x for i, x in enumerate(self.filters) if i not in fishtodelete]
            self.steps_without_measurement = [x for i, x in enumerate(self.steps_without_measurement) if i not in fishtodelete]
        # #print "filters now" + str(self.filters)

        #first, let's determine which measurement belongs to which fish
        #find distance from each measurement to each state estimate
        #initialize aligned measurement vector
        ameas = []
        orphanmeas = []

        for k in range(0,len(self.pose_estimates)):
            ameas.append([])
        if (len(self.filters)>0 and len(measurements)>0):
            #print "calculating distances"
            dists = zeros((len(measurements),len(self.filters)))
                #TODO I do'nt think this algorithm handles when we have too many measurements...
            for k in range(0,len(measurements)):
                for j in range(0,len(self.filters)):
                    #calculate the distance from this measurement k to this pose estimate j
                    dists[k,j] = ((measurements[k][0]-self.pose_estimates[j][0])**2+(measurements[k][1]-self.pose_estimates[j][1])**2)**.5
                #which fish does this measurement belong to?
                minmeas_index = argmin(dists[k,:])
                #TODO to handle "too many" measurements, check to see if we already set this, and then check distances?
                if dists[k,minmeas_index]<self.meas_error_thresh:
                    if len(ameas[minmeas_index])==0:
                        ameas[minmeas_index]=[measurements[k][0],measurements[k][1]]
                    else:
                        print "adding orphan"
                        orphanmeas.append([measurements[k][0],measurements[k][1]])
                else: #code this as an orphan measurement
                    print "adding orphan"
                    orphanmeas.append([measurements[k][0],measurements[k][1]])
            print "aligned measurements"+str(ameas)

        #check to see if all fish are being tracked
        if len(self.filters)==0:
            for k in range(0,min(len(measurements),self.num_fish)):
                    print "adding more filters"
                    newkf = cv2.KalmanFilter(4,2,0)
                    self.filters.append(cv2.KalmanFilter(4,2,0))#set up a new KF
                    self.pose_estimates.append(array([[measurements[k][0]*1.0],[0.],[1.0*measurements[k][1]],[0.]]))
                    #self.pose_estimates.append(array([[0*1.0],[0.],[0*1.0],[0.]]))

                    self.steps_without_measurement.append(0)
                    #note the "copy" below... this is NECESSARY or the KF objects will share matrices... ugh.
                    self.filters[k].transitionMatrix = copy(self.kalman_transition)
                    self.filters[k].measurementMatrix = copy(self.kalman_measurement)
                    self.filters[k].processNoiseCov = copy(self.kalman_processnoise)
                    self.filters[k].measurementNoiseCov = copy(self.kalman_measurementnoise)
                    self.filters[k].errorCovPost = copy(self.kalman_errorcovpost)
                    self.filters[k].statePost = copy(self.kalman_statepost)
        elif (len(self.filters)<=self.num_fish):
            #now check to see if we have more measurements than filters
            if (len(measurements)>=len(self.filters)):
                #now add/initialize any filters we are missing
                for k in range(0,len(orphanmeas)):
                    print "adding more filters"
                    newkf = cv2.KalmanFilter(4,2,0)
                    self.filters.append(cv2.KalmanFilter(4,2,0))#set up a new KF
                    self.pose_estimates.append(array([[orphanmeas[k][0]*1.0],[0.],[1.0*orphanmeas[k][1]],[0.]]))
                    #self.pose_estimates.append(array([[0*1.0],[0.],[0*1.0],[0.]]))

                    self.steps_without_measurement.append(0)
                    #note the "copy" below... this is NECESSARY or the KF objects will share matrices... ugh.
                    self.filters[k].transitionMatrix = copy(self.kalman_transition)
                    self.filters[k].measurementMatrix = copy(self.kalman_measurement)
                    self.filters[k].processNoiseCov = copy(self.kalman_processnoise)
                    self.filters[k].measurementNoiseCov = copy(self.kalman_measurementnoise)
                    self.filters[k].errorCovPost = copy(self.kalman_errorcovpost)
                    self.filters[k].statePost = copy(self.kalman_statepost)
        # print self.pose_estimates
        #print self.filters
        #print "measurements "+str(measurements)
        #print "filters: "+str(len(self.filters))
        #now we need to predict and update on each filter.
        
        #now we actually run the kalman update for each filter, correcting only if we have a meas.
        for k in range(0,len(self.filters)):
            #make a prediction in this kalman filter
            thisfilter = self.filters[k]
            temppos = thisfilter.predict()
            # print "predicted: "+str(self.pose_estimates[k])
            if len(ameas)>k:
                print len(ameas),k
                if len(ameas[k])>0:
                    print "using measurement " +str(ameas[k])
                    print len(ameas[k])
                    # self.pose_estimates[k] = self.filters[k].correct(array([[float(ameas[k][0])],[float(ameas[k][1])]]))
                    measurement = array([[ameas[k][0]*1.0],[1.0*ameas[k][1]]])
                    innovation = array([[ameas[k][0]*1.0-temppos[0]],[1.0*ameas[k][1]-temppos[2]]])
                    if (innovation[0]<self.meas_error_thresh and innovation[1]<self.meas_error_thresh):
                        #print "measurement: "+str(measurement)
                        temppos=thisfilter.correct(measurement)
                        self.steps_without_measurement[k]=0
                    #print "corrected: "+str(self.pose_estimates[k])
                else:
                    self.steps_without_measurement[k]=self.steps_without_measurement[k]+1
                    #self.pose_estimates[k]=thisfilter.correct()
                    #print "did not correct filter " +str(k)
                #print "temp pos is: " +str(temppos)
                self.pose_estimates[k] = temppos
                print "steps without measurement: " + str(self.steps_without_measurement)


    def bgClean(self,frame):
        rows,cols,depth = frame.shape
        rects = None
        if rows>0:
            #fgmask = self.fgbg.apply(frame)
            gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            #gray = cv2.bilateralFilter(gray, 11, 17, 17)
            fgmask = self.fgbg.apply(gray)
            gray = cv2.bitwise_and(gray,fgmask)
            #cv2.dilate(fgmask,self.kernel,iterations=1)
            cv2.imshow('masked',gray)
            return gray

    def detect(self,img):
        #print cascade
        #rejectLevels??
        #maxSize=(200,100),
        # rects = cascade.detectMultiScale(img, scaleFactor=1.6, minNeighbors=24,  minSize=(20,20),maxSize=(200,100),flags=cv2.CASCADE_SCALE_IMAGE)
        rects = self.cascade.detectMultiScale(img, scaleFactor=1.05, minNeighbors=2,  minSize=(20,20),maxSize=(100,50),flags=cv2.CASCADE_SCALE_IMAGE)
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
                        #print "found bad rect at index "+str(testnum)+" of "+str(len(rects[:,0]))
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
                        #print "found bad rect at index "+str(testnum)+" of "+str(len(rects[:,0]))
        rectsout=delete(rectsout,badrects,0)
        return rectsout

    def box(self,rects, img, color=(127,255,0)):
        rects = array(rects)
        #print rects.shape
        for x1, y1, x2, y2 in rects:
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        #cv2.imwrite('one.jpg', img);

    def boxBW(self,rects, img):
        for x1, y1, x2, y2 in rects:
            cv2.rectangle(img, (x1, y1), (x2, y2), ( 255,255,255), 2)
        #cv2.imwrite('one.jpg', img);

  #this function fires whenever a new image_raw is available. it is our "main loop"
    def callback(self,data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            he,wi,de = frame.shape
            frame = frame[self.top_crop:he-self.bottom_crop,:]
            frame_orig = frame
        except CvBridgeError, e:
            print e
        
        frame_clean = self.bgClean(frame)
        frame_clean_cvt = cv2.cvtColor(frame_clean,cv2.COLOR_GRAY2BGR)
        rects,frame_clean = self.detect(frame_clean)
        #self.box(rects,frame,(255,0,0))
        
        

        #begin kalman stuff
        measurements = self.measurements_from_boxes(rects)
        #print measurements
        for meas in measurements:
            #print meas
            cv2.circle(frame,(meas[0],meas[1]),10,(0,0,255),2)
        self.runKalman(measurements)
        self.fish_positions = zeros((len(self.pose_estimates),2))
        for k in range(0,len(self.pose_estimates)):
            self.fish_positions[k,:] = array([int(self.pose_estimates[k][0][0]),int(self.pose_estimates[k][2][0])])
            x = int(self.fish_positions[k,0])
            y = int(self.fish_positions[k,1])
            cv2.circle(frame,(x,y),5,(0,255,0),-1)

        print self.fish_positions

        #show results (for debug only)
        cv2.imshow('detected',frame)
        cv2.waitKey(1)

        self.timenow = rospy.Time.now()
        rows,cols,depth = frame.shape
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
