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

class Saver:
    def __init__(self):

        direc = rospy.get_param('output_dir','')
        bagname = rospy.get_param('bag','')
        self.fishnum = str(rospy.get_param('fishnum','1'))
        self.bagchop = bagname[0:-4]
        direc = direc+"/"+self.bagchop+'_auto'

        if not os.path.exists(direc):
            os.makedirs(direc)

        self.measurefile_path = direc+'/'+self.bagchop+'_'+'output_fish'+self.fishnum+'_measured.txt'
        self.kalmanfile_path = direc+'/'+self.bagchop+'_'+'output_fish'+self.fishnum+'_kalman.txt'
        self.anglefile_path=direc+'/'+self.bagchop+'_'+'output_fishangle'+self.fishnum+'.txt'
        
        
        self.measuredf = open(self.measurefile_path,'wb')
        self.kalmanf = open(self.kalmanfile_path,'wb')
        self.anglef = open(self.anglefile_path,'wb')

        self.anglesub = rospy.Subscriber("/fishtracker/measuredangle",Float32Stamped,self.anglecallback,queue_size=1)#change this to proper name!
        self.measuredsub = rospy.Subscriber("/fishtracker/measuredfishpose",PoseStamped,self.measuredsub,queue_size=1)
        self.kalmansub = rospy.Subscriber("/fishtracker/kalmanfishpose",PoseStamped,self.kalmansub,queue_size=1)

    def anglecallback(self,data):
        self.anglef.write(str(data.header.stamp)+"\t"+str(data.header.seq)+"\t"+str(data.data)+"\r\n")

    def measuredsub(self,data):
        self.measuredf.write(str(data.header.stamp)+"\t"+str(data.pose.position.x)+"\t"+str(data.pose.position.y)+"\t"+str(data.pose.position.z)+"\r\n")

    def kalmansub(self,data):
        self.kalmanf.write(str(data.header.stamp)+"\t"+str(data.pose.position.x)+"\t"+str(data.pose.position.y)+"\t"+str(data.pose.position.z)+"\r\n")

def main(args):
  
  rospy.init_node('saver', anonymous=True)
  ic = Saver()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)