#!/usr/bin/env python
# Software License Agreement (BSD License)

# Object detection using color
# Finds red monsters and runs them over.
# Stops when it finds a blue blob

import rospy
import numpy as np
import argparse
import cv2
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan

class MonsterTracker():

    def __init__(self):
        rospy.init_node('monstertracker', anonymous = True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/camera/image_raw', Image, self.collect_image)
        self.sub = rospy.Subscriber('scan', LaserScan, self.blueblob)

        self.bridge = CvBridge()
        self.cv_image = Image()
        self.motion = True
        self.cx = 0

    def collect_image(self, msg):
        # msg is of type sensor_msgs/Image

        # use cv bridge to convert from ros images to cv images
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print e

        # using a BGR range to detect red
        lowerR = np.array([2,0,85], "uint8")
        upperR = np.array([65,71,215], "uint8")        

        # using a BGR range to detect blue
        lowerB = np.array([60, 16, 0], "uint8")
        upperB = np.array([235, 103, 65], "uint8")

        # using inRange to detect red blobs
        blobR = cv2.inRange(self.cv_image, lowerR, upperR)
        blob_img_R = cv2.bitwise_and(self.cv_image, self.cv_image, mask = blobR)

        # using inRange to detect blue blobs
        blobB = cv2.inRange(self.cv_image, lowerB, upperB)
        blob_img_B = cv2.bitwise_and(self.cv_image, self.cv_image, mask = blobB)


        # finding contours for red and blue blobs
        contoursR, hierarchyR = cv2.findContours(blobR, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        contoursB, hierarchyB = cv2.findContours(blobB, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        # finding contour with maximum area and store it as best_cnt
        max_area = 0
        best_cnt = None

        # is the biggest blob red?
        for cntR in contoursR:
            areaR = cv2.contourArea(cntR)

            if areaR > max_area:
                max_area = areaR
                best_cnt = cntR
                blob_img = blob_img_R
                self.motion = True

        # or is the biggest blob blue?
        for cntB in contoursB:
            areaB = cv2.contourArea(cntB)
            if areaB > max_area:
                max_area = areaB
                best_cnt = cntB
                blob_img = blob_img_B
                self.motion = False

        # don't move if there is no blob
        if best_cnt is None:
            msg = Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0))
            self.pub.publish(msg)
        # if there is a blob...
        else:
            # finding centroids of best_cnt and draw a circle there
            M = cv2.moments(best_cnt)
            self.cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            cv2.circle(blob_img,(self.cx,cy),5,255,-1)

            # we found a centroid! display it.
            cv2.imshow('Original', np.hstack([self.cv_image, blob_img]))

            # self.motion is True if the NEATO sees red

            if self.motion is True:
                # see where the blob is in relation to the NEATO
                # turn and approach based on location of the blob
                if self.cx < 207: # turn right
                    msg = Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.1))
                    self.pub.publish(msg)        
                elif self.cx > 414: # turn left
                    msg = Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,-0.1))
                    self.pub.publish(msg)        
                else: # go straight
                    msg = Twist(Vector3(0.3,0.0,0.0),Vector3(0.0,0.0,0.0))
                    self.pub.publish(msg)

    def blueblob(self, msg):

        ff = [] # front front
        laserscan = []
        
        # averaging the laser scan points in front of the NEATO
        for i in range(30):
            if msg.ranges[329+i] > 0:
                ff.append(msg.ranges[329+i])
            if msg.ranges[30-i] > 0:
                ff.append(msg.ranges[30-i])
        if len(ff) > 0:
            laserscan.append(sum(ff)/float(len(ff)))
        else:
            laserscan.append(float(0))

        # distance between NEATO and blob
        blobdistance = sum(laserscan)/float(len(laserscan))
        
        # self.motion is False if the NEATO sees blue
        if self.motion is False:

            # stop if the NEATO is within half a meter of the blue blob
            if (blobdistance < .5) and (blobdistance > 0):
                print "stop"
                msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
                self.pub.publish(msg)
            else:
                # see where the blob is in relation to the NEATO
                # turn and approach based on location of the blob                
                if self.cx < 207: # turn right
                    msg = Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.1))
                    self.pub.publish(msg)        
                elif self.cx > 414: # turn left
                    msg = Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,-0.1))
                    self.pub.publish(msg)        
                else: # go straight
                    msg = Twist(Vector3(0.3,0.0,0.0),Vector3(0.0,0.0,0.0))
                    self.pub.publish(msg)

    def run(self):

        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            cv2.waitKey(3)
            r.sleep()

if __name__ == '__main__':
    try:
        node = MonsterTracker()
        node.run()
    except rospy.ROSInterruptException: pass