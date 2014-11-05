#!/usr/bin/env python
# Software License Agreement (BSD License)


# Detects footprints on the ground.


# import roslib
# roslib.load_manifest('my_package')
# import sys

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

class FootprintFinder():

    def __init__(self):
        rospy.init_node('footprintfinder', anonymous = True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/camera/image_raw', Image, self.collect_image, queue_size=1)

        self.bridge = CvBridge()


    def collect_image(self, msg):
        # msg is of type sensor_msgs/Image
        # use cv bridge to convert from ros images to cv images

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print e

        # displays the image

            # Tried to use SimpleBlobDetector but it didn't work
            # bd = cv2.SimpleBlobDetector()
            # bd.setInt('blobColor', 0)        
            # blob = bd.detect(cv_image)
            # blob_img = cv2.drawKeypoints(cv_image,blob,None,(255,0,0),4)

        # in range and find contours
        # use cvtColor to change to HSV colors if necessary


        # Guessing at our pink feet
        # lower = np.array([150,0,180], "uint8")
        # upper =np.array([204,81,255], "uint8")

        # Red from train example
        lowerR = np.array([17,15,100], "uint8")
        upperR = np.array([50,56,200], "uint8")

        lowerB = np.array([86, 31, 4], "uint8")
        upperB = np.array([220, 88, 50], "uint8")


        blobR = cv2.inRange(cv_image, lowerR, upperR)
        blob_img_R = cv2.bitwise_and(cv_image, cv_image, mask = blobR)

        blobB = cv2.inRange(cv_image, lowerB, upperB)
        blob_img_B = cv2.bitwise_and(cv_image, cv_image, mask = blobB)

        # cv2.imshow("Original and Blobbed", np.hstack([cv_image, blob_img]))
        # cv2.waitKey(3)

        contoursR, hierarchyR = cv2.findContours(blobR, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        contoursB, hierarchyB = cv2.findContours(blobB, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        # finding contour with maximum area and store it as best_cnt
        max_area = 1
        best_cnt = None

        for cntR in contoursR:
            areaR = cv2.contourArea(cntR)
            if areaR > max_area:
                max_area = areaR
                best_cnt = cntR
                print best_cnt
                blob_img = blob_img_R

        for cntB in contoursB:
            areaB = cv2.contourArea(cntB)
            if areaB > max_area:
                max_area = areaB
                best_cnt = cntB
                blob_img = blob_img_B

        if best_cnt is None:
            msg = Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0))
            self.pub.publish(msg)
        else:

            # finding centroids of best_cnt and draw a circle there
            M = cv2.moments(best_cnt)
            cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            cv2.circle(blob_img,(cx,cy),5,255,-1)

            # Show it, if key pressed is 'Esc', exit the loop
            cv2.imshow('Original', np.hstack([cv_image, blob_img]))
#            cv2.waitKey(3)

            if cx < 207: # turn right
                msg = Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.1))
                self.pub.publish(msg)        
            elif cx > 414: # turn left
                msg = Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,-0.1))
                self.pub.publish(msg)        
            else: # go straight
                msg = Twist(Vector3(0.2,0.0,0.0),Vector3(0.0,0.0,0.0))
                self.pub.publish(msg)

    def run(self):

        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            # msg = Twist(Vector3(vel,0.0,0.0),Vector3(0.0,0.0,turning))
            # pub.publish(msg)
            cv2.waitKey(50)
            r.sleep()

# Sanity check: rosrun  image_view image_view image:=/camera/image_raw


if __name__ == '__main__':
    try:
        node = FootprintFinder()
        node.run()
    except rospy.ROSInterruptException: pass