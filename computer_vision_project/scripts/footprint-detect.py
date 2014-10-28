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
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class FootprintFinder():

    def __init__(self):
        rospy.init_node('footprintfinder', anonymous = True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/camera/image_raw', Image, self.collect_image)

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
        lower = np.array([17,15,100], "uint8")
        upper =np.array([50,56,200], "uint8")

        blob = cv2.inRange(cv_image, lower, upper)
        blob_img = cv2.bitwise_and(cv_image, cv_image, mask = blob)

        cv2.imshow("Original and Blobbed", np.hstack([cv_image, blob_img]))
        cv2.waitKey(3)

    def run(self):

        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            # msg = Twist(Vector3(vel,0.0,0.0),Vector3(0.0,0.0,turning))
            # pub.publish(msg)
            r.sleep()



if __name__ == '__main__':
    try:
        node = FootprintFinder()
        node.run()
    except rospy.ROSInterruptException: pass