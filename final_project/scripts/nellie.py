#import audio.py
#import voice_commands.py
#import color_recognition.py

import rospy
import numpy as np
import argparse
import cv2
import cv
import time
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
import pyaudio
import speech_recognition as sr

class Nellie():

    def __init__(self):
        rospy.init_node('nellie', anonymous = True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/camera/image_raw', Image, self.camera)
        self.sub = rospy.Subscriber('scan', LaserScan, self.laser)

        self.bridge = CvBridge()
        self.cv_image = Image()

        self.cx = 0
        self.turn=0
        self.vel=0
        self.obstacle=False
        self.seeColor=False
        self.redAction=False

        self.right = 5
        self.frontright = 5
        self.front = 5
        self.frontleft = 5
        self.left = 5
        self.view = []


    def camera(self, msg):
        #color recognition code
        if self.obstacle is False:
            try:
                self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError, e:
                print e

            # using a BGR range to detect red
            # lowerR = np.array([2,0,85], "uint8")
            # upperR = np.array([65,71,215], "uint8")
            lowerR = np.array([0,0,150], "uint8")
            upperR = np.array([50,50,255], "uint8")       

            # using a BGR range to detect blue
            # lowerB = np.array([60, 16, 0], "uint8")
            # upperB = np.array([235, 103, 65], "uint8")
            # lowerB = np.array([60, 0, 0], "uint8")
            # upperB = np.array([255, 50, 50], "uint8")

            # using inRange to detect red stop signal
            redStop = cv2.inRange(self.cv_image, lowerR, upperR)
            redStopImage = cv2.bitwise_and(self.cv_image, self.cv_image, mask = redStop)

            # using inRange to detect blue turn signal
            # blueTurn = cv2.inRange(self.cv_image, lowerB, upperB)
            # blueTurnImage = cv2.bitwise_and(self.cv_image, self.cv_image, mask = blueTurn)

            # finding contours for red and blue blobs
            contoursR, hierarchyR = cv2.findContours(redStop, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            # contoursB, hierarchyB = cv2.findContours(blueTurn, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            
            # finding contour with maximum area and store it as best_cnt
            max_area = 0
            best_cnt = None

            # is red closest?
            for cntR in contoursR:
                areaR = cv2.contourArea(cntR)

                if areaR > max_area and areaR > 1000:
                    max_area = areaR
                    best_cnt = cntR
                    signal_img = redStopImage
                    print "red"
                    self.turn=0
                    self.vel=0.1
                    self.seeColor=True
                    if self.redAction is False:
                        time.sleep(2)
                        self.redAction=True
                    msg=Twist(Vector3(self.vel,0.0,0.0),Vector3(0.0,0.0,self.turn))
                    self.pub.publish(msg)

            # #or is blue closest?
            # for cntB in contoursB:
            #     areaB = cv2.contourArea(cntB)
            #     if areaB > max_area and areaB > 1000:
            #         max_area = areaB
            #         best_cnt = cntB
            #         signal_img = blueTurnImage
            #         print "blue"
            #         self.turn=0.5
            #         self.vel=0
            #         self.seeColor=True
            #         self.redAction=False
            #         msg=Twist(Vector3(self.vel,0.0,0.0),Vector3(0.0,0.0,self.turn))
            #         self.pub.publish(msg)
            if best_cnt is None: 
                pass
                # msg = Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0))
                # self.pub.publish(msg)
            # if there is a blob.
            else:
                # finding centroids of best_cnt and draw a circle there
                M = cv2.moments(best_cnt)
                self.cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                cv2.circle(signal_img,(self.cx,cy),5,255,-1)
                cv2.imshow('Original', np.hstack([self.cv_image, signal_img]))

    def laser(self, msg):
        valid_ranges = []
        laserscan = []

        #Checks to see if what's in front is valid
        for i in range(45):
            if msg.ranges[i+314] > 0:
                valid_ranges.append(msg.ranges[i+314])
            if msg.ranges[45-i] > 0:
                valid_ranges.append(msg.ranges[45-i])
        
        if len(valid_ranges) > 0:
            length=len(valid_ranges)
            self.left=sum(valid_ranges[0:length/5])/(float(length)/5)
            self.frontleft=sum(valid_ranges[length/5:length/5*2])/(float(length)/5)
            self.front=sum(valid_ranges[length/5*2:length/5*3])/(float(length)/5)
            self.frontright=sum(valid_ranges[length/5*3:length/5*4])/(float(length)/5)
            self.right=sum(valid_ranges[length/5*4:length])/(float(length)/5)
            self.view=[self.left,self.frontleft,self.front,self.frontright,self.right]
        else:
            print "I'm blind"

        # print "right"
        # print self.right
        # print "frontright"
        # print self.frontright
        # print "front"
        # print self.front
        # print "frontleft"
        # print self.frontleft
        # print "left"
        # print self.left
        # print " "

        if (min(self.view) < .7) and (self.front > 0):
            print "Obstacle!"
            # print "MIN"
            # print self.view.index(min(self.view))
            if self.view.index(min(self.view)) > 2:
                self.turn=0
            else:
                self.turn=0
            self.vel=0
            self.obstacle=True
            msg=Twist(Vector3(self.vel,0.0,0.0),Vector3(0.0,0.0,self.turn))
            self.pub.publish(msg)
            #time.sleep(.1)
        # else:
        #     print "No obstacle"
        #     # self.turn=0
        #     # self.vel=0
        #     self.obstacle=False
        #     # msg=Twist(Vector3(self.vel,0.0,0.0),Vector3(0.0,0.0,self.turn))
        #     # self.pub.publish(msg)

    def run(self):
        while self.seeColor == False and self.obstacle == False:
            self.audio = audio(self)
        r = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            cv2.waitKey(3)
            r.sleep()

def audio(self):

    r = sr.Recognizer()
    with sr.Microphone() as source:
        audio = r.listen(source)
        command = r.recognize(audio)

    if command == "go forward":
        # Code to go straight
        msg = Twist(Vector3(0.2,0.0,0.0),Vector3(0.0,0.0,0.0))
        self.pub.publish(msg)
    elif command == "go back":
        # Code to go backwards
        msg = Twist(Vector3(-0.2,0.0,0.0),Vector3(0.0,0.0,0.0))
        self.pub.publish(msg)
    elif command == "turn left":
        # Code to turn left
        msg = Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.2))
        self.pub.publish(msg)
    elif command == "turn right":
        # Code to turn right
        msg = Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,-0.2))
        self.pub.publish(msg)
    elif command == "stop":
        # Code to stop
        msg = Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0))
        self.pub.publish(msg)
    elif command == "quit":
        # Quit code
        quit()


if __name__ == '__main__':
    try:
        node = Nellie()
        node.run()
    except rospy.ROSInterruptException: pass
