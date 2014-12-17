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
import datetime

class Nellie():

    def __init__(self):
        rospy.init_node('nellie', anonymous = True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/camera/image_raw', Image, self.camera)
        self.sub = rospy.Subscriber('scan', LaserScan, self.laser)

        self.bridge = CvBridge()
        self.cv_image = Image()

        self.obstacle = False
        self.color = False

        self.vel = 0.0
        self.turn = 0.0

    def camera(self, msg):

        if self.color is True:
            # Already saw the Stop sign, doesn't need to look for it again
            pass

        elif self.obstacle is False and self.color is False:
            # Check to see if there is a Stop sign
            try:
                self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError, e:
                print e

            # using a BGR range to detect red
            # lowerR = np.array([0,0,150], "uint8")
            # upperR = np.array([50,50,255], "uint8")       
            lowerR = np.array([2,0,85], "uint8")
            upperR = np.array([65,71,215], "uint8")

            # using inRange to detect red stop signal
            redStop = cv2.inRange(self.cv_image, lowerR, upperR)
            redStopImage = cv2.bitwise_and(self.cv_image, self.cv_image, mask = redStop)

            # finding contours for red
            contoursR, hierarchyR = cv2.findContours(redStop, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            
            # finding contour with maximum area and store it as best_cnt
            max_area = 0
            best_cnt = None

            for cntR in contoursR:
                areaR = cv2.contourArea(cntR)

                if areaR > max_area and areaR > 1000:
                    max_area = areaR
                    best_cnt = cntR
                    signal_img = redStopImage
                    print "red"
                    self.color = True
                    # Make it not take the next frame because lag
                    self.next_frame_to_process = msg.header.seq + 15*5
                    break

            if self.color == True:
                # Stop for two seconds.
                time.sleep(2)
                self.vel = 0.0
                self.turn = 0.4
                time.sleep(5)
                self.vel = 0.0
                self.turn = 0.0
                self.color = False
            else:
                return

    # Stops if it sees an obstacle.
    def laser(self, msg):
        fr = [] # front right
        fl = [] # front left
        distance_l = 0.0
        distance_r = 0.0
        
        # averaging the laser scan points in front of the NEATO
        for i in range(30):
            if msg.ranges[330+i] > 0:
                fr.append(msg.ranges[329+i])
            if msg.ranges[30-i] > 0:
                fl.append(msg.ranges[30-i])
        # if msg.ranges[0] > 0:
        #     ff.append(msg.ranges[0])
        # if len(ff) > 0:
        #     laserscan.append(sum(ff)/float(len(ff)))
        # else:
        #     laserscan.append(float(0))

        # distance between NEATO and obstacle
        if len(fr)>0 and sum(fr)/float(len(fr))>0:
            distance_r = sum(fr)/float(len(fr))
            if (distance_r < .7) and (distance_r > 0):
                self.obstacle = True
                self.vel = 0.0
                self.turn = 0.2
            # if sum(msg.ranges[270:275])/float(5)<0.3:
            #     print sum(msg.ranges[270:275])/float(5)
            #     self.vel = 0.0
            #     self.turn = 0.0
        if len(fl)>0 and sum(fl)/float(len(fl))>0:
            distance_l = sum(fl)/float(len(fl))
        # if sum(laserscan)/float(len(laserscan)) > 0:
        #     distance = sum(laserscan)/float(len(laserscan))

            # turn away from the obstacle if the NEATO is too close
            if (distance_l < .7) and (distance_l > 0):
                self.obstacle = True
                self.vel = 0.0
                self.turn = -0.2
            elif (distance_l>0.69) and (distance_r>0.69) and (self.obstacle==True):
                self.vel = 0.0
                self.turn = 0.0
            else:
                self.obstacle = False

    def run(self):
        # while self.color == False and self.obstacle == False:
        #     self.audio = audio(self)

        r = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            if self.color == False and self.obstacle == False:
                self.audio = audio(self)
            #cv2.waitKey(3)
            msg=Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,self.turn))
            self.pub.publish(msg)
            r.sleep()

def audio(self):
    r = sr.Recognizer()
    with sr.Microphone() as source:
        audio = r.listen(source)
        command = r.recognize(audio)

    print command
    if command == "go straight":
        # Code to go straight
        self.vel = 0.2
        self.turn = 0.0
    elif command == "go back":
        # Code to go backwards
        self.vel = -0.2
        self.turn = 0.0
    elif command == "turn left":
        # Code to turn left
        self.vel = 0.0
        self.turn = 0.2
    elif command == "turn right":
        # Code to turn right
        self.vel = 0.0
        self.turn = -0.2
    elif command == "stop":
        # Code to stop
        self.vel = 0.0
        self.turn = 0.0
    elif command == "quit":
        # Quit code
        quit()


if __name__ == '__main__':
    try:
        node = Nellie()
        node.run()
    except rospy.ROSInterruptException: pass
