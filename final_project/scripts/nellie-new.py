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
from time import gmtime, strftime
import datetime

class Nellie():

    def __init__(self):
        rospy.init_node('nellie', anonymous = True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/camera/image_raw', Image, self.camera)
        self.sub = rospy.Subscriber('scan', LaserScan, self.laser)

        self.bridge = CvBridge()
        self.cv_image = Image()

        self.obstacle=False
        self.seeColor=False

    def camera(self, msg):

        if self.obstacle is False and self.seeColor is False:
            try:
                self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError, e:
                print e

            # using a BGR range to detect red
            lowerR = np.array([0,0,150], "uint8")
            upperR = np.array([50,50,255], "uint8")       

            # using inRange to detect red stop signal
            redStop = cv2.inRange(self.cv_image, lowerR, upperR)
            redStopImage = cv2.bitwise_and(self.cv_image, self.cv_image, mask = redStop)

            # finding contours for red and blue blobs
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
                    self.seeColor = True
            if self.seeColor == True:
                print "here"
                # self.seeColor = False
                # self.seeColor = True
                # Make it calculate two seconds out.
                currentTime = int(datetime.datetime.today().timetuple()[5])
                if int(datetime.datetime.today().timetuple()[5]) == currentTime + 2:
                    print "here2"
                    # msg=Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.5))
                    # self.pub.publish(msg)
                    # self.seeColor = False

    # Stops if it sees an obstacle.
    def laser(self, msg):
        ff = [] # front front
        laserscan = []
        
        # averaging the laser scan points in front of the NEATO
        for i in range(30):
            if msg.ranges[330+i] > 0:
                ff.append(msg.ranges[329+i])
            if msg.ranges[30-i] > 0:
                ff.append(msg.ranges[30-i])
        if msg.ranges[0] > 0:
            ff.append(msg.ranges[0])
        if len(ff) > 0:
            laserscan.append(sum(ff)/float(len(ff)))
        else:
            laserscan.append(float(0))

        # distance between NEATO and obstacle
        if sum(laserscan)/float(len(laserscan)) > 0:
            distance = sum(laserscan)/float(len(laserscan))
            print distance
        
            # stop if the NEATO is within half a meter of obstacle
            if (distance < .3) and (distance > 0):
                self.obstacle = True
                msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
                self.pub.publish(msg)

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