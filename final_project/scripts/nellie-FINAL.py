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
        # Initializing Nellie
        rospy.init_node('nellie', anonymous = True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/camera/image_raw', Image, self.camera)
        self.sub = rospy.Subscriber('scan', LaserScan, self.laser)

        # Image variables
        self.bridge = CvBridge()
        self.cv_image = Image()
        self.next_frame_to_process = -1

        # Boolean variables to define state
        self.obstacle = False
        self.color = False

        # Constantly updating velocity and turn variables
        self.vel = 0.0
        self.turn = 0.0

    def camera(self, msg):
        # Color detection function
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print e

        cv2.waitKey(15)
        if self.color is True:
            # Already saw the Stop sign, doesn't need to look for it again
            pass

        elif self.obstacle is False and self.color is False:
            # Check to see if there is a Stop sign
 
            # using a BGR range to detect red      
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
                # Pause before continuing to move
                self.vel = 0.0
                self.turn = 0.0
                msg=Twist(Vector3(self.vel,0.0,0.0),Vector3(0.0,0.0,self.turn))
                self.pub.publish(msg)    
                time.sleep(.5)
                self.vel = 0.2
                self.turn = 0.0
                self.color = False
            else:
                return

    def laser(self, msg):
        # Obstacle avoidance function
        fr = [] # front right
        fl = [] # front left
        distance_l = 0.0
        distance_r = 0.0

        # averaging the laser scan points in front of the NEATO
        for i in range(30):
            if msg.ranges[329+i] > 0:
                fr.append(msg.ranges[329+i])
            if msg.ranges[30-i] > 0:
                fl.append(msg.ranges[30-i])

        if (len(fr)>0 and sum(fr)/float(len(fr))>0) or (len(fl)>0 and sum(fl)/float(len(fl))>0):
            # find the distance to the nearest obstacle
            if len(fr)>0 and sum(fr)/float(len(fr))>0:
                distance_r = sum(fr)/float(len(fr))
            if len(fl)>0 and sum(fl)/float(len(fl))>0:
                distance_l = sum(fl)/float(len(fl))

            # if the obstacle is too close, turn
            if (distance_r < 1) and (distance_r > 0):
                self.vel = 0.0
                self.turn = 0.2
                self.obstacle = True
                print('left')
            elif (distance_l < 1) and (distance_l > 0):
                self.vel = 0.0
                self.turn = -0.2
                self.obstacle = True
                print('right')

            # if it has finished avoiding obstacles, stop and wait for the next command
            elif (distance_l>0.99) and (distance_r>0.99) and (self.obstacle==True):
                self.vel = 0.0
                self.turn = 0.0
                self.obstacle = False

            else:
                self.obstacle = False

    def run(self):
        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            if self.color == False and self.obstacle == False:
                # check if there are any visual cues, if not do the voice recognition
                self.audio = audio(self)
            cv2.waitKey(15)
            msg=Twist(Vector3(self.vel,0.0,0.0),Vector3(0.0,0.0,self.turn))   
            self.pub.publish(msg)
            r.sleep()

def audio(self):
    # Voice recognition function
    r = sr.Recognizer()

    with sr.Microphone() as source:
        my_audio = r.listen(source)
        command = r.recognize(my_audio)

    print command
    if command == "go straight":
        # Code to go straight
        self.vel = 0.2
        self.turn = 0.0
    elif command == "go forward":
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
    elif command == "bad robot":
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
