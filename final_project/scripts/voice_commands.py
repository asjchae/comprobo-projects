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
import pyaudio
import speech_recognition as sr





class VoiceCommands():

    def __init__(self):
        rospy.init_node('voicecommands', anonymous = True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # self.sub = rospy.Subscriber('/camera/image_raw', Image, self.collect_image)
        # self.sub = rospy.Subscriber('scan', LaserScan, self.blueblob)

        self.audio = String

    def commands(audio):
        print audio

    def run(self):
        while 1:
            self.audio = mainfunction(self)
            if self.audio == "quit":
                quit()

        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            cv2.waitKey(3)
            r.sleep()

def mainfunction(self):
    r = sr.Recognizer()
    with sr.Microphone() as source:
        audio = r.listen(source)
        command = r.recognize(audio)
    
    if command == "go forward":
        # Code to go straight
        msg = Twist(Vector3(0.5,0.0,0.0),Vector3(0.0,0.0,0.0))
        self.pub.publish(msg)
    elif command == "go back":
        # Code to go backwards
        msg = Twist(Vector3(-0.5,0.0,0.0),Vector3(0.0,0.0,0.0))
        self.pub.publish(msg)
    elif command == "turn left":
        # Code to turn left
        msg = Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.5))
        self.pub.publish(msg)
    elif command == "turn right":
        # Code to turn right
        msg = Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,-0.5))
        self.pub.publish(msg)
    elif command == "stop":
        # Code to stop
        msg = Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0))
        self.pub.publish(msg)
    elif command == "quit":
        # Quit code
        return "quit"
    

if __name__ == '__main__':
    try:
        node = VoiceCommands()
        node.run()
    except rospy.ROSInterruptException: pass