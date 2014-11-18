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
        # self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # self.sub = rospy.Subscriber('/camera/image_raw', Image, self.collect_image)
        # self.sub = rospy.Subscriber('scan', LaserScan, self.blueblob)

        self.audio = String

    def commands(audio):
        print audio

    def run(self):
        while 1:
            self.audio = mainfunction()

        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            cv2.waitKey(3)
            r.sleep()

def mainfunction():
    r = sr.Recognizer()
    with sr.Microphone() as source:
        audio = r.listen(source)
    print r.recognize(audio) + " 1"
    return r.recognize(audio)
    

if __name__ == '__main__':
    try:
        node = VoiceCommands()
        node.run()
    except rospy.ROSInterruptException: pass