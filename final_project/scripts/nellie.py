#import audio.py
#import voice_commands.py
#import color_recognition.py

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

class Nellie():

	def __init__(self):
		rospy.init_node('nellie', anonymous = True)
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		self.sub = rospy.Subscriber('/camera/image_raw', Image, self.camera)
		self.sub = rospy.Subscriber('scan', LaserScan, self.laser)
		#self.sub = rospy.Subscriber('voice', Voice, self.audio)

		self.bridge = CvBridge()
		self.cv_image = Image()
		self.motion = True
		self.cx = 0
		self.turn=0
		self.vel=0
		print "I made Nellie"

	def audio(self, msg):
		#ari's voice code
		pass

	def camera(self, msg):
		#color recognition code
		bridge = CvBridge()
		cv_image = Image()

		cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

		# using a BGR range to detect red
		lowerR = np.array([2,0,85], "uint8")
		upperR = np.array([65,71,215], "uint8")        

		# using a BGR range to detect blue
		lowerB = np.array([60, 16, 0], "uint8")
		upperB = np.array([235, 103, 65], "uint8")

		# using inRange to detect red thing
		redThing = cv2.inRange(cv_image, lowerR, upperR)
		redThingImage = cv2.bitwise_and(cv_image, cv_image, mask = redThing)

		# using inRange to detect blue things
		blueThing = cv2.inRange(cv_image, lowerB, upperB)
		blueThingImage = cv2.bitwise_and(cv_image, cv_image, mask = blueThing)

		cv2.imshow('Original', np.hstack([cv_image, redThingImage]))

	def laser(self, msg):
		valid_ranges = []

		#Checks to see if what's in front is valid
		for i in range(60):
			if msg.ranges[i+300]>0.01 and msg.ranges[i+300]<8:
				valid_ranges.append(msg.ranges[i+300])
		for i in range(60):
			if msg.ranges[i]>0.01 and msg.ranges[i]<8:
				valid_ranges.append(msg.ranges[i])
		
		#Checks if there are obstacles in front
		if len(valid_ranges)>0:
			for i in valid_ranges:
				if i<1:
					print "Obstacle!"
					self.turn=-.5
					self.vel=0
					msg=Twist(Vector3(self.vel,0.0,0.0),Vector3(0.0,0.0,self.turn))
					self.pub.publish(msg)
				else: #Stop and check for color/voice command
					print "No obstacle"
					self.turn=0
					self.vel=.1
					msg=Twist(Vector3(self.vel,0.0,0.0),Vector3(0.0,0.0,self.turn))
					self.pub.publish(msg)
					#color(self,msg)
		else:
			"I see nothing"
			self.vel=0
			self.turn=0
			#color(self,msg)

	def run(self):
		r = rospy.Rate(10) # 10hz

		while not rospy.is_shutdown():
			cv2.waitKey(3)
			r.sleep()

if __name__ == '__main__':
	try:
		node = Nellie()
		node.run()
	except rospy.ROSInterruptException: pass