import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

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