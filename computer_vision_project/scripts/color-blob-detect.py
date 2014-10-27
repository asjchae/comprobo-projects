import numpy as np
import cv2

def colorBlobDetect():
	b = cv2.SimpleBlobDetector()

	blob = b.detect(img)