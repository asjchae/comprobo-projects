import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('doggie.jpeg',0) #Looking at the original image
edges = cv2.Canny(img,100,200) #Finding the edges

#Plotting the two images side by side
plt.subplot(121),plt.imshow(img,cmap = 'gray')
plt.title('Original Image'), plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(edges,cmap = 'gray')
plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
plt.show()