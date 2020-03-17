#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 15 12:25:00 2020

@author: hakim
"""

#import numpy as np
import cv2
from matplotlib import pyplot as plt

img = cv2.imread('image_172.jpg',0)

# Initiate STAR detector
orb = cv2.ORB()
# find the keypoints with ORB
kp = orb.detect(img,None)

# compute the descriptors with ORB
kp, des = orb.compute(img, kp)

# draw only keypoints location,not size and orientation
img2 = cv2.drawKeypoints(img,kp,color=(0,255,0), flags=0)
plt.imshow(img)
plt.imshow(img2),plt.show()

print(kp)

