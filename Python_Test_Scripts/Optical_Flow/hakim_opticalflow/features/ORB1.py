#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 15 13:28:46 2020

@author: hakim
"""

import numpy as np
import cv2
from matplotlib import pyplot as plt

img2 = cv2.imread('image_sequence_pure_ver1/image_40.jpg',0)

orb = cv2.ORB_create(edgeThreshold=5, patchSize=30, nlevels=8, fastThreshold=8, scaleFactor=1.2, WTA_K=1,scoreType=cv2.ORB_HARRIS_SCORE, firstLevel=0, nfeatures=5000)
kp2 = orb.detect(img2)
img2_kp = cv2.drawKeypoints(img2, kp2, None, color=(0,255,0), \
        flags=cv2.DrawMatchesFlags_DEFAULT)

plt.figure()
plt.imshow(img2_kp)
plt.show()

pp=np.ones((len(kp2),2));

for i,keypoint in enumerate(kp2):
    pp[i,0]=keypoint.pt[0];
    pp[i,1]=keypoint.pt[1];
    
  #  print (keypoint.pt)
