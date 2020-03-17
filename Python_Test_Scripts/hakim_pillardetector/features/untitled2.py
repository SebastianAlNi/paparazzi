#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 17 00:03:32 2020

@author: hakim
"""

import cv2
import numpy as np

img = cv2.imread('image_sequence_pure_ver1/image_50.jpg',0)
kernel = np.ones((5,5),np.uint8)
erosion = cv2.erode(img,kernel,iterations = 1)
cv2.imshow("Source",erosion)