#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 10 14:13:27 2020

@author: ziemersky
"""

import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
filename = '/home/ziemersky/Documents/Autonomous_Flight_of_Micro_Air_Vehicles/AE4317_2019_datasets/cyberzoo_aggressive_flight_bottomcam/20190121-143427/257211470.jpg'
img = cv.imread(filename)
gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
gray = np.float32(gray)
dst = cv.cornerHarris(gray,2,3,0.04)
#result is dilated for marking the corners, not important
dst = cv.dilate(dst,None)
# Threshold for an optimal value, it may vary depending on the image.
img[dst>0.01*dst.max()]=[0,0,255]
plt.imshow(img),plt.show()
#cv.imshow('dst',img)
#if cv.waitKey(0) & 0xff == 27:
#    cv.destroyAllWindows()