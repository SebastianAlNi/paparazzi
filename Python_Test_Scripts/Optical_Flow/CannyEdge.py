#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 10 13:25:16 2020

@author: ziemersky
"""

import cv2
import numpy as np
from matplotlib import pyplot as plt

def imageblur(img):
    kernel = np.ones((5,5),np.float32)/25
    dst = cv2.filter2D(img,-1,kernel)
    plt.subplot(121),plt.imshow(img),plt.title('Original')
    plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(dst),plt.title('Averaging')
    plt.xticks([]), plt.yticks([])
    plt.show()
    return img

img = cv2.imread('/home/ziemersky/Documents/Autonomous_Flight_of_Micro_Air_Vehicles/AE4317_2019_datasets/cyberzoo_aggressive_flight_bottomcam/20190121-143427/257211470.jpg',0)
#img = imageblur(img)
#img = blur = cv2.GaussianBlur(img,(5,5),0)
img = cv2.bilateralFilter(img,9,75,75)
edges = cv2.Canny(img,10,200)

plt.subplot(121),plt.imshow(img,cmap = 'gray')
plt.title('Original Image'), plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(edges,cmap = 'gray')
plt.title('Edge Image'), plt.xticks([]), plt.yticks([])

plt.show()