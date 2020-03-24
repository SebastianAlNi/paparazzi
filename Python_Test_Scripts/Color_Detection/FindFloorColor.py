#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 12 12:11:43 2020

@author: ziemersky
"""

import cv2
import matplotlib.pyplot as plt
import numpy as np

def YUVfromRGB(rgb):
    yuv = np.zeros(3)
    yuv[0] =  0.257 * rgb[0] + 0.504 * rgb[1] + 0.098 * rgb[2] +  16
    yuv[1] = -0.148 * rgb[0] - 0.291 * rgb[1] + 0.439 * rgb[2] + 128
    yuv[2] =  0.439 * rgb[0] - 0.368 * rgb[1] - 0.071 * rgb[2] + 128
  
    return yuv

img = cv2.imread('/home/ziemersky/paparazzi/AE4317_2019_datasets/cyberzoo_bottomcam/20190121-152231/41570128.jpg');

average = img.mean(axis=0).mean(axis=0)
average_yuv = YUVfromRGB(average)
avg_patch = np.ones(shape=img.shape, dtype=np.uint8)*np.uint8(average)
avg_patch_yuv = np.ones(shape=img.shape, dtype=np.uint8)*np.uint8(average_yuv)
print(average)
print(average_yuv)
plt.imshow(avg_patch_yuv)
cv2.imwrite('output.jpg',avg_patch)