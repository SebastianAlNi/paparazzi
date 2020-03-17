#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 16 15:39:47 2020

@author: hakim
"""
import cv2
import numpy as np
import math

image1 = cv2.imread('image_sequence_pure_ver1/image_10.jpg')
gray=cv2.cvtColor(image1,cv2.COLOR_BGR2GRAY)
dst = cv2.Canny(gray, 50, 200)

lines= cv2.HoughLines(dst, 1, math.pi/180.0, 100, np.array([]), 0, 0)
#lines1 = cv2.HoughLines(image1,1,math.pi/180.0,5)
#lines2 = cv2.HoughLines(image2,1,math.pi/180.0,5)

#lines1 = lines1[0]
#lines2 = lines2[0]

a,b,c = lines.shape
for i in range(a):
    rho = lines[i][0][0]
    theta = lines[i][0][1]
    a = math.cos(theta)
    b = math.sin(theta)
    x0, y0 = a*rho, b*rho
    pt1 = ( int(x0+1000*(-b)), int(y0+1000*(a)) )
    pt2 = ( int(x0-1000*(-b)), int(y0-1000*(a)) )
    cv2.line(image1, pt1, pt2, (0, 0, 255), 2, cv2.LINE_AA)


cv2.imshow('image1',image1)
cv2.waitKey(0)
cv2.destoryAllWindows(0)