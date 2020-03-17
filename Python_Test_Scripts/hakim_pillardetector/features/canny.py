#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 13 14:52:27 2020

@author: hakim
"""

import cv2
import numpy as np

img = cv2.imread('image_18.jpg')
edges = cv2.Canny(img,100,200)
ans = []
highestY = 0
highestX = 0
highestCoordinate = [0, 0]

for x in range(0, edges.shape[0]):
    for y in range(0, edges.shape[1]):
        if edges[x, y] != 0:            
            ans = ans + [[x, y]]
            if highestX < x:
                highestX = x
            if highestY < y:
                highestY = y
                highestCoordinate = [x, y]    
                
                
                