# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
import cv2
import numpy as np
from matplotlib import pyplot as plt
import argparse

# parse argument
#ap = argparse.ArgumentParser()
#ap.add_argument("-i", "--image", required = True, help = "Path to the image")
#args = vars(ap.parse_args())

#img = cv2.imread(args['35248551.jpg'])
img = cv2.imread('/home/jean-luc/Documents/MAV/AE4317_2019_datasets/cyberzoo_poles/20190121-135009/77078115.jpg',0)
edges = cv2.Canny(img,20,30)

#
#def callback(foo):
#    pass

## create windows and trackbar
#cv2.namedWindow('parameters')
#cv2.createTrackbar('threshold1', 'parameters', 0, 255, callback)  # change the maximum to whatever you like
#cv2.createTrackbar('threshold2', 'parameters', 0, 255, callback)  # change the maximum to whatever you like
#cv2.createTrackbar('apertureSize', 'parameters', 0,2, callback)
#cv2.createTrackbar('L1/L2', 'parameters', 0, 1, callback)
#
#while(True):
#    # get threshold value from trackbar
#    th1 = cv2.getTrackbarPos('threshold1', 'parameters')
#    th2 = cv2.getTrackbarPos('threshold2', 'parameters')
#    
#    # aperture size can only be 3,5, or 7
#    apSize = cv2.getTrackbarPos('apertureSize', 'parameters')*2+3
#    
#    # true or false for the norm flag
#    norm_flag = cv2.getTrackbarPos('L1/L2', 'parameters') == 1
#    
#    # print out the values
#    print('')
#    print('threshold1: {}'.format(th1))
#    print('threshold2: {}'.format(th2))
#    print('apertureSize: {}'.format(apSize))
#    print('L2gradient: {}'.format(norm_flag))
#    
#    edge = cv2.Canny(img, th1, th2, apertureSize=apSize, L2gradient=norm_flag)
#    cv2.imshow('canny', edge)
#    
#    if cv2.waitKey(1)&0xFF == ord('q'):
#        break
#        
#cv2.destroyAllWindows()

plt.subplot(121),plt.imshow(img,cmap = 'gray')
plt.title('Original Image'), plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(edges,cmap = 'gray')
plt.title('Edge Image'), plt.xticks([]), plt.yticks([])

plt.show()