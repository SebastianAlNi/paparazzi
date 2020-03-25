#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 12 12:11:43 2020

@author: ziemersky
"""

import cv2
import matplotlib.pyplot as plt
import numpy as np

def rescale(img, scale_percent):
    # Scale image resolution
    #scale_percent = 100 # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    # resize image
    scaled = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    return scaled

img_front = cv2.imread('/home/ziemersky/paparazzi/AE4317_2019_datasets/sim_poles_panels/20190121-161422/42987000.jpg');
img_floor = cv2.imread('/home/ziemersky/paparazzi/AE4317_2019_datasets/cyberzoo_bottomcam/20190121-152231/39351172.jpg');

# Text in image options
# font 
font = cv2.FONT_HERSHEY_SIMPLEX 
  
# org 
org = (5, 15) 
  
# fontScale 
fontScale = 0.35
   
# color in BGR 
color = (255, 255, 255) 
  
# Line thickness of 2 px 
thickness = 1

for num in range(1):
    directory = 'Frame_Series_Mats/'
    filename = directory+'frame_'+str(num+1)+'.jpg'
    filename = '/home/ziemersky/paparazzi/AE4317_2019_datasets/sim_poles_panels/20190121-161422/42987000.jpg'
    img_front = cv2.imread(filename)
    #plt.imshow(img_front)
    
    img_front = img_front[0:520, 0:150]
    #img_front_blur = cv2.blur(img_front,(15,15))
    img_front_scaled = rescale(img_front, 50)
    # Convert BGR to HSV
    # Use YUV instead!!
    #img_front_hsv = cv2.cvtColor(img_front_scaled, cv2.COLOR_BGR2HSV)
    img_front_YCrCb = cv2.cvtColor(img_front_scaled, cv2.COLOR_RGB2YCrCb)
    
    #green = np.uint8([[[0,255,0 ]]])
    #hsv_green = cv2.cvtColor(green,cv2.COLOR_BGR2HSV)
    #print(hsv_green)
    
    #lower_green_hsv = np.array([15,0,0])
    #upper_green_hsv = np.array([40,255,200])
    #lower_green_YCrCb = np.array([65,120,110])
    #upper_green_YCrCb = np.array([110,132,130])
    lower_green_YCrCb = np.array([0,0,0])
    upper_green_YCrCb = np.array([255,110,130])
    
    #mask = cv2.inRange(img_front_hsv, lower_green_hsv, upper_green_hsv)
    mask = cv2.inRange(img_front_YCrCb, lower_green_YCrCb, upper_green_YCrCb)
    print(mask[0])
    #edges = cv2.Canny(mask,100,200)
    edges = cv2.Canny(mask,600,1500) # not necessary, following steps can also be done on mask
    #edges = mask
    #print(edges)
    
    #vectors = cv2.findContours(edges, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE, offset=(0, 0))
    #print(edges)
    #print(vectors)
    #plt.imshow(vectors,cmap = 'gray')
    #edges = cv2.fastNlMeansDenoising(edges, edges, 50) #long computation
    
    #erosion_size = 5
    #element = cv2.getStructuringElement(cv2.MORPH_RECT,(100,100))
    #print(element)
    #cv2.dilate(edges_init, edges, element);
    #print(edges[300])
    #print(len(edges))
    
    green_threshold = 0.8
    count_green_columns = 0
    green_column_min_index = 0
    green_column_max_index = 0
    #print(len(edges))
    #print(int((len(edges)-1)/2))
    
    bool_green = np.zeros(len(edges))
    sum_green_true = 0
    sum_green_false = 0
    
    for h in range(len(edges)):
        if sum(edges[h]) > 0:
            bool_green[h] = True
            sum_green_true = sum_green_true + 1
        else:
            bool_green[h] = False
            sum_green_false = sum_green_false + 1
            
    #print(bool_green)
    
    for i in range(len(edges)):
        if bool_green[i] == True:# and edges[i][0] == 1:
        #if edges[i][0] == 1:
            green_column_min_index = i
            break
            
    for j in range(len(edges)-1, 0, -1):
        if bool_green[j] == True:# and edges[i][0] == 1:
        #if edges[j][0] == 1:
            green_column_max_index = j
            break
    
    sum_indices_obst = 0
    sum_indices_green = 0
    
    for k in range(green_column_min_index, green_column_max_index+1):
        if bool_green[k] == True:
            count_green_columns = count_green_columns + 1
            sum_indices_green = sum_indices_green + k
        else:
            sum_indices_obst = sum_indices_obst + k
    
    #print(sum_indices_green)
    green_length = green_column_max_index - green_column_min_index
    if green_length != 0:
        ratio_green = count_green_columns / green_length
    else:
        ratio_green = 0
    count_obstacle_pixels = green_length - count_green_columns
    if count_obstacle_pixels != 0:
        cog_obst = sum_indices_obst / count_obstacle_pixels
    else:
        cog_obst = 0
    if count_green_columns != 0:
        cog_green = sum_indices_green / count_green_columns
    else:
        cog_green = 0
    
    '''print('green_column_min_index:', green_column_min_index)
    print('green_column_max_index:', green_column_max_index)
    print('count_green_columns:', count_green_columns)
    print('count_obstacle_pixels:', count_obstacle_pixels)
    print('green_length:', green_length)
    print('ratio_green:', ratio_green)
    print('cog_obst:', cog_obst)
    print('cog_green:', cog_green)
    print('image center:', len(edges)/2)'''
    
    if ratio_green == 0:
        #print('no floor detected, turn around')
        img_front = cv2.putText(img_front_scaled, 'no floor detected', org, font, fontScale, color, thickness, cv2.LINE_AA)
    elif green_column_max_index <= len(edges)/2:
        img_front = cv2.putText(img_front_scaled, 'turn left', org, font, fontScale, color, thickness, cv2.LINE_AA)
    elif green_column_min_index > len(edges)/2:
        img_front = cv2.putText(img_front_scaled, 'turn right', org, font, fontScale, color, thickness, cv2.LINE_AA) 
    elif ratio_green > green_threshold:
        #print('no close obstacle')
        img_front = cv2.putText(img_front_scaled, 'no close obstacle', org, font, fontScale, color, thickness, cv2.LINE_AA) 
        #print('close obstacle')# perhaps check cog of green and obstacles to increase confidence
    elif cog_obst >= len(edges)/2:
        #print('turn left')
        img_front = cv2.putText(img_front_scaled, 'turn left', org, font, fontScale, color, thickness, cv2.LINE_AA) 
    elif cog_obst < len(edges)/2:
        #print('turn right')
        img_front = cv2.putText(img_front_scaled, 'turn right', org, font, fontScale, color, thickness, cv2.LINE_AA) 
    else:
        img_front = cv2.putText(img_front_scaled, 'None', org, font, fontScale, color, thickness, cv2.LINE_AA) 

    #minLineLength = 10
    #maxLineGap = 1
    #lines = cv2.HoughLinesP(edges,1,np.pi/180,85,minLineLength,maxLineGap)
    #for i in range(len(lines)):
    #    for x1,y1,x2,y2 in lines[i]:
    #        cv2.line(img_front,(x1,y1),(x2,y2),(0,255,0),2)
            
    #print(lines)    
    
    #res = cv2.bitwise_and(img_front,img_front, mask= mask)
    #res = cv2.cvtColor(res, cv2.COLOR_HSV2BGR)
    
    
    '''COLOR_OBJECT_DETECTOR_LUM_MIN2 = 65
    COLOR_OBJECT_DETECTOR_LUM_MAX2 = 110
    COLOR_OBJECT_DETECTOR_CB_MIN2 = 110
    COLOR_OBJECT_DETECTOR_CB_MAX2 = 130
    COLOR_OBJECT_DETECTOR_CR_MIN2 = 120
    COLOR_OBJECT_DETECTOR_CR_MAX2 = 132'''
    
    #img_floor = rescale(img_floor, 5)
    
    #floor_count_frac = 0.8
    #floor_count_threshold = floor_count_frac * img_floor.size
    #print(floor_count_threshold)
    
    #if(color_count < color_count_threshold){
    #    obstacle_free_confidence++;
    #  } else {
    #    obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
    #  }
    
    #blur_floor = cv2.blur(img_floor,(3,3))
    #scaled = rescale(img_floor, 10)
    
    #plt.imshow(img_front)
    #plt.imshow(img_floor)
    #plt.imshow(mask)
    #img_front_hsv = cv2.cvtColor(img_front_hsv, cv2.COLOR_HSV2BGR)
    #plt.imshow(mask)
    #cv2.imwrite('original_front.jpg',img_front_hsv)
    edges = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR) # convert canny image to bgr
    res = cv2.addWeighted(img_front_scaled,1.0,edges,1.0,0)
    cv2.imwrite(directory+'out_frame'+str(num+1)+'.jpg',res)
    
    plt.subplot(121),plt.imshow(img_front,cmap = 'gray')
    plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(mask,cmap = 'gray')
    plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
