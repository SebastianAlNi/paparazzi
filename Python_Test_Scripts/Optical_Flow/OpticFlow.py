#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 10 15:25:34 2020

@author: ziemersky
"""


#%matplotlib inline
import extract_information_flow_field as OF
import cv2

# Change the image numbers below to answer the questions:
img_nr_1 = cv2.imread('/home/ziemersky/Documents/Autonomous_Flight_of_Micro_Air_Vehicles/AE4317_2019_datasets/cyberzoo_poles/20190121-135009/95277983.jpg');
img_nr_2 = cv2.imread('/home/ziemersky/Documents/Autonomous_Flight_of_Micro_Air_Vehicles/AE4317_2019_datasets/cyberzoo_poles/20190121-135009/96677961.jpg');
#plt.imshow(img_nr_1);
points_old, points_new, flow_vectors = OF.show_flow2(img_nr_1, img_nr_2);
#print(flow_vectors);