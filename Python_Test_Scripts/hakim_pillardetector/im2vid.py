#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 11 18:03:42 2020

@author: hakim
"""

import cv2
import os
from tqdm import tqdm
import glob
#TODO
image_folder = 'pics'
video_name = 'videor.mp4'#save as .avi
#is changeable but maintain same h&w over all  frames
width=640 
height=400 
#this fourcc best compatible for avi
fourcc = cv2.VideoWriter_fourcc('M','J','P','G')
video=cv2.VideoWriter(video_name,fourcc, 2.0, (width,height))



for i in tqdm((sorted(glob.glob(image_folder),key=os.path.getmtime))):
     x=cv2.imread(i)
     video.write(x)

cv2.destroyAllWindows()
video.release()