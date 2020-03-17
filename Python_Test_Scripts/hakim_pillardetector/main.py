"""
Created on Tue Mar 17 22:21:19 2020

@author: hakim
"""

import numpy as np
import extract_information_flow_field as OF
import matplotlib.pyplot as plt

 #if not os.path.exists('pics'): 
        
# Change the image numbers below to answer the questions:
kk=1;
dd = np.zeros((2200,1))  
cc = np.zeros((2201,1))  


for x in range(1,428):

    img_nr_1 = x;
    img_nr_2 = x+1;
    points_old, points_new, flow_vectors,c, d = OF.show_flow(x,img_nr_1, img_nr_2);
    #dd[kk,0]=d;
    #cc[kk,0]=c; 
    kk=kk+1;



