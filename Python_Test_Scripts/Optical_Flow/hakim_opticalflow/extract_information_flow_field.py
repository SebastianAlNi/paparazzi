# -*- coding: utf-8 -*-
"""

Created on Wed Mar 07 10:46:05 2018

Script that can be run on a directory, calculates optical flow and extracts useful information from the flow field.

@author: Guido de Croon.
"""
import sys
import math
#import matplotlib.pyplot.hold as hold
import numpy as np
import math
import matplotlib as mpl
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
import re
import time
import sys
import pandas as pd
import PIL
#import pandas as pd

def determine_optical_flow(image_name_1, width, x,prev_bgr, bgr, graphics= True):
    
    # *******************************************************************
    # TODO: In the !second! lecture on optical flow, study this function
    # and change the parameters below to investigate the trade-off between
    # accuracy and computational efficiency
    # *******************************************************************
    
    # convert the images to grayscale:
    prev_gray = cv2.cvtColor(prev_bgr, cv2.COLOR_BGR2GRAY);
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY);
    
    # params for ShiTomasi corner detection
    feature_params = dict( maxCorners = 100,
                           qualityLevel = 0.3,
                           minDistance = 7,
                           blockSize = 7 )
#@file hough_lines.py
#@brief This program demonstrates line finding with the Hough transform

    argv=sys.argv[1:]
    default_file = image_name_1;
    kernel = np.ones((5,5), np.uint8) 

    filename = argv[0] if len(argv) > 0 else default_file
    #filename=prev_gray;
    # Loads an image
    
    
    src = cv2.imread(cv2.samples.findFile(filename), cv2.IMREAD_GRAYSCALE)
    src=  cv2.rotate(src, cv2.ROTATE_90_COUNTERCLOCKWISE)
    img_erosion = cv2.erode(src, kernel, iterations=1) 
    src=img_erosion;
    gray=cv2.erode(gray, kernel, iterations=1);
    # Check if image is loaded fine
    if src is None:
        print ('Error opening image!')
        print ('Usage: hough_lines.py [image_name -- default ' + default_file + '] \n')
    #return -1
    ## [load]
    
    ## [edge_detection]
    # Edge detection
    dst = cv2.Canny(src, 30, 50, None, 3)
    ## [edge_detection]
    
    # Copy edges to the images that will display the results in BGR
    cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
    cdstP = np.copy(cdst)
    
    ## [hough_lines]
    #  Standard Hough Line Transform
    lines = cv2.HoughLines(dst, 1, np.pi / 180, 150, None, 0, 0)
    ## [hough_lines]
    ## [draw_lines]
    # Draw the lines
    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
    
            cv2.line(cdst, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
    ## [draw_lines]
    r=0;
    ## [hough_lines_p]
    # Probabilistic Line Transform
    linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)
    ## [hough_lines_p]
    ## [draw_lines_p]
    # Draw the lines
    
    pp = np.zeros((20,2))  
    
    
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            l1=int(linesP[i][0][0])
            l2=linesP[i][0][1]
            l3=linesP[i][0][2]
            l4=linesP[i][0][3]
    #        cv.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv.LINE_AA)
           # l0=int(math.ceil(l[0] / 10.0)) * 10;
           # l2=int(math.ceil(l[2] / 10.0)) * 10
    #        l[0][0]
            theta=np.arctan((l[3]- l[1])/(l[2]- l[0]))
            #print(theta)
           # l=np.array(list(l))
            if abs(theta)>1.56:
                l=np.expand_dims(l , axis=0);  
                l=np.array(l)
              #  l=l.flatten()
                pp[r,0]=l1
                pp[r+1,0]=l3
                pp[r,1]=l2
                pp[r+1,1]=l4
                r=r+2;
               # print (pp)
            #print (linesP[i][0])
    
       # global pp
    kk= linesP;
    #print(kk)
    #ll =np.expand_dims(pp , axis=0);  
    #point[0,0]=ll[0];
    #point[0,1]=ll[1];
    #point[1,2]=ll[1];
    
#    ll=np.array(list(l))

   
    points_old=pp;
    points_old = points_old.astype('float32') ;
    points_old =np.expand_dims(points_old , axis=1);
   # print(points_old)


    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    """
    #canny edge
   # prev_grey.convertTo(prev_gray, CV_32F)
    edges = cv2.Canny(prev_gray, 20, 30)
    indices = np.where(edges != [0])
    coordinates = zip(indices[0], indices[1])
    coordinates=set(coordinates);
    coordinates=np.array(list(coordinates))
   
    #coordinates=np.float32;
    points_old = coordinates;
    points_old=np.array(points_old,dtype=float);
    points_old = points_old.astype('float32') ;
    #points_old = np.float32(points_old)
    #points_old= np.ndarray.copy(points_old, order='C');
    points_old =np.expand_dims(points_old , axis=1);

  
    
    
    #Blob
    from math import sqrt
    from skimage import data
    from skimage.feature import blob_dog, blob_log, blob_doh
    from skimage.color import rgb2gray
    
    blobs_dog = blob_dog(prev_gray, max_sigma=30, threshold=.1)
    blobs_dog[:, 2] = blobs_dog[:, 2] * sqrt(2)


    blobs_doh = blob_doh(gray, max_sigma=30, threshold=.01)

    blobs_list = [blobs_dog]
    colors = ['yellow', 'lime', 'red']
    titles = [ 'Difference of Gaussian']
 #         'Determinant of Hessian']
    sequence = zip(blobs_list, colors, titles)

  #  fig, axes = plt.subplots(1, 3, figsize=(9, 3), sharex=True, sharey=True)
#    ax = axes.ravel()
    cc=0;
    #pp=np.ones((len(blobs),2))  
  
    for idx, (blobs, colour, title) in enumerate(sequence):
        pp=np.ones((len(blobs),2))     
    #    ax[idx].set_title(title)
    #    ax[idx].imshow(image)
        for blob in blobs:
            y, x, r = blob
            pp[[cc,1]]=y;
            pp[cc,0]=x;
           # c = plt.Circle((x, y), r, color=colors, linewidth=2, fill=False)
           # ax[idx].add_patch(c)
            cc=cc+1;
   #     ax[idx].set_axis_off()
    
    points_old=pp;
    points_old = points_old.astype('float32') ;
    points_old =np.expand_dims(points_old , axis=1);
    

   
    
    #orb
    from matplotlib import pyplot as plt
    
    img2 = prev_bgr;
    
    orb = cv2.ORB_create(edgeThreshold=5, patchSize=30, nlevels=8, fastThreshold=8, scaleFactor=1.2, WTA_K=1,scoreType=cv2.ORB_HARRIS_SCORE, firstLevel=0, nfeatures=5000)
    kp2 = orb.detect(img2)
    img2_kp = cv2.drawKeypoints(img2, kp2, None, color=(0,255,0), \
            flags=cv2.DrawMatchesFlags_DEFAULT)
    
    #plt.figure()
    #plt.imshow(img2_kp)
    #plt.show()
    
    pp=np.ones((len(kp2),2));
    
    for i,keypoint in enumerate(kp2):
        pp[i,0]=keypoint.pt[0];
        pp[i,1]=keypoint.pt[1];
    points_old=pp;
    points_old = points_old.astype('float32') ;
    points_old =np.expand_dims(points_old , axis=1);
 """
  #  print (keypoint.pt)
        
    
    
    
  #  fig, axes = plt.subplots(1, 3, figsize=(9, 3), sharex=True, sharey=True)
    
        
    #Parameters for lucas kanade optical flow
    lk_params = dict( winSize  = (15, 15),
                      maxLevel = 2,
                      criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
    
    # detect features:
    
    #points_old = cv2.goodFeaturesToTrack(prev_gray, mask = None, **feature_params);
      
    # calculate optical flow
    points_new, status, error_match = cv2.calcOpticalFlowPyrLK(prev_gray, gray, points_old, None, **lk_params)
    #points_new=points_old;
    # filter the points by their status:
    #print(status)
    #print(status.shape)
    #print(points_old.shape)
    points_old = points_old[status == 1];
    points_new = points_new[status == 1];
   # print(points_old)
    flow_vectors = points_new - points_old;
    
    
    
    if(graphics):
        im = (0.5 * prev_bgr.copy().astype(float) + 0.5 * bgr.copy().astype(float)) / 255.0;
        n_points = len(points_old);
        color = (0.0,1.0,0.0);
        
        wr=0;
        wl=0;
        w = np.ones((n_points,1))  
        #width=128;       
     
        for p in range(n_points):
            if points_old[p,0]==0:
                if points_old[p,1]==0:
                   w[p,0]=0;
                   points_new[p,:]=0;
                   
            cv2.arrowedLine(im, tuple(points_old[p, :]), tuple(points_new[p,:]), color);
           
            
            #w[p,0]= (points_new[p,1]-points_old[p,1])**2+(points_new[p,0]-points_old[p,0])**2;
            w[p,0]=(points_new[p,0]-points_old[p,0])**2;
            if points_old[p,0]==0:
                if points_old[p,0]==0:
                   w[p,0]=0;
            
            if w[p,0]<width/1000 :
                w[p,0]=0;
                
            #if w[p,0]<width/1000 or  w[p,0]>(width-width/1000000):
            #    w[p,0]=0;
                
            if points_old[p,0] > width:
               wr=w[p,0]+wr;
            else:
               wl=w[p,0]+wl;
               
               
     #   plt.figure();
     #   plt.imshow(im);
     #   plt.title('Optical flow ');
        
        
        
        #message = f"c = {c}"
        #plt.text(message)
        ""
        #if not os.path.exists('pics'): 
       #     os.makedirs('pics') 

        #plt.savefig('pics/im'+str(x)+'.png')
        
      
    d=wr+wl;
    c=0;
    th=2000; 
    """
    if d>th:
        c=1;
    elif d<-th:
        c=-1;
        
    """
    
    ""
    plt.figure();
    plt.imshow(im);
  
   
    plt.text(30, 100, "d is {}".format(d))
    plt.title(' x=%i ' %x )
    plt.xlabel('d=%i'%c)
    plt.legend
    plt.show()
   # plt.tight_layout()
   # plt.show()
    ""
   # difference
    if d > 2000:
        if d < 30000:
            c=1
            
            #"file_" + str(i) + ".dat"
            #cv2.imwrite('houghlines%i.jpg' %c,im)
            
            if not os.path.exists('picsforest'): 
                os.makedirs('picsforest')
            """    
            plt.title(' x=%i ' %x  )
            plt.xlabel('d=%i'%d)
            plt.figure();
            plt.imshow(im)
            plt.figure();
            plt.title(' x=%i ' %x )
            plt.savefig('picsforest/im'+str(x)+'.jpg')
           """
    #plt.hold(False)
  #  fig, axes = plt.subplots(1, 3, figsize=(9, 3), sharex=True, sharey=True)
  #  ax = axes.ravel()
   # fig, axes = plt.subplots(1, 3, figsize=(9, 3), sharex=True, sharey=True)
    
    return points_old, points_new, flow_vectors,c,d ;
    
       
def estimate_linear_flow_field(points_old, flow_vectors, RANSAC=False, n_iterations=100, error_threshold=10.0):
    
    n_points = points_old.shape[0];
    sample_size = 3; # minimal sample size is 3
    
    if(n_points >= sample_size):
        
        if(not RANSAC):
            
            # *****************************************
            # TODO: investigate this estimation method:
            # *****************************************
            
            # estimate a linear flow field for horizontal and vertical flow separately:
            # make a big matrix A with elements [x,y,1]
            A = np.concatenate((points_old, np.ones([points_old.shape[0], 1])), axis=1);
            
            # Moore-Penrose pseudo-inverse:
            # https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse
            pseudo_inverse_A = np.linalg.pinv(A);
            
            # target = horizontal flow:
            u_vector = flow_vectors[:,0];
            # solve the linear system:
            pu = np.dot(pseudo_inverse_A, u_vector);
            # calculate how good the fit is:
            errs_u = np.abs(np.dot(A, pu) - u_vector);
            
            # target = vertical flow:
            v_vector = flow_vectors[:,1];
            pv = np.dot(pseudo_inverse_A, v_vector);
            errs_v = np.abs(np.dot(A, pv) - v_vector);
            err = (np.mean(errs_u) + np.mean(errs_v)) / 2.0;
            
        else:
            # This is a RANSAC method to better deal with outliers
            # matrices and vectors for the big system:
            A = np.concatenate((points_old, np.ones([points_old.shape[0], 1])), axis=1);
            u_vector = flow_vectors[:,0];
            v_vector = flow_vectors[:,1];
            
            # solve many small systems, calculating the errors:
            errors = np.zeros([n_iterations, 2]);
            pu = np.zeros([n_iterations, 3])
            pv = np.zeros([n_iterations, 3])
            for it in range(n_iterations):
                inds = np.random.choice(range(n_points), size=sample_size, replace=False);
                AA = np.concatenate((points_old[inds,:], np.ones([sample_size, 1])), axis=1);
                pseudo_inverse_AA = np.linalg.pinv(AA);
                # horizontal flow:
                u_vector_small = flow_vectors[inds, 0];
                # pu[it, :] = np.linalg.solve(AA, UU);
                pu[it,:] = np.dot(pseudo_inverse_AA, u_vector_small);
                errs = np.abs(np.dot(A, pu[it,:]) - u_vector);
                errs[errs > error_threshold] = error_threshold;
                errors[it, 0] = np.mean(errs);
                # vertical flow:
                v_vector_small = flow_vectors[inds, 1];
                # pv[it, :] = np.linalg.solve(AA, VV);
                pv[it, :] = np.dot(pseudo_inverse_AA, v_vector_small);
                errs = np.abs(np.dot(A, pv[it,:]) - v_vector);
                errs[errs > error_threshold] = error_threshold;
                errors[it, 1] = np.mean(errs);
            
            # take the minimal error
            errors = np.mean(errors, axis=1);
            ind = np.argmin(errors);
            err = errors[ind];
            pu = pu[ind, :];
            pv = pv[ind, :];
    else:
        # not enough samples to make a linear fit:
        pu = np.asarray([0.0]*3);
        pv = np.asarray([0.0]*3);
        err = error_threshold;
        
    return pu, pv, err;

# these functions are to get a nice directory listing
def get_number_file_name(name,d):
    inds1 = [m.start() for m in re.finditer('_', name)]
    if(inds1 == []):
        return 0;
    ind1 = inds1[-1];
    inds2 = [m.start() for m in re.finditer('\.', name)]
    if(inds2 == []):
        return 0;
    ind2 = inds2[-1];
    number = name[ind1+1:ind2];
    return int(number);

def compare_file_names(name1, name2):
    number1 = get_number_file_name(name1);
    number2 = get_number_file_name(name2);
    return number1 - number2;
    
def show_flow(x,image_nr_1, image_nr_2, image_dir_name = './image_sequence_pure_ver1/', image_prefix='image_', image_type = 'jpg'):
    image_name_1 = image_dir_name + image_prefix + str(image_nr_1) + '.' + image_type;
    prev_bgr = cv2.imread(image_name_1);
    prev_bgr = cv2.rotate(prev_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)
    
    scale_percent=100;
    wid1 = int(prev_bgr.shape[1] * scale_percent / 100)
    height1 = int(prev_bgr.shape[0] * scale_percent / 100)
    dim1 = (wid1, height1)
    prev_bgr = cv2.resize(prev_bgr, dim1, interpolation = cv2.INTER_AREA)
   # plt.figure();
   # plt.imshow(prev_bgr);
   #plt.title('First image, nr' + str(image_nr_1));
    
    image_name_2 = image_dir_name + image_prefix + str(image_nr_2) + '.' + image_type;
    
    bgr = cv2.imread(image_name_2);
    bgr = cv2.rotate(bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)
    
    wid = int(bgr.shape[1] * scale_percent / 100)
    height = int(bgr.shape[0] * scale_percent / 100)
    dim = (wid, height)
    bgr = cv2.resize(bgr, dim, interpolation = cv2.INTER_AREA)

    
    
    
    
    width = (bgr.shape[1])/2;
    
    
   #plt.figure();
   #plt.imshow(bgr);
   # plt.title('Second image, nr' + str(image_nr_2));
    
    # print('name1: {}\nname2: {}'.format(image_name_1, image_name_2));
    points_old, points_new, flow_vectors,c, d= determine_optical_flow(image_name_1,width,x,prev_bgr, bgr, graphics=True);
    return points_old, points_new, flow_vectors, c,d;
    

def extract_flow_information(image_dir_name = './image_sequence_pure_ver7/', image_type = 'jpg', verbose=True, graphics = True, flow_graphics = False):
    
    # get the image names from the directory:
    image_names = [];
    for file in os.listdir(image_dir_name):
        if file.endswith(image_type):
            image_names.append(image_dir_name + file);
    if sys.version_info[0] < 3:
        # Python 2:
        image_names.sort(cmp=compare_file_names);
    else:
        # Python 3:
        image_names.sort(key=get_number_file_name);
    
    # iterate over the images:
    n_images = len(image_names);
    FoE_over_time = np.zeros([n_images, 2]);
    horizontal_motion_over_time = np.zeros([n_images, 1]);
    vertical_motion_over_time = np.zeros([n_images, 1]);
    divergence_over_time = np.zeros([n_images, 1]);
    errors_over_time = np.zeros([n_images, 1]);
    elapsed_times = np.zeros([n_images,1]);
    FoE = np.asarray([0.0]*2);
    for im in np.arange(0, n_images, 1):
        
        bgr = cv2.imread(image_names[im]);
        
        if(im > 0):
            
            t_before = time.time()
            # determine optical flow:
            points_old, points_new, flow_vectors = determine_optical_flow(prev_bgr, bgr, graphics=flow_graphics);
            # do stuff
            elapsed = time.time() - t_before;
            if(verbose):
                print('Elapsed time = {}'.format(elapsed));
            elapsed_times[im] = elapsed;

            # convert the pixels to a frame where the coordinate in the center is (0,0)
            points_old -= 128.0;
            
            # extract the parameters of the flow field:
            pu, pv, err = estimate_linear_flow_field(points_old, flow_vectors);
            
            # ************************************************************************************
            # TODO: assignment: extract the focus of expansion and divergence from the flow field:
            # ************************************************************************************
            # change the following five lines of code!
            horizontal_motion = -pu[2]; # 0.0;
            vertical_motion = -pv[2]; #0.0;
            # theoretically correct, but numerically not so stable:
            FoE[0] = -pu[2]/pu[0]; #0.0;
            FoE[1] = -pv[2]/pv[1]; #0.0;
            divergence = (pu[0]+pv[1]) / 2.0; # 0.0;
            
            # book keeping:
            horizontal_motion_over_time[im] = horizontal_motion;
            vertical_motion_over_time[im] = vertical_motion;
            FoE_over_time[im, 0] = FoE[0];
            FoE_over_time[im, 1] = FoE[1];
            divergence_over_time[im] = divergence;
            errors_over_time[im] = err;
            
            if(verbose):
                # print the FoE and divergence:
                print('error = {}, FoE = {}, {}, and divergence = {}'.format(err, FoE[0], FoE[1], divergence));
        
        # the current image becomes the previous image:
        prev_bgr = bgr;
    
    print('*** average elapsed time = {} ***'.format(np.mean(elapsed_times[1:,0])));
    
    if(graphics):
        
        # ********************************************************************
        # TODO:
        # What is the unit of the divergence?
        # Can you also draw the time-to-contact over time? In what unit is it?
        # ********************************************************************
        
        plt.figure();
        plt.plot(range(n_images), divergence_over_time, label='Divergence');
        plt.xlabel('Image')
        plt.ylabel('Divergence')
        
        plt.figure();
        plt.plot(range(n_images), FoE_over_time[:,0], label='FoE[0]');
        plt.plot(range(n_images), FoE_over_time[:,1], label='FoE[1]');
        plt.plot(range(n_images), np.asarray([0.0]*n_images), label='Center of the image')
        plt.legend();
        plt.xlabel('Image')
        plt.ylabel('FoE')
        
        plt.figure();
        plt.plot(range(n_images), errors_over_time, label='Error');
        plt.xlabel('Image')
        plt.ylabel('Error')
        
        plt.figure();
        plt.plot(range(n_images), horizontal_motion_over_time, label='Horizontal motion');
        plt.plot(range(n_images), vertical_motion_over_time, label='Vertical motion');
        plt.xlabel('Image')
        plt.ylabel('Motion U/Z')        


    
    
    
    
    
    
    
    
    
    
