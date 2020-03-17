"""
@file hough_lines.py
@brief This program demonstrates line finding with the Hough transform
"""
import sys
import math
import cv2 as cv
import numpy as np
import math

#def main(argv):
    ## [load]
        
 default_file = 'image_sequence_pure_ver1/image_190.jpg';
    kernel = np.ones((5,5), np.uint8) 
    
    
    
    #img_erosion = cv.erode(filename, kernel, iterations=1) 
    #filename=img_erosion;
   # default_file = cv.rotate(default_file, cv.ROTATE_90_COUNTERCLOCKWISE)
    filename = argv[0] if len(argv) > 0 else default_file
   # img_erosion = cv.erode(filename, kernel, iterations=1) 
    # Loads an image
    
    src = cv.imread(cv.samples.findFile(filename), cv.IMREAD_GRAYSCALE)
    src=  cv.rotate(src, cv.ROTATE_90_COUNTERCLOCKWISE)
    prev_bgr=src;
    scale_percent=100;
    wid1 = int(prev_bgr.shape[1] * scale_percent / 100)
    height1 = int(prev_bgr.shape[0] * scale_percent / 100)
    dim1 = (wid1, height1)
    prev_bgr = cv.resize(prev_bgr, dim1, interpolation = cv.INTER_AREA)
    
    
    img_erosion =src;
    img_erosion = cv.erode(prev_bgr, kernel, iterations=11)
    src=img_erosion;
    # Check if image is loaded fine
    if src is None:
        print ('Error opening image!')
        print ('Usage: hough_lines.py [image_name -- default ' + default_file + '] \n')
        return -1
    ## [load]

    ## [edge_detection]
    # Edge detection
    dst = cv.Canny(src, 30, 50, None, 3)
    ## [edge_detection]

    # Copy edges to the images that will display the results in BGR
    cdst = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)
    cdstP = np.copy(cdst)

    ## [hough_lines]
    #  Standard Hough Line Transform
    lines = cv.HoughLines(dst, 1, np.pi / 180, 150, None, 0, 0)
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

            cv.line(cdst, pt1, pt2, (0,0,255), 3, cv.LINE_AA)
    ## [draw_lines]
    r=0;
    ## [hough_lines_p]
    # Probabilistic Line Transform
    linesP = cv.HoughLinesP(dst, 1, np.pi / 180, 50, None, 150, 10)
    ## [hough_lines_p]
    ## [draw_lines_p]
    # Draw the lines
    global pp
    pp = np.zeros((20,2))  
    minx=100;

   # pp =np.expand_dims(pp , axis=1);
    
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv.LINE_AA)
           # l0=int(math.ceil(l[0] / 10.0)) * 10;
           # l2=int(math.ceil(l[2] / 10.0)) * 10
            theta=np.arctan((l[3]- l[1])/(l[2]- l[0]))
             
            l1=int(linesP[i][0][0])
            l2=linesP[i][0][1]
            l3=linesP[i][0][2]
            l4=linesP[i][0][3]
            #print(theta)
            if abs(theta)>1.5:
                l=np.expand_dims(l , axis=0);  
                l=np.array(l)
              #  l=l.flatten()
                pp[r,0]=l1
                pp[r+1,0]=l3
                pp[r,1]=l2
                pp[r+1,1]=l4
                #if pp[r,0]<minx:
                #   minx=pp[r,0] 
                r=r+2;
                if pp(1,0)
                   print(i)
            #print (linesP[i][0])
            
    global kk
    #for i in range (0,len(pp)):
        
    #global pp
    kk= linesP;
  
    print(pp)
    
    
    
    
    #print(kk)
    ## [draw_lines_p]
    ## [imshow]
    # Show results
    #print (l)
    """
    cv.imshow("Source", src)
    cv.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)
    cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
    """
    
    ## [imshow]
    # [exit]
    # Wait and Exit
    #cv.destroyAllWindows()
    cv.waitKey()
    cv.destroyAllWindows()
#    cv.destroyWindow("Source")
 #   cv.destroyWindow("Detected Lines (in red) - Standard Hough Line Transform")
  #  cv.destroyWindow("Detected Lines (in red) - Probabilistic Line Transform")
    return (0)
    ## [exit]
    #cv.destroyAllWindows()
    #cv.destroyAllWindows()

#if __name__ == "__main__":
 #    main(sys.argv[1:])
#    cv.destroyWindow("Source")
#    cv.destroyWindow("Detected Lines (in red) - Standard Hough Line Transform")
#    cv.destroyWindow("Detected Lines (in red) - Probabilistic Line Transform")
#    cv.destroyAllWindows(0)
#    cv.waitKey(0)
 
     """   
     [275. 149.]
     [467. 124.]
     [273. 148.]
     [263. 150.]
     [264. 148.]
     [ 61. 131.]
     [267. 126.]
     [267.  34.]
     
     
       [107., 149.],
       [199., 155.],
       [443., 119.],
       [245., 237.],
       [ 97., 123.],
       [510.,  79.],
       [512., 100.],
       [197.,  97.],
       [197.,  42.],
       [  0.,   0.]]
     
     """
     
     
     