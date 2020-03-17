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
argv=sys.argv[1:]
file_name = 'image_sequence_pure_ver1/image_50.jpg';

#default_file = cv.rotate(default_file, cv.ROTATE_90_COUNTERCLOCKWISE)
#filename = argv[0] if len(argv) > 0 else default_file

# Loads an image
src = cv.imread(cv.samples.findFile(filename), cv.IMREAD_GRAYSCALE)
src=  cv.rotate(src, cv.ROTATE_90_COUNTERCLOCKWISE)
# Check if image is loaded fine
if src is None:
    print ('Error opening image!')
    print ('Usage: hough_lines.py [image_name -- default ' + default_file + '] \n')
#return -1
## [load]

## [edge_detection]
# Edge detection
dst = cv.Canny(src, 30, 150, None, 3)
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
linesP = cv.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)
## [hough_lines_p]
## [draw_lines_p]
# Draw the lines

pp = np.zeros((10,2))  


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
        if abs(theta)>3.17/3:
            l=np.expand_dims(l , axis=0);  
            l=np.array(l)
          #  l=l.flatten()
            pp[r,0]=l1
            pp[r+1,0]=l3
            pp[r,1]=l2
            pp[r+1,1]=l4
            r=r+1;
           # print (pp)
        #print (linesP[i][0])

   # global pp
kk= linesP;
ll =np.expand_dims(pp , axis=0);  
#point[0,0]=ll[0];
#point[0,1]=ll[1];
#point[1,2]=ll[1];

ll=np.array(list(l))

