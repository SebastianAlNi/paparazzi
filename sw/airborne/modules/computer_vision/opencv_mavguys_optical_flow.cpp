/*
 * Copyright (C) C. De Wagter
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/opencv_example.cpp"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */


#include "opencv_mavguys_optical_flow.h"



using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include <math.h> 
using namespace cv;
#include "opencv_image_functions.h"



int opencv_optical_flow(char *img, int width, int height)
{
  //printf("opencv_optical_flow called.\n");
  // Create a new image, using the original bebop image.
  Mat M(width, height, CV_8UC2, img); // original
  Mat src, srcErode, dst;

  // Convert UYVY in paparazzi to grayscale in opencv
  cvtColor(M, src, CV_YUV2GRAY_Y422);

  //Erode
    int iterations = 8;
    int erosion_size = 5;
    Mat element = getStructuringElement(MORPH_RECT, Size(erosion_size,erosion_size));
    Point Anchor(-1,-1)
    erode(src, srcErode, element, anchor, iterations);

  // Canny Edge Detector
  int edgeThresh = 30;
  Canny(srcErode, dst, edgeThresh, edgeThresh * 2);

 // Propabilistic Hough Line Transfer
  vector<Vec4i> linesP;
  HoughLinesP(dst, linesP, 1, CV_PI/180, 150, 50, 10 );

 //Obstacle conditions
   bool isObject = false;
   int  obj    = 0;

// Draw the lines
    for( int i = 0; i < linesP.size(); i++ )
    {
        Vec4i l = linesP[i];
	double x1, y1, x2, y2, X, Y, theta;
        line( dst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
	x1 = l[0]; 
	y1 = l[1];
	x2 = l[2];
	y2 = l[3];
	X = abs(x2-x1);
	Y = abs(y2-y1);
	theta = atan2(X,Y);
 	if (theta < 0.26) {
		isObject = true;
}	
    }

printf("isObject is: %d ", isObject);
  return isObject;
}
