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


#include "opencv_detect_green_floor_functions.h"



using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"
#include <cstdio>
#include <iostream>

int check = 0;

int opencv_blur(char *img, int width, int height, int val){
	// Create a new image, using the original bebop image.
	Mat M(width, height, CV_8UC2, img); // original

	// Convert UYVY in paparazzi to YUV in opencv
	cvtColor(M, M, CV_YUV2RGB_Y422);
	//cvtColor(M, M, CV_RGB2YUV);
	//cvtColor(M, M, CV_YUV2BGR_Y422);
	
	// Blur the image
	blur(M, M, Size(val, val));
	//GaussianBlur(M, M, Size(val, val), 0);
	
	// Convert back to YUV422 and put it in place of the original image
	colorbgr_opencv_to_yuv422(M, img, width, height);
	
	return 0;
}

int opencv_find_green(char *img, int width, int height,
		int color_lum_min, int color_lum_max,
		int color_cb_min, int color_cb_max,
		int color_cr_min, int color_cr_max)
{
  //printf("C++ function called here.\n");
  // Create a new image, using the original bebop image.
  Mat M(width, height, CV_8UC2, img); // original
  Mat thresh_image, edge_image, orig_image, dst;
  
  //orig_image = M;

  // Blur it, blurs it really hard, maybe only necessary for real flight, not for simulation
  //blur(M, M, Size(3, 3));

  // Rescale
  //resize(M, M, Size(), 0.1, 0.1, INTER_AREA);

  // Convert UYVY in paparazzi to YUV in opencv
  cvtColor(M, M, CV_YUV2RGB_Y422);
  cvtColor(M, M, CV_RGB2YUV);
  
  //Mat res_image = M;

  // Mask filter
  // Threshold all values within the indicted YUV values.
  inRange(M, Scalar(color_lum_min, color_cb_min, color_cr_min),
		  Scalar(color_lum_max, color_cb_max, color_cr_max), thresh_image);
  
  //Mat B(width, height, CV_8UC1, thresh_image);
  //grayscale_opencv_to_yuv422(thresh_image, thresh_image, width, height);
  
  Scalar intensity = M.at<uchar>(0, 0);
  printf("%f ", intensity.val[0]); // was %d
  intensity = M.at<uchar>(1, 0);
  printf("%f ", intensity.val[0]);
  intensity = M.at<uchar>(2, 0);
  printf("%f ", intensity.val[0]);
  intensity = M.at<uchar>(3, 0);
  printf("%f ", intensity.val[0]);
  intensity = M.at<uchar>(4, 0);
  printf("%f ", intensity.val[0]);
  intensity = M.at<uchar>(5, 0);
  printf("%f ", intensity.val[0]);
  printf("\n");
  
  /*if(check == 0){
	  for(int y = 0; y < height; y++){
		  printf("%d ", intensity.val[0]);
		  intensity = thresh_image.at<uchar>(y, 0);
	  }
  }
  check++;*/
  //Scalar intensity = thresh_image.at<uchar>(200, 200);
  //printf("intensity.val[0]: %d\n", intensity.val[0]);
  //printf(intensity.val[0]);

  // Canny Edge Detector
  //int edgeThresh = 50;
  //Canny(thresh_image, edge_image, edgeThresh, edgeThresh * 3);
  edge_image = thresh_image;
  
  /*if(check == 0){
	  printf("Size of thresh_image: %d\n", width);
	  check++;
  };*/
  
  //sizeof(thresh_image) = 96
  //height = 520
  //width = 240
  
  
  
  // Add edges to original image
  /*dst = Scalar::all(0);
  Mat addweight;
  M.copyTo( dst, edge_image); // copy part of src image according the canny output, canny is used as mask
  cvtColor(edge_image, edge_image, CV_GRAY2BGR); // convert canny image to bgr
  addWeighted( M, 0.5, edge_image, 0.5, 0.0, addweight); // blend src image with canny image
  M += edge_image; // add src image with canny image*/
  
  //coloryuv_opencv_to_yuv422(M, img, width, height);
  //grayscale_opencv_to_yuv422(thresh_image, img, width, height);

  
  float green_threshold = 0.8;
  int count_green_columns = 0;
  int green_column_min_index = 0;
  int green_column_max_index = 0;
  
  bool green[height] = {0};
  int sum_green_true = 0;
  int sum_green_false = 0;
  
  //printf("M green: %d\n", green);
  //printf("thresh_image: %d\n", thresh_image.at<char>(0,100));
  //printf("thresh_image: ");
  //printf(thresh_image.at<char*>(0,100));
  
  // If one pixel per image column is green, store a 1 in the green array
  for(int h = 0; h < height; h++){
	  for(int w = 0; w < width; w++){
		  if(thresh_image.at<int>(w,h) == '0'){
			  green[h] = 1;
			  break;
		  }
	  }
  }
  
  //printf("thresh_image.rows: %d ", thresh_image.rows);
  //printf("thresh_image.cols: %d ", thresh_image.cols);
  
  /*for(int col = 0; col < thresh_image.cols; ++col) {
      unsigned char* p = thresh_image.ptr(col); //pointer p points to the first place of each row
      for(int row = 0; row < thresh_image.rows; ++row) {
    	  //printf("%d ", *p);
    	  if(*p > 0){
    		  green[col] = 1;
    		  break;
    	  }
     	  *p++; // what about maximum *p?
      }
  }*/
  
  if(check == 0){
	  for(int i = 0; i < height; i++){
		  printf("%d ", green[i]);
	  }
	  check++;
  }
  
  // Find left border of green floor
  for(int i = 0; i < height; i++){
	  if(green[i] == 1){
		  green_column_min_index = i;
		  break;
	  }
  }
  
  // Find right border of green floor
  for(int j = height; j > 0; j--){
	  if(green[j] == 1){
		  green_column_max_index = j;
		  break;
	  }
  }
  
  printf("M green_column_min_index: %d\n", green_column_min_index);
  printf("M green_column_max_index: %d\n", green_column_max_index);
  
  /*for(int i = 0; i < height; i++){
	  printf("%d ", green[i]);
  }*/
  
  // Convert canny image to yuv
  //cvtColor(edge_image, edge_image, CV_GRAY2RGB);
  
  // Add original image and canny edges
  //addWeighted(M,1.0,thresh_image,1.0,0,M);
  
  // Put mask on original image
  //bitwise_and(M, M, thresh_image);

  // Convert back to YUV422 and put it in place of the original image
  //cvtColor(M, M, CV_GRAY2RGB);
  //colorbgr_opencv_to_yuv422(M, img, width, height);
  //grayscale_opencv_to_yuv422(edge_image, img, width, height);
  
  

  // Create a new image, using the original bebop image.
  //Mat image(height, width, CV_8UC2, img);

/*#if OPENCVDEMO_GRAYSCALE
  //  Grayscale image example
  cvtColor(M, image, CV_YUV2GRAY_Y422);
  // Canny edges, only works with grayscale image
  int edgeThresh = 35;
  Canny(image, image, edgeThresh, edgeThresh * 3);
  // Convert back to YUV422, and put it in place of the original image
  grayscale_opencv_to_yuv422(image, img, width, height);
#else // OPENCVDEMO_GRAYSCALE
  // Color image example
  // Convert the image to an OpenCV Mat
  cvtColor(M, image, CV_YUV2BGR_Y422);
  // Blur it, because we can
  blur(image, image, Size(5, 5));
  // Convert back to YUV422 and put it in place of the original image
  colorbgr_opencv_to_yuv422(image, img, width, height);
#endif // OPENCVDEMO_GRAYSCALE*/

  return 0;
}
