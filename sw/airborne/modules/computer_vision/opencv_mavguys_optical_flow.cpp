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
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"



int opencv_optical_flow(char *img, int width, int height,
		int color_lum_min, int color_lum_max,
		int color_cb_min, int color_cb_max,
		int color_cr_min, int color_cr_max)
{
  // Create a new image, using the original bebop image.
  Mat M(width, height, CV_8UC2, img); // original
  Mat image;

  // Blur it
  blur(M, M, Size(15, 15));

  // Rescale
  resize(M, M, Size(), 0.5, 0.5, INTER_AREA);

  // Convert UYVY in paparazzi to YUV in opencv
  cvtColor(M, M, CV_YUV2RGB_Y422);
  cvtColor(M, M, CV_RGB2YUV);

  // Mask filter
  // Threshold all values within the indicted YUV values.
  inRange(M, Scalar(color_lum_min, color_cb_min, color_cr_min),
		  Scalar(color_lum_max, color_cb_max, color_cr_max), M);

  // Canny Edge Detector
  int edgeThresh = 500;
  Canny(M, M, edgeThresh, edgeThresh * 3);

  // Convert back to YUV422 and put it in place of the original image
  colorbgr_opencv_to_yuv422(M, img, width, height);

  // Create a new image, using the original bebop image.
  //Mat image(height, width, CV_8UC2, img);
  
  // Convert color to YUV (is already)
  //Mat image;
  // cvtColor(M, image, CV_YUV2GRAY_Y422);
  
  // Rescale image
  //Mat Image_scaled
  //resize(image, image, Size(), 0.5, 0.5, INTER_AREA);
  
  // Filter
  //lower_green_YCrCb = np.array([65,120,110])
  //upper_green_YCrCb = np.array([110,132,130])
  //inRange(img_front_YCrCb, lower_green_YCrCb, upper_green_YCrCb)

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
