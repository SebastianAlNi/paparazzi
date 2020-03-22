/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/colorfilter.c
 */

// Own header
#include "modules/computer_vision/cv_detect_green_floor.h"
#include "modules/computer_vision/cv.h"
#include <stdio.h>
#include <stdbool.h>
#include "modules/computer_vision/lib/vision/image.h"
#include "modules/computer_vision/opencv_detect_green_floor_functions.h"
#include "modules/computer_vision/opencv_mavguys_optical_flow.h"
//#include "modules/computer_vision/opencv_contour.h"
#include "subsystems/abi.h" // abi deals with messaging between modules

#include <time.h>

#ifndef COLORFILTER_FPS
#define COLORFILTER_FPS 10       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(COLORFILTER_FPS)


#ifndef COLORFILTER_SEND_OBSTACLE
#define COLORFILTER_SEND_OBSTACLE FALSE    ///< Default sonar/agl to use in opticflow visual_estimator
#endif
PRINT_CONFIG_VAR(COLORFILTER_SEND_OBSTACLE)

struct video_listener *listener = NULL;

// Filter Settings real
/*uint8_t color_lum_min = 65;
uint8_t color_lum_max = 110;
uint8_t color_cb_min  = 110;
uint8_t color_cb_max  = 130;
uint8_t color_cr_min  = 120;
uint8_t color_cr_max  = 132;*/

// Filter Settings simulation
uint8_t color_lum_min = 0;
uint8_t color_lum_max = 255;
uint8_t color_cb_min  = 0;
uint8_t color_cb_max  = 110;
uint8_t color_cr_min  = 0;
uint8_t color_cr_max  = 130;

float green_threshold = 0.9;
float obst_threshold = 0.1;
float floor_count_frac = 0.05f;       // floor detection threshold as a fraction of total of image
//uint8_t green[520];
uint8_t num_upper_pixels_checked = 10; // Number of pixel rows checked in upper image for green obstacles

// Result
//volatile int color_count = 0;

/*
 * find_object_centroid
 *
 * Finds the centroid of pixels in an image within filter bounds.
 * Also returns the amount of pixels that satisfy these filter bounds.
 *
 * @param img - input image to process formatted as YUV422.
 * @param p_xc - x coordinate of the centroid of color object
 * @param p_yc - y coordinate of the centroid of color object
 * @param lum_min - minimum y value for the filter in YCbCr colorspace
 * @param lum_max - maximum y value for the filter in YCbCr colorspace
 * @param cb_min - minimum cb value for the filter in YCbCr colorspace
 * @param cb_max - maximum cb value for the filter in YCbCr colorspace
 * @param cr_min - minimum cr value for the filter in YCbCr colorspace
 * @param cr_max - maximum cr value for the filter in YCbCr colorspace
 * @param draw - whether or not to draw on image
 * @return number of pixels of image within the filter bounds.
 * Image height and width are switched!
 */
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max)
{
  clock_t start, end;
  double cpu_time_used;
  start = clock();

  uint32_t cnt = 0;
  uint32_t cnt_upper_green = 0;
  uint8_t command = 0;
  uint32_t tot_x = 0;
  uint32_t tot_y = 0;
  uint32_t tot_upper_y = 0;

  // Rescale
  uint8_t scale_factor = 1; // only use powers of 2
  uint16_t height = img->h / scale_factor;
  uint16_t width = img->w / scale_factor;
  width /= 2; //only consider bottom half of image
  /*struct image_t *img_scaled;
  image_create(img_scaled, img->w, img->h, IMAGE_YUV422);
  image_copy(img, img_scaled);*/
  //image_yuv422_downsample(img, img, scale_factor);

  //printf("height: %d\n", height); //520
  //printf("width: %d\n", width); //240

  int green[height];
  for(uint16_t i = 0; i < height; i++){
	  green[i] = 0;
  }

  uint8_t *buffer = img->buf; // find more information about ->buf, ->h and ->w by holding mouse over image_t in function header

  // Go through all the pixels
  for (uint16_t y = 0; y < img->h; y+=scale_factor) {
    for (uint16_t x = 0; x < img->w/2; x +=scale_factor) { // only check bottom half of the image
      // Check if the color is inside the specified values
      uint8_t *yp, *up, *vp;
      if (x % 2 == 0) {
        // Even x
        up = &buffer[y * 2 * img->w + 2 * x];      // U
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
        vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
        //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
      } else {
        // Uneven x
        up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
        //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
        vp = &buffer[y * 2 * img->w + 2 * x];      // V
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
      }
      if ( (*yp >= lum_min) && (*yp <= lum_max) &&
           (*up >= cb_min ) && (*up <= cb_max ) &&
           (*vp >= cr_min ) && (*vp <= cr_max )) { // check if pixel color fulfills filter definition
        cnt ++; // increase counter variable by 1
        tot_x += x;
        tot_y += y;
        green[y/scale_factor] = 1;
        //if (draw){
        //  *yp = 255;  // make pixel brighter in image
        //}
      }
    }

    for (uint16_t x = img->w-num_upper_pixels_checked-1; x < img->w; x +=scale_factor) { // check top 10 pixel rows for green obstacles
      // Check if the color is inside the specified values
      uint8_t *yp, *up, *vp;
      if (x % 2 == 0) {
        // Even x
        up = &buffer[y * 2 * img->w + 2 * x];      // U
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
        vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
        //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
      } else {
        // Uneven x
        up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
        //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
        vp = &buffer[y * 2 * img->w + 2 * x];      // V
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
      }
      if ( (*yp >= lum_min) && (*yp <= lum_max) &&
           (*up >= cb_min ) && (*up <= cb_max ) &&
           (*vp >= cr_min ) && (*vp <= cr_max )) { // check if pixel color fulfills filter definition
    	cnt_upper_green ++; // increase counter variable by 1
        //tot_x += x;
        //tot_upper_y += y;
        //if (draw){
        //  *yp = 255;  // make pixel brighter in image
        //}
      }
    }
  }

  // Filter single and double green columns
  for(uint16_t g = 1; g < height-2; g++){
	  if(green[g-1] == 0 && green[g] == 1 && green[g+2] == 0){
		  green[g] = 0;
		  green[g+1] = 0;
	  }
  }
  // Filter single and double green columns at the image's left and right borders
  if(green[0] == 1 && green[2] == 0){
	  green[0] = 0;
	  green[1] = 0;
  }
  if(green[height-1] == 1 && green[height-3] == 0){
	  green[height-1] = 0;
	  green[height-2] = 0;
  }

  uint32_t floor_count_threshold = floor_count_frac * width/2 * height;
  uint32_t upper_count_threshold = obst_threshold * num_upper_pixels_checked * height;
  uint16_t count_green_columns = 0;
  uint16_t count_obst_columns = 0;
  uint16_t green_column_min_index = 0;
  uint16_t green_column_max_index = 0;

  // Find left border of green floor
  for(uint16_t i = 0; i < height; i++){
	  if(green[i] == 1){
		  green_column_min_index = i;
		  break;
	  }
  }

  // Find right border of green floor
  for(uint16_t j = height; j > 0; j--){
	  if(green[j] == 1){
		  green_column_max_index = j;
		  break;
	  }
  }

  int sum_indices_obst = 0;
  int sum_indices_green = 0;

  // Count pixel columns with and without obstacle. Sum indices for obstacle centroid calculation
  for(uint16_t k = green_column_min_index; k < green_column_max_index+1; k++){
      if(green[k] == 1){
          count_green_columns++;
          sum_indices_green += k;
      }
      else{
    	  count_obst_columns++;
          sum_indices_obst += k;
      }
  }

  int green_length = green_column_max_index - green_column_min_index;
  float ratio_green;
  float ratio_obst;

  // Calculate ratios of green pixel columns (w.r.t. green horizon width) and of obstacle pixel columns (w.r.t. image width)
  if(green_length != 0){
	  ratio_green = (float)count_green_columns / (float)green_length;
	  ratio_obst = (float)count_obst_columns / (float)height;
  }
  else{
	  ratio_green = 0;
	  ratio_obst = 1;
  }

  //int count_obstacle_pixels = green_length - count_green_columns;
  float cog_obst;

  // Calculate obstacle centroid for heading selection
  if(count_obst_columns > 0){
	  cog_obst = (float)sum_indices_obst / (float)count_obst_columns;
  }
  else{
	  cog_obst = height/2;
  }

  /*int cog_green;
  if(count_green_columns != 0){
	  cog_green = sum_indices_green / count_green_columns;
  }
  else{
	  cog_green = 0;
  }*/

  // Check conditions and provide according command for the navigation module
  // Case 0: Green threshold in upper image area is fulfilled -> green obstacle detected -> turn until out of sight
  if(cnt_upper_green > upper_count_threshold){
	  command = 2; // left
	  printf("Case: 0\n");
  }
  // Case 1: Right third of image contains no green -> turn left
  else if(green_column_max_index <= height/3*2){ // also holds if no floor at all is detected (at border) because green_column_max_index is initialized as 0
	  command = 2; // turn left
	  printf("Case: 1\n");
  }
  // Case 2: Left third of image contains no green -> turn right
  else if(green_column_min_index >= height/3){
	  command = 1; // turn right
	  printf("Case: 2\n");
  }
  //else if(cnt < floor_count_threshold){
  //	  command = 3; // out of bounds, little or no floor detected, no direction could be given
  //}
  //else if(ratio_green > green_threshold){
  // Case 3: obstacle threshold not fulfilled -> move forward
  else if(ratio_obst < obst_threshold){
	  command = 0; // no obstacle found
	  printf("Case: 3\n");
  }
  // Case 4: Obstacle centroid is in right half of the image -> turn left
  else if(cog_obst >= height/2){
	  command = 2; // turn left
	  printf("Case: 4\n");
  }
  // Case 5: Obstacle centroid is in left half of the image -> turn right
  else if(cog_obst < height/2){
	  command = 1; // turn right
	  printf("Case: 5\n");
  }
  else{
	  printf("Error: No matching command for current situation.\n");
  }

  int isObject;
  isObject = opencv_optical_flow((char *) img->buf, img->w, img->h); //function in C++
  printf("isObject is: %d\n ", isObject);

  end = clock();
  cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;

  // Console Output
  printf("green_column_min_index: %d\n", green_column_min_index);
  printf("green_column_max_index: %d\n", green_column_max_index);
  printf("count_green_columns: %d\n", count_green_columns);
  printf("green_length: %d\n", green_length);
  printf("ratio_green: %.2f\n", ratio_green);
  printf("ratio_obst: %.2f\n", ratio_obst);
  printf("ratio_upper_green: %.2f\n", (float)cnt_upper_green/(float)(num_upper_pixels_checked * height));
  printf("cog_obst: %.2f\n", cog_obst);
  printf("Command: %d\n", command);
  printf("Time: %f\n", cpu_time_used);

  /*if (cnt > 0) { // if color was detected (cnt>0), calculate and store centroid coordinates
    *p_xc = (int32_t)roundf(tot_x / ((float) cnt) - img->w * 0.5f);
    *p_yc = (int32_t)roundf(img->h * 0.5f - tot_y / ((float) cnt));
  } else {
    *p_xc = 0;
    *p_yc = 0;
  }*/
  return command; // return amount of detected pixels with specified color
}

// Function
static struct image_t *determine_green_func(struct image_t *img)
{
	// Blur image with opencv in C++
	opencv_blur((char *) img->buf, img->w, img->h, 5);

	int32_t x_c, y_c; // coordinates for centroid
	// Filter, find centroid and get command
	uint32_t command = find_object_centroid(img, &x_c, &y_c, color_lum_min, color_lum_max, color_cb_min, color_cb_max, color_cr_min, color_cr_max); // x_c and y_c are transferred as pointers, i.e. their values are changed directly in the called function. The function also counts the amount of the specified color and returns it

	AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION2_ID, x_c, y_c, 0, 0, command, 1);

	/*printf("count: %d\n", count);
	printf("x_c: %d\n", x_c);
	printf("y_c: %d\n", y_c);*/

	//printf("C++ function called.\n");
	// Find green in C++
	/*opencv_find_green((char *) img->buf, img->w, img->h,
            color_lum_min, color_lum_max,
            color_cb_min, color_cb_max,
            color_cr_min, color_cr_max);*/

	//find_contour((char *) img->buf, img->w, img->h);

	// Rescale
	//image_yuv422_downsample(img, img, 2);

	// Filter
	/*color_count = image_yuv422_colorfilt(img, img,
	                                       color_lum_min, color_lum_max,
	                                       color_cb_min, color_cb_max,
	                                       color_cr_min, color_cr_max);*/

  /*if (COLORFILTER_SEND_OBSTACLE) {
    if (color_count > 20)
    {
      AbiSendMsgOBSTACLE_DETECTION(OBS_DETECTION_COLOR_ID, 1.f, 0.f, 0.f); // name AbiSendMsgOBSTACLE_DETECTION may have to be changed
    }
    else
    {
      AbiSendMsgOBSTACLE_DETECTION(OBS_DETECTION_COLOR_ID, 10.f, 0.f, 0.f);
    }
  }*/

  return img; // Colorfilter did not make a new image
}

void colorfilter_init(void)
{
  cv_add_to_device(&COLORFILTER_CAMERA, determine_green_func, COLORFILTER_FPS);
}
