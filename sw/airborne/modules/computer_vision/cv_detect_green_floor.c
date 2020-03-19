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
//#include "modules/computer_vision/opencv_contour.h"
#include "subsystems/abi.h" // abi deals with messaging between modules

#ifndef COLORFILTER_FPS
#define COLORFILTER_FPS 0       ///< Default FPS (zero means run at camera fps)
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

float green_threshold = 0.8;

//int check = 0;

// Result
//volatile int color_count = 0;

// Function
/*uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max);*/

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
 */
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max)
{
  uint32_t cnt = 0;
  uint32_t command = 0;
  uint32_t tot_x = 0;
  uint32_t tot_y = 0;
  int green[img->h];
  for(int i = 0; i < img->h; i++){
	  green[i] = 0;
  }
  //printf("height: %d\n", img->h); //520
  //printf("width: %d\n", img->w); //240
  uint8_t *buffer = img->buf; // find more information about ->buf, ->h and ->w by holding mouse over image_t in function header

  // Go through all the pixels
  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = 0; x < img->w; x ++) {
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
        green[y] = 1;
        //if (draw){
        //  *yp = 255;  // make pixel brighter in image
        //}
      }
    }
  }

  //float green_threshold = 0.8;
  int count_green_columns = 0;
  int green_column_min_index = 0;
  int green_column_max_index = 0;

  // Find left border of green floor
  for(int i = 0; i < img->h; i++){
	  if(green[i] == 1){
		  green_column_min_index = i;
		  break;
	  }
  }

  // Find right border of green floor
  for(int j = img->h; j > 0; j--){
	  if(green[j] == 1){
		  green_column_max_index = j;
		  break;
	  }
  }

  printf("green_column_min_index: %d\n", green_column_min_index);
  printf("green_column_max_index: %d\n", green_column_max_index);

  int sum_indices_obst = 0;
  int sum_indices_green = 0;

  for(int k = green_column_min_index; k < green_column_max_index+1; k++){
      if(green[k] == 1){
          count_green_columns++;
          sum_indices_green++;
      }
      else{
          sum_indices_obst++;
      }
  }

  int green_length = green_column_max_index - green_column_min_index;
  float ratio_green;
  if(green_length != 0){
	  ratio_green = (float)count_green_columns / (float)green_length;
	  printf("count_green_columns: %d\n", count_green_columns);
	  printf("green_length: %d\n", green_length);
	  printf("ratio_green: %.2f\n", ratio_green);
  }
  else{
	  ratio_green = 0;
  }

  int count_obstacle_pixels = green_length - count_green_columns;
  float cog_obst;
  if(count_obstacle_pixels != 0){
	  cog_obst = (float)sum_indices_obst / (float)count_obstacle_pixels;
  }
  else{
	  cog_obst = 0;
  }

  /*int cog_green;
  if(count_green_columns != 0){
	  cog_green = sum_indices_green / count_green_columns;
  }
  else{
	  cog_green = 0;
  }*/

  if(ratio_green == 0){
	  command = -1; // no floor detected
  }
  else if(green_column_max_index <= img->h/2){
	  command = 1; // turn left
  }
  else if(green_column_min_index > img->h/2){
	  command = 2; // turn right
  }
  else if(ratio_green > green_threshold){
	  command = 0; // no obstacle found
  }
  else if(cog_obst >= img->h/2){
	  command = 1; // turn left
  }
  else if(cog_obst < img->h/2){
	  command = 2; // turn right
  }
  else{
	  printf("Error: No matching command for current situation.\n");
  }

  printf("Command: %d\n", command);

  if (cnt > 0) { // if color was detected (cnt>0), calculate and store centroid coordinates
    *p_xc = (int32_t)roundf(tot_x / ((float) cnt) - img->w * 0.5f);
    *p_yc = (int32_t)roundf(img->h * 0.5f - tot_y / ((float) cnt));
  } else {
    *p_xc = 0;
    *p_yc = 0;
  }
  return command; // return amount of detected pixels with specified color
}

// Function
static struct image_t *determine_green_func(struct image_t *img)
{
	int32_t x_c, y_c; // coordinates for centroid
	// Filter and find centroid
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
