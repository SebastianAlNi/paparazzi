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
#include "modules/computer_vision/cv_mavguys_optical_flow.h"
#include "modules/computer_vision/cv.h"
#include <stdio.h>
#include "modules/computer_vision/lib/vision/image.h"
#include "modules/computer_vision/opencv_mavguys_optical_flow.h"

#ifndef COLORFILTER_FPS  // If statement to see if defined or not, if not, it runs the lines between the if statement. 
#define COLORFILTER_FPS 10       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(COLORFILTER_FPS) // saves this value I think?


#ifndef COLORFILTER_SEND_OBSTACLE
#define COLORFILTER_SEND_OBSTACLE FALSE    ///< Default sonar/agl to use in opticflow visual_estimator
#endif
PRINT_CONFIG_VAR(COLORFILTER_SEND_OBSTACLE)

//struct video_listener *listener = NULL;

// Filter Settings
/*uint8_t color_lum_min = 65;
uint8_t color_lum_max = 110;
uint8_t color_cb_min  = 110;
uint8_t color_cb_max  = 130;
uint8_t color_cr_min  = 120;
uint8_t color_cr_max  = 132;*/

// Result
//volatile int color_count = 0;

#include "subsystems/abi.h"

// Function, 
static struct image_t *determine_optical_flow(struct image_t *img) //struture, image_t is data type pointer (*) to the funtion determine optical flow, input: img.
{
	// Find object in C++
	int isObject;
	isObject = opencv_optical_flow((char *) img->buf, img->w, img->h); //function in C++
	printf("HAKIM MAGIC OBSTACLE DETECTOR IS RUNNING");
        printf("isObject is: %d\n ", isObject);


  return isObject; // Colorfilter did not make a new image NEW: we want to return 0, or 1 to show that there is an obstacle or not
}

void mavguys_optical_flow_init(void)
{
  cv_add_to_device(&COLORFILTER_CAMERA, determine_optical_flow, COLORFILTER_FPS);
}
