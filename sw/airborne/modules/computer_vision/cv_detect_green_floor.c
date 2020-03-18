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
#include "modules/computer_vision/lib/vision/image.h"
#include "modules/computer_vision/opencv_detect_green_floor_functions.h"
#include "modules/computer_vision/opencv_contour.h"

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

//int check = 0;

// Result
//volatile int color_count = 0;

#include "subsystems/abi.h"

// Function
static struct image_t *determine_green_func(struct image_t *img)
{
	//printf("C++ function called.\n");
	// Find green in C++
	opencv_find_green((char *) img->buf, img->w, img->h,
            color_lum_min, color_lum_max,
            color_cb_min, color_cb_max,
            color_cr_min, color_cr_max);

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
