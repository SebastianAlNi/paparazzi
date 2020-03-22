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
 * @file "modules/computer_vision/cv_opencvdemo.h"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */

#ifndef OPENCV_EXAMPLE_H
#define OPENCV_EXAMPLE_H

#ifdef __cplusplus
extern "C" {
#endif

int opencv_blur(char *img, int width, int height, int val);

int opencv_find_green(char *img, int width, int height,
		int color_lum_min, int color_lum_max,
		int color_cb_min, int color_cb_max,
		int color_cr_min, int color_cr_max);

#ifdef __cplusplus
}
#endif

#endif

