/**
 * @file modules/computer_vision/cv_detect_green_floor.h
 * @group Automonous Flight of Micro Air Vehicles 2020 Group 2
 */

#ifndef COLORFILTER_CV_PLUGIN_H
#define COLORFILTER_CV_PLUGIN_H

#include <stdint.h>
#include "modules/computer_vision/cv.h"

// Module functions
//extern void colorfilter_init(void);
//---------copied--------- from cv_opencvdemo.h
#ifndef CV_DETECTGREENFLOOR_H
#define CV_DETECTGREENFLOOR_H

extern void colorfilter_init(void);

#endif
//---------------------------
extern uint8_t color_lum_min;
extern uint8_t color_lum_max;

extern uint8_t color_cb_min;
extern uint8_t color_cb_max;

extern uint8_t color_cr_min;
extern uint8_t color_cr_max;

extern float green_threshold;
extern float obst_threshold;
extern float border_green_threshold;
extern float image_fraction_read;
//extern float floor_count_frac;
//extern uint8_t green[520];

//extern volatile int color_count;


#endif /* COLORFILTER_CV_PLUGIN_H */
