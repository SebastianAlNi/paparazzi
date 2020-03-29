/**
 * @file modules/computer_vision/cv_detect_green_floor.h
 * @group Automonous Flight of Micro Air Vehicles 2020 Group 2
 */

#ifndef COLORFILTER_CV_PLUGIN_H
#define COLORFILTER_CV_PLUGIN_H

#include <stdint.h>

// Module functions
extern void detect_green_floor_init(void);
extern void detect_green_floor_periodic(void);

#endif
//---------------------------
extern uint8_t color_lum_min;
extern uint8_t color_lum_max;

extern uint8_t color_cb_min;
extern uint8_t color_cb_max;

extern uint8_t color_cr_min;
extern uint8_t color_cr_max;

extern float obst_threshold;
extern float border_green_threshold;
extern float image_fraction_read;
