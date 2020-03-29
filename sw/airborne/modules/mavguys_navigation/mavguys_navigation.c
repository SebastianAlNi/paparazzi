/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/mavguys_navigation/mavguys_navigation.c"
 * @author Mavguys 
 * this .c file is based on the original orange_avoider_guided navigation module.
 * This document navigates the drone through the cyberzoo. This is done by information extracted from the vision part. The information is in principle
 * just 0,1,2 which represents SAFE, LEFT and RIGHT. According to this command the drone actuators are controlled. A SAFE command will increase the confidence
 * used during the navigation script and makes it fly straight a head and increase the speed to a limit. If the vision sends a LEFT or RIGHT signal, the navigation
 * module will decrease the confidence and when confidence is 0, the drone will slow down, stop and turn left or right according to the command. 

*/

// STUDENT ADDITIONS: including necessary modules for the nav to run
#include "modules/mavguys_navigation/mavguys_navigation.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <stdio.h>
#include <time.h>

// STUDENT ADDITIONS: defining variables, functions and states
#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[floor_color_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

uint8_t chooseRandomIncrementAvoidance(void);
uint8_t choosedeterminedIncrementAvoidance(void);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
};

// define settings
float oag_max_speed = 1.3f;               // max flight speed [m/s]
float oag_heading_rate = RadOfDeg(60.f);  // heading change setpoint for avoidance [rad/s]
float leftorright = 0.0f;                   //safe, links of rights

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;   // current state in state machine
int32_t floor_centroid = 0;             // floor detector centroid in y direction (along the horizon)
float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead if safe.

const int16_t max_trajectory_confidence = 5;  // number of consecutive negative object detections to be sure we are obstacle free

// STUDENT ADDITIONS:Communication set-up between vision and navigation
#ifndef FLOOR_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define FLOOR_VISUAL_DETECTION_ID to the orange filter
#error Please define FLOOR_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event floor_detection_ev;
static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  leftorright = quality; //STUDENT ADDITIONS: command function (safe, right or left)
  floor_centroid = pixel_y;
}
/*
 * Initialisation function
 */
void mavguys_navigation_init(void) // STUDENT ADDITIONS: initialising the c-file
{
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance(); //STUDENT ADDITIONS: function is used in the beginning

  //STUDENT ADDITIONS: bind our vision program callbacks to receive the vision outputs
  AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
}

/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
void mavguys_navigation_periodic(void)
{
  // Only run the mudule if we are in the correct flight mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = SEARCH_FOR_SAFE_HEADING;
    obstacle_free_confidence = 0;
    return;
  }

  //STUDENT ADDITIONS: update our safe confidence using color threshold by our vision
  if(leftorright != 0.0f){
    obstacle_free_confidence -= 2; // be more cautious with positive obstacle detections
    
  } else {
    obstacle_free_confidence++;
  }

  VERBOSE_PRINT("left or right, %f\n",  leftorright);
  // //STUDENT ADDITIONS: bound obstacle_free_confidence so max 5 and min 0, is a preset number which is still quite okay (does not cause any problems)
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);
  
  //STUDENT ADDITIONS: changed speed, min is 0, max is 1,2
  float speed_sp = fminf(oag_max_speed, 0.5f * obstacle_free_confidence); //was 0.2

  switch (navigation_state){ //STUDENT ADDITIONS: only SAFE, OBSTACLE_FOUND and SEARCH_FOR_SAFE_HEADING, reenter arena etc is covered in the vision part
    case SAFE:
      VERBOSE_PRINT("Safe %d\n", navigation_state);
      VERBOSE_PRINT("SAFE: %d, obstacle_free_confidence: %d\n", navigation_state, obstacle_free_confidence);
       
      if (obstacle_free_confidence == 0){
        navigation_state = OBSTACLE_FOUND;
      } 
      else {
        guidance_h_set_guided_body_vel(speed_sp, 0);
      }

      break;
    case OBSTACLE_FOUND:
      VERBOSE_PRINT("Obstacle found %d\n", navigation_state);
      VERBOSE_PRINT("Obstacle found: %d, obstacle_free_confidence: %d\n", navigation_state, obstacle_free_confidence);
      // stop
      guidance_h_set_guided_body_vel(0, 0);

      // determined select new search direction
      choosedeterminedIncrementAvoidance();

      navigation_state = SEARCH_FOR_SAFE_HEADING;

      break;
    case SEARCH_FOR_SAFE_HEADING:
      VERBOSE_PRINT("Search for safe heading %d\n", navigation_state);
      VERBOSE_PRINT("Search for safe heading: %d, obstacle_free_confidence: %d\n", navigation_state, obstacle_free_confidence);
      guidance_h_set_guided_heading_rate(avoidance_heading_direction * oag_heading_rate);

      // make sure we have a couple of good readings before declaring the way safe
      if (obstacle_free_confidence >= 2){
        guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);
        navigation_state = SAFE;
      }
       break;
   
     default:
      break;
  }
  return;
}

/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */

//STUDENT ADDITIONS: Both function stated below are used, the random one for the initialization and the determined one for the navigation through flight according to vision
uint8_t chooseRandomIncrementAvoidance(void)
{  
  if (rand() % 2 == 0) {
    avoidance_heading_direction = 1.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
  } else {
    avoidance_heading_direction = -1.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
  }
  
  return false;
}

uint8_t choosedeterminedIncrementAvoidance(void){
// Randomly choose CW or CCW avoiding direction
  if (leftorright == 1) {
    avoidance_heading_direction = 1.f;
    VERBOSE_PRINT("Set avoidance increment to left %f\n", avoidance_heading_direction * oag_heading_rate);
   } else if(leftorright == 2) {
    avoidance_heading_direction = -1.f;
    VERBOSE_PRINT("Set avoidance increment to right %f\n", avoidance_heading_direction * oag_heading_rate);
    }
  
  return false;
}
