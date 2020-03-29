/**
 * @file modules/computer_vision/cv_detect_green_floor.c
 * @group Automonous Flight of Micro Air Vehicles 2020 Group 2
 */

// Own header
#include "modules/computer_vision/cv_detect_green_floor.h"
#include "modules/computer_vision/cv.h"
#include <stdio.h>
#include <stdbool.h>
#include "pthread.h"
//#include "modules/computer_vision/lib/vision/image.h"
//#include "modules/computer_vision/opencv_detect_green_floor_functions.h"
#include "subsystems/abi.h" // abi deals with messaging between modules

#include <time.h>

#ifndef COLORFILTER_FPS
#define COLORFILTER_FPS 10       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(COLORFILTER_FPS)

static pthread_mutex_t mutex; // initialize mutex for thread lock -> important when changing variables that are accessed by multiple threads

#ifndef COLORFILTER_SEND_OBSTACLE
#define COLORFILTER_SEND_OBSTACLE FALSE    ///< Default sonar/agl to use in opticflow visual_estimator
#endif
PRINT_CONFIG_VAR(COLORFILTER_SEND_OBSTACLE)

struct video_listener *listener = NULL;

//Function Declaration
uint8_t green_filter_commands(struct image_t *img,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max);

// define global variables
struct command_object {
  uint8_t command;
  bool updated;
};
struct command_object global_command;

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

float obst_threshold = 0.1;
float border_green_threshold = 0.3;
float image_fraction_read = 0.5;
//uint8_t num_upper_pixels_checked = 5; // Number of pixel rows checked in upper image for green obstacles

// coordinates for centroid, global, so that values from previous frame can be accessed
//int32_t y_c = 0; // Only y_c is used below
uint16_t ct_green = 0; // green centroid

/*
 * green_filter_commands
 *
 * Finds the centroid of pixels in an image within filter bounds.
 * Also returns one of the following commands for the navigation module:
 * 		0 -> no obstacle detected
 * 		1 -> obstacle detected or border reached -> turn right
 * 		2 -> obstacle detected or border reached -> turn left
 *
 * @param img - input image to process formatted as YUV422.
 * @param lum_min - minimum y value for the filter in YCbCr colorspace
 * @param lum_max - maximum y value for the filter in YCbCr colorspace
 * @param cb_min - minimum cb value for the filter in YCbCr colorspace
 * @param cb_max - maximum cb value for the filter in YCbCr colorspace
 * @param cr_min - minimum cr value for the filter in YCbCr colorspace
 * @param cr_max - maximum cr value for the filter in YCbCr colorspace
 * @param draw - whether or not to draw on image
 * @return command for the navigation module (0/1/2)
 * Image height and width are switched!
 */
uint8_t green_filter_commands(struct image_t *img,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max)
{
  // Define time variables and start timer
  clock_t start, end;
  double cpu_time_used;
  start = clock();

  uint32_t cnt = 0;
  //uint32_t cnt_upper_green = 0; // Counter for the green pixels in the upper image -> green obstacle detection
  uint8_t command = 0; // Command that will be returned for the navigation module
  //uint32_t tot_x = 0;
  //uint32_t tot_y = 0;
  //uint32_t tot_upper_y = 0;

  // Scale at which the image is read:
  //	1 means every pixel is read
  //	2 means every other pixel is read
  //	4 means every fourth pixel is read and so on...
  uint8_t scale_factor = 2; // only use powers of 2
  uint16_t height = img->h / scale_factor;
  uint16_t width = img->w / scale_factor;
  width /= 2; //only consider bottom half of image

  int green[height]; // Array that stores whether a pixel column contains a green pixel or not
  for(uint16_t i = 0; i < height; i++){
	  green[i] = 0; // Initialize array with zeros
  }

  uint8_t *buffer = img->buf; // copy image buffer

  // Go through the pixels of the image's bottom half
  for (uint16_t y = 0; y < img->h; y+=scale_factor) {
    for (uint16_t x = 0; x < img->w*image_fraction_read; x +=scale_factor) { // only check bottom half of the image
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
        cnt++; // increase counter variable by 1
        //tot_x += x;
        //tot_y += y/scale_factor;
        green[y/scale_factor] = 1; // Store a one for every pixel column that contains a green pixel
        *yp = 255;  // make pixel brighter in image to see it on the video
      }
    }
    // Go through the upper pixels of the image to detect a potential green obstacle
    /*for (uint16_t x = img->w-num_upper_pixels_checked-1; x < img->w; x +=scale_factor) { // check top 10 pixel rows for green obstacles
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
    	cnt_upper_green++; // increase counter variable by 1
        //tot_upper_y += y;
      }
    }*/
  }

  //---------------------------------------
  // Filter background noise
  //---------------------------------------

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

  //---------------------------------------
  // Variable Definitions
  //---------------------------------------

  //uint32_t floor_count_threshold = floor_count_frac * width/2 * height;
  //uint32_t upper_count_threshold = obst_threshold * num_upper_pixels_checked * height; // threshold for green pixels in upper image
  uint16_t count_green_columns = 0; // number of pixel columns that contain a green pixel
  uint16_t count_obst_columns = 0; // number of pixel columns that belong to an obstacle
  uint16_t green_column_min_index = 0; // smallest column index that belongs to the green floor
  uint16_t green_column_max_index = height-1; // largest column index that belongs to the green floor

  //---------------------------------------
  // Determine left and right borders of green area
  //---------------------------------------

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

  //---------------------------------------
  // Centroid Calculation
  //---------------------------------------

  uint32_t sum_indices_obst = 0;
  uint32_t sum_indices_green = 0;

  // Count pixel columns with and without obstacle. Sum indices for obstacle centroid calculation
  for(uint16_t k = green_column_min_index; k < green_column_max_index+1; k++){
      if(green[k] == 1){
          count_green_columns++;
          sum_indices_green += k;
      }
      //else{
	  if(green[k] == 0){
    	  count_obst_columns++;
          sum_indices_obst += k;
      }
  }

  //uint_16 count_obstacle_pixels = green_length - count_green_columns;
  float ct_obst = 0.0f; // obstacle centroid

  // Calculate obstacle centroid for heading selection
  if(count_obst_columns > 0){
	  ct_obst = (float)sum_indices_obst / (float)count_obst_columns;
  }
  else{
	  ct_obst = height/2;
  }
  // Calculate green centroid only when a lot of green is visible
  if(cnt >= 1000){
	  ct_green = sum_indices_green / (float)count_green_columns;
  }

  //---------------------------------------
  // Obstacle Ratio Calculation
  //---------------------------------------

  uint16_t green_length = green_column_max_index - green_column_min_index;
  //float ratio_green;
  float ratio_obst;

  // Calculate ratios of green pixel columns (w.r.t. green horizon width) and of obstacle pixel columns (w.r.t. image width)
  if(green_length != 0){
	  //ratio_green = (float)count_green_columns / (float)green_length;
	  ratio_obst = (float)count_obst_columns / (float)height;
  }
  else{
	  //ratio_green = 0;
	  ratio_obst = 1;
  }

  //---------------------------------------
  // Command Generation
  //---------------------------------------

  // Check conditions and provide according command for the navigation module

  // Case 0: Green threshold in upper image area is fulfilled -> green obstacle detected -> turn until out of sight
  /*if(cnt_upper_green > upper_count_threshold){
	  command = 2; // left
	  printf("Case: 0\n");
  }*/
  // Case 1: Right third of image contains no green -> turn left
  if(green_column_max_index <= height * (1-border_green_threshold) && cnt > 100){ // also holds if no floor at all is detected (at border) because green_column_max_index is initialized as 0
	  command = 2; // turn left
	  printf("Case: 1\n");
  }
  // Case 2: Left third of image contains no green -> turn right
  else if(green_column_min_index >= height * border_green_threshold && cnt > 100){
	  command = 1; // turn right
	  printf("Case: 2\n");
  }
  //Case 3: Out of bounds, check latest available centroid of green area to determine new heading
  else if(cnt < 100){
	  if(ct_green <= height/2){
		  command = 2; // left
		  printf("Case 3: No green floor detected. Last green floor centroid was left.\n");
	  }
	  else{
	  	  command = 1; // right
	  	  printf("Case 3: No green floor detected. Last green floor centroid was right.\n");
	  }
  }
  // Case 4: obstacle threshold not fulfilled -> move forward
  else if(ratio_obst < obst_threshold){
	  command = 0; // no obstacle found
	  printf("Case: 4\n");
  }
  // Case 5: Obstacle centroid is in right half of the image -> turn left
  else if(ct_obst >= height/2){
	  command = 2; // turn left
	  printf("Case: 5\n");
  }
  // Case 6: Obstacle centroid is in left half of the image -> turn right
  else if(ct_obst < height/2){
	  command = 1; // turn right
	  printf("Case: 6\n");
  }
  else{
	  command = 1; // turn right until the drone finds a matching situation
	  printf("Error: No matching command for current situation.\n");
  }

  end = clock();
  cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;

  //---------------------------------------
  // Console Output
  //---------------------------------------

  //printf("green_column_min_index: %d\n", green_column_min_index);
  //printf("green_column_max_index: %d\n", green_column_max_index);
  printf("ratio obstacle: %.2f\n", ratio_obst);
  //printf("ratio_upper_green: %.2f\n", (float)cnt_upper_green/(float)(num_upper_pixels_checked * height));
  printf("ct_obst: %.2f\n", ct_obst);
  printf("ct_green: %d\n", ct_green);
  printf("cnt: %d\n", cnt);
  printf("Command: %d\n", command);
  printf("Time: %f\n", cpu_time_used);

  return command; // return amount of detected pixels with specified color
}

// Function
static struct image_t *determine_green_func(struct image_t *img)
{
	// Filter, find centroid and get command
	uint8_t command = green_filter_commands(img,
			color_lum_min, color_lum_max, color_cb_min, color_cb_max, color_cr_min, color_cr_max); // x_c and y_c are transferred as pointers, i.e. their values are changed directly in the called function. The function also counts the amount of the specified color and returns it

	//AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION2_ID, 0, 0, 0, 0, command, 1);

	pthread_mutex_lock(&mutex); // lock threads while accessing common variables
	global_command.command = command; // store command in command structure
	global_command.updated = true; // store that command data has been changed
	pthread_mutex_unlock(&mutex); // release thread lock

	return img; // Colorfilter did not make a new image
}

void detect_green_floor_init(void)
{
	global_command.command = 0; // Initialize global command structure
	global_command.updated = false;

	cv_add_to_device(&COLORFILTER_CAMERA, determine_green_func, COLORFILTER_FPS); // Video callback
}

void detect_green_floor_periodic(void)
{
  static struct command_object local_command; // local command

  pthread_mutex_lock(&mutex); // lock threads so they can't access same variables at the same time
  local_command = global_command;
  pthread_mutex_unlock(&mutex); // release thread lock

  if(local_command.updated){ // check whether something changed
	  AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION2_ID, 0, 0, 0, 0, local_command.command, 1); // send command to navigation module
	  global_command.updated = false; // store that latest changes are transmitted to navigation module
  }
}
