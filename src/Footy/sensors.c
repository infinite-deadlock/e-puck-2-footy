#include "sensors.h"

//  Standard Library
#include <math.h>

// ChibiOS & others
#include "ch.h"							// main include files
#include "hal.h"						// Hardware Abstraction Layer subsystem header
#include <usbcfg.h>						// USB services
#include <chprintf.h>					// mini printf-like functionality

// EPFL-MICRO 315 library
#include <camera/dcmi_camera.h>			// e-puck-2 Digital CaMera Interface
#include <camera/po8030.h>				// e-puck-2 main frontal camera
#include <sensors/VL53L0X/VL53L0X.h>	// e-puck-2 Time Of Flight sensor
#include <sensors/proximity.h>					// IR sensors

// this project files
#include "debug.h"
#include "move.h"
#include "central.h"
#include "constantes.h"

// local defines
#define IMAGE_BUFFER_SIZE	PO8030_MAX_WIDTH			// size of the image acquired, in RGB565 pixel
#define IMAGE_LINE_HEIGHT	PO8030_MAX_HEIGHT/2			// captured line height

#define NO_RISE_FALL_FOUND_POS          IMAGE_BUFFER_SIZE + 1
#define GREEN_PIXEL_RISE_FALL_THRESHOLD (int16_t)(0.15 * 63)
#define THRESHOLD_BALL_COLOR_IN_GREEN   13
#define THRESHOLD_BALL_COLOR_IN_RED		5
#define TAN_45_OVER_2_CONST             0.4142135679721832275390625f // in rad, fit for float
#define DERIV_DELTA						2

#define	MOVE_SECURITY_SPACE				50

#define IR_SAMPLE_PERIOD				200
#define IR_N_SAMPLE_AVERAGE				5 //low-pass filter
#define IR_TRIGGER_VALUE				20
#define PROX_LEFT						5
#define PROX_RIGHT						2
#define PROX_RIGHT_BACK					3
#define PROX_LEFT_BACK					4

// enumerations
typedef enum {
	SENSOR_LEFT=0,
	SENSOR_RIGHT,
	SENSOR_RIGHT_BACK,
	SENSOR_LEFT_BACK,
	SENSORS_NUMBER,
}IR_sensors;

// semaphores
static BSEMAPHORE_DECL(sensors_semaphore_image_ready_for_process, TRUE);
static BSEMAPHORE_DECL(sensors_semaphore_image_completed, TRUE);

// global variables to this module
static bool sensors_ball_found = false;
static float sensors_ball_angle;
static float sensors_ball_seen_half_angle;
static struct IR_triggers sensors_IR_triggers;

// local function prototypes
void add_value_sum_buffer(uint32_t * sum, uint16_t * buffer, uint8_t next_value_index, uint16_t new_value);
float compute_angle_from_image(uint16_t ball_middle_pos);
void detection_in_image(uint8_t * green_pixels, uint8_t * red_pixels);

// threaded functions
static THD_WORKING_AREA(wa_acquire_image, 256);
static THD_FUNCTION(acquire_image, arg)
{
	// this function is used in a thread
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	// acquire a line from the camera, from pixel 0 in line IMAGE_LINE_HEIGHT of size IMAGE_BUFFER_SIZE x 2 // @PI a revoir
    // caution: two lines asked for internal reasons

	if(po8030_advanced_config(FORMAT_RGB565, 0, IMAGE_LINE_HEIGHT, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1) != MSG_OK)
		chprintf((BaseSequentialStream *)&SD3, "Error po8030_advanced_config\n");
	//if(po8030_set_awb(0) != MSG_OK)
		//chprintf((BaseSequentialStream *)&SD3, "Error po8030_set_awb\n");

	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1)
    {
    	// await the end of an image capture to be signaled
    	chBSemWait(central_get_semaphore_authorization_acquire());
    	// start capturing the image
		dcmi_capture_start();
		// wait until completed
		wait_image_ready();
		// signal completeness with the semaphore
		chBSemSignal(&sensors_semaphore_image_ready_for_process);
    }
}

static THD_WORKING_AREA(wa_process_image, 2048);
static THD_FUNCTION(process_image, arg)
{
	// this function is used in a thread
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t * img_raw_RGB565_pixels = NULL;
	uint8_t   green_pixels[IMAGE_BUFFER_SIZE] = {0};
	uint8_t   red_pixels[IMAGE_BUFFER_SIZE] = {0};

	//static bool s_send_computer = true;
    while(1)
    {
    	// await the end of an image capture to be signaled
        chBSemWait(&sensors_semaphore_image_ready_for_process);

		// get the pointer to the array filled with the last image in RGB565
        img_raw_RGB565_pixels = dcmi_get_last_image_ptr();

        for(uint16_t i = 0 ; i < 2 * IMAGE_BUFFER_SIZE ; i += 2)
        {
        	green_pixels[i/2] = (((img_raw_RGB565_pixels[i] & 7) << 3) | (img_raw_RGB565_pixels[i + 1] >> 5)) & 63;
        	red_pixels[i/2] = (img_raw_RGB565_pixels[i] >> 3);
        }

        detection_in_image(green_pixels, red_pixels);


		//debug_send_uint8_array_to_computer(green_pixels, IMAGE_BUFFER_SIZE);

		//debug_send_for_printlinke_couple_uint8(img_raw_RGB565_pixels, 2 * IMAGE_BUFFER_SIZE);

        chBSemSignal(&sensors_semaphore_image_completed);
    }
}

static THD_WORKING_AREA(wa_watch_IR, 1024);
static THD_FUNCTION(watch_IR, arg)
{
	// this function is used in a thread
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    //low_pass filter (moving average)
	uint16_t samples[SENSORS_NUMBER][IR_N_SAMPLE_AVERAGE] = { 0 };
	uint8_t next_sample_index = 0;
	uint32_t sum[SENSORS_NUMBER] = { 0 };

    proximity_start();
    calibrate_ir();

    while(1)
    {
    	//read values IR sensors, moving sum
    	add_value_sum_buffer(&sum[SENSOR_LEFT], samples[SENSOR_LEFT], next_sample_index, abs(get_calibrated_prox(PROX_LEFT)));
    	add_value_sum_buffer(&sum[SENSOR_LEFT_BACK], samples[SENSOR_LEFT_BACK], next_sample_index, abs(get_calibrated_prox(PROX_LEFT_BACK)));
    	add_value_sum_buffer(&sum[SENSOR_RIGHT], samples[SENSOR_RIGHT], next_sample_index, abs(get_calibrated_prox(PROX_RIGHT)));
    	add_value_sum_buffer(&sum[SENSOR_RIGHT_BACK], samples[SENSOR_RIGHT_BACK], next_sample_index, abs(get_calibrated_prox(PROX_RIGHT_BACK)));
    	++next_sample_index;
    	next_sample_index %= IR_N_SAMPLE_AVERAGE;

    	//check if triggered
    	sensors_IR_triggers.left_triggered = sum[SENSOR_LEFT] >= IR_TRIGGER_VALUE * IR_N_SAMPLE_AVERAGE;
    	sensors_IR_triggers.right_triggered = sum[SENSOR_RIGHT] >= IR_TRIGGER_VALUE * IR_N_SAMPLE_AVERAGE;
    	sensors_IR_triggers.back_triggered = sum[SENSOR_RIGHT_BACK] >= IR_TRIGGER_VALUE * IR_N_SAMPLE_AVERAGE
    												|| sum[SENSOR_LEFT_BACK] >= IR_TRIGGER_VALUE * IR_N_SAMPLE_AVERAGE;

    	chThdSleepMilliseconds(IR_SAMPLE_PERIOD);
    }
}

void add_value_sum_buffer(uint32_t * sum, uint16_t * buffer, uint8_t next_value_index, uint16_t new_value)
{
	*sum -= buffer[next_value_index];
	buffer[next_value_index] = new_value;
	*sum += new_value;
}

float compute_angle_from_image(uint16_t pos)
{
    return atan((1-((float)pos / 320)) * TAN_45_OVER_2_CONST) * 180.f / M_PI;
}

void detection_in_image(uint8_t * green_pixels, uint8_t * red_pixels)
{
    int8_t pixel_derivative;
    uint32_t sum_green = 0;
    uint32_t sum_red = 0;
    uint16_t sum_inc = 0;
    bool last_fall_found = false;
    float last_fall_angle;

    for(uint16_t i = DERIV_DELTA ; i < IMAGE_BUFFER_SIZE - DERIV_DELTA ; ++i)
    {
        pixel_derivative = (int16_t)green_pixels[i + DERIV_DELTA] - (int16_t)green_pixels[i - DERIV_DELTA];
        if(last_fall_found)   // if the beginning of a ball has been seen, we can look at the end of a ball
        {
        	sum_green += green_pixels[i];
        	sum_red += red_pixels[i];
        	sum_inc++;
            if(pixel_derivative >= GREEN_PIXEL_RISE_FALL_THRESHOLD && pixel_derivative)
            {
            	chprintf((BaseSequentialStream *)&SD3, "rise found\n");
                if(sum_green < THRESHOLD_BALL_COLOR_IN_GREEN * (uint32_t)sum_inc && sum_red > THRESHOLD_BALL_COLOR_IN_RED * (uint32_t)sum_inc)
                {
					sensors_ball_found = true;
					sensors_ball_angle = (compute_angle_from_image(i) + last_fall_angle) * 0.5f;
					sensors_ball_seen_half_angle = last_fall_angle-sensors_ball_angle;

                    chprintf((BaseSequentialStream *)&SD3, "ball is located in between: %f and %f\n", last_fall_angle, compute_angle_from_image(i));
                    chprintf((BaseSequentialStream *)&SD3, "angle is %f\n", sensors_ball_angle);
                }
            }
        }
        if(pixel_derivative <= -GREEN_PIXEL_RISE_FALL_THRESHOLD)
        {
        	last_fall_found = true;
        	last_fall_angle = compute_angle_from_image(i);
        	sum_green = 0;
        	sum_red = 0;
            chprintf((BaseSequentialStream *)&SD3, "last fall found\n");
        }
    }
}

void sensors_start(void)
{
	// image
	chThdCreateStatic(wa_process_image, sizeof(wa_process_image), NORMALPRIO, process_image, NULL);
	chThdCreateStatic(wa_acquire_image, sizeof(wa_acquire_image), NORMALPRIO, acquire_image, NULL);
	// IR captors
	chThdCreateStatic(wa_watch_IR, sizeof(wa_watch_IR), NORMALPRIO, watch_IR, NULL);
	// TOF distance
	VL53L0X_start();
}

void sensors_set_ball_to_be_search(void)
{
	sensors_ball_found = false;
}

bool sensors_is_ball_found(float * ball_angle, float * ball_seen_half_angle)
{
	*ball_angle = sensors_ball_angle;
	*ball_seen_half_angle = sensors_ball_seen_half_angle;

	return sensors_ball_found;
}

struct IR_triggers sensors_get_IR_triggers(void)
{
	return sensors_IR_triggers;
}
bool sensors_can_move(void)
{
	/// @PI: Ne marche pas � cause de l'orientation trop vers le haut du VL53L0X
	debug_send_uint32_to_computer(VL53L0X_get_dist_mm());
	return VL53L0X_get_dist_mm() > MOVE_SECURITY_SPACE;	// VL53L0X is now an opaque type outside this module.
}

void * sensors_get_semaphore_authorization_move(void)
{
	return &sensors_semaphore_image_completed;
}
