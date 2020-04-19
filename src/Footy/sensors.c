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
#define DERIV_DELTA						2

#define	MOVE_SECURITY_SPACE				50

#define IR_SAMPLE_PERIOD				200
#define IR_N_SAMPLE_AVERAGE				5 //low-pass filter
#define IR_TRIGGER_VALUE				40
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
static int16_t sensors_ball_angle;
static int16_t sensors_ball_seen_half_angle;
static struct IR_triggers sensors_IR_triggers;
static bool s_sensors_clockwise_search = true;
static bool s_sensors_invert_rotation = false;

// local function prototypes
void add_value_sum_buffer(uint32_t * sum, uint16_t * buffer, uint8_t next_value_index, uint16_t new_value);
int16_t compute_angle_from_image(int16_t pos);
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

        for(uint16_t i = 0, j; i < 2 * IMAGE_BUFFER_SIZE ; i += 2)
        {
        	j = s_sensors_clockwise_search ? i/2 : IMAGE_BUFFER_SIZE-1-i/2;
        	green_pixels[j] = (((img_raw_RGB565_pixels[i] & 7) << 3) | (img_raw_RGB565_pixels[i + 1] >> 5)) & 63;
        	red_pixels[j] = (img_raw_RGB565_pixels[i] >> 3);
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

int16_t compute_angle_from_image(int16_t pos)
{
	//fprintf(fp, "%0.f", DEG2EPUCK(atan((float)(IMAGE_BUFFER_SIZE/2 - pos) / (imageWidth/2) * TAN_45_OVER_2_CONST) * 180.f / M_PI));
	static const int16_t precalculated_values[IMAGE_BUFFER_SIZE/2] =
	{27,	53,		80,		107,	133,	160,	186,	213,	240,	266,
	293,	320,	346,	373,	399,	426,	453,	479,	506,	532,
	559,	586,	612,	639,	666,	692,	719,	745,	772,	799,
	825,	852,	878,	905,	931,	958,	985,	1011,	1038,	1064,
	1091,	1117,	1144,	1170,	1197,	1224,	1250,	1277,	1303,	1330,
	1356,	1383,	1409,	1436,	1462,	1489,	1515,	1542,	1568,	1595,
	1621,	1648,	1674,	1700,	1727,	1753,	1780,	1806,	1833,	1859,
	1886,	1912,	1938,	1965,	1991,	2017,	2044,	2070,	2097,	2123,
	2149,	2176,	2202,	2228,	2255,	2281,	2307,	2333,	2360,	2386,
	2412,	2439,	2465,	2491,	2517,	2544,	2570,	2596,	2622,	2648,
	2675,	2701,	2727,	2753,	2779,	2805,	2831,	2858,	2884,	2910,
	2936,	2962,	2988,	3014,	3040,	3066,	3092,	3118,	3144,	3170,
	3196,	3222,	3248,	3274,	3300,	3326,	3352,	3378,	3404,	3430,
	3456,	3482,	3508,	3533,	3559,	3585,	3611,	3637,	3663,	3688,
	3714,	3740,	3766,	3791,	3817,	3843,	3869,	3894,	3920,	3946,
	3971,	3997,	4022,	4048,	4074,	4099,	4125,	4150,	4176,	4202,
	4227,	4253,	4278,	4304,	4329,	4355,	4380,	4405,	4431,	4456,
	4482,	4507,	4532,	4558,	4583,	4608,	4634,	4659,	4684,	4710,
	4735,	4760,	4785,	4810,	4836,	4861,	4886,	4911,	4936,	4961,
	4987,	5012,	5037,	5062,	5087,	5112,	5137,	5162,	5187,	5212,
	5237,	5262,	5287,	5311,	5336,	5361,	5386,	5411,	5436,	5461,
	5485,	5510,	5535,	5560,	5584,	5609,	5634,	5658,	5683,	5708,
	5732,	5757,	5782,	5806,	5831,	5855,	5880,	5904,	5929,	5953,
	5978,	6002,	6027,	6051,	6075,	6100,	6124,	6148,	6173,	6197,
	6221,	6245,	6270,	6294,	6318,	6342,	6367,	6391,	6415,	6439,
	6463,	6487,	6511,	6535,	6559,	6583,	6607,	6631,	6655,	6679,
	6703,	6727,	6751,	6775,	6798,	6822,	6846,	6870,	6894,	6917,
	6941,	6965,	6988,	7012,	7036,	7059,	7083,	7107,	7130,	7154,
	7177,	7201,	7224,	7248,	7271,	7295,	7318,	7341,	7365,	7388,
	7411,	7435,	7458,	7481,	7505,	7528,	7551,	7574,	7597,	7621,
	7644,	7667,	7690,	7713,	7736,	7759,	7782,	7805,	7828,	7851,
	7874,	7897,	7920,	7943,	7965,	7988,	8011,	8034,	8057,	8079};

	pos = pos < IMAGE_BUFFER_SIZE/2 ? IMAGE_BUFFER_SIZE/2-pos-1 : IMAGE_BUFFER_SIZE/2-pos;

	return pos > 0 ? precalculated_values[abs(pos)] : -precalculated_values[abs(pos)];
}

void detection_in_image(uint8_t * green_pixels, uint8_t * red_pixels)
{
    int8_t pixel_derivative;
    uint32_t sum_green = 0;
    uint32_t sum_red = 0;
    uint16_t sum_inc = 0;
    bool last_fall_found = false;
    int16_t last_fall_angle;

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
					sensors_ball_angle = (compute_angle_from_image(i) + last_fall_angle)/2;
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

    if(sensors_ball_found && s_sensors_clockwise_search)
    	sensors_ball_angle*=-1;

    //ball cut or manual inversion
    if(s_sensors_invert_rotation)
    {
    	s_sensors_invert_rotation = false;
    	s_sensors_clockwise_search = !s_sensors_clockwise_search;
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
	s_sensors_clockwise_search = true;
}

bool sensors_is_ball_found(int16_t * ball_angle, int16_t * ball_seen_half_angle)
{
	*ball_angle = sensors_ball_angle;
	*ball_seen_half_angle = sensors_ball_seen_half_angle;

	return sensors_ball_found;
}

bool sensors_search_clockwise(void)
{
	return s_sensors_clockwise_search;
}

void sensors_invert_rotation(void)
{
	s_sensors_invert_rotation = true;
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
