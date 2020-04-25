#include "sensors.h"

// EPFL-MICRO 315 library
#include <camera/dcmi_camera.h>			// e-puck-2 Digital CaMera Interface
#include <camera/po8030.h>				// e-puck-2 main frontal camera
#include <sensors/VL53L0X/VL53L0X.h>	// e-puck-2 Time Of Flight sensor
#include <sensors/proximity.h>			// IR sensors

// this project files
#include "constantes.h"
#include "debug.h"

// local defines
#define IMAGE_BUFFER_SIZE	PO8030_MAX_WIDTH			// size of the image acquired, in RGB565 pixel
#define IMAGE_LINE_HEIGHT	(PO8030_MAX_HEIGHT / 2)		// captured line height

#define GREEN_PIXEL_RISE_FALL_THRESHOLD (int16_t)(0.15 * 63)
#define THRESHOLD_BALL_COLOR_IN_GREEN   13
#define THRESHOLD_BALL_COLOR_IN_RED		5
#define DERIV_DELTA						2

#define	MOVE_SECURITY_SPACE				50

#define IR_SAMPLE_PERIOD				200
#define IR_N_SAMPLE_AVERAGE				5 	// low-pass filter
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

// global variables to this module
static bool s_sensors_clockwise_search = true;
static bool s_sensors_invert_rotation = false;
static bool sensors_ball_found = false;
static int16_t sensors_ball_angle;
static int16_t sensors_ball_seen_half_angle;

static struct IR_triggers sensors_IR_triggers;

// local function prototypes
static void add_value_sum_buffer(uint32_t * sum, uint16_t * buffer, uint8_t next_value_index, uint16_t new_value);
static int16_t compute_angle_from_image(int16_t pos);
static void analyze_image(void);
static void detection_in_image(uint8_t * green_pixels, uint8_t * red_pixels);

// threaded functions

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
    	sensors_IR_triggers.left_triggered  = sum[SENSOR_LEFT] >= IR_TRIGGER_VALUE * IR_N_SAMPLE_AVERAGE;
    	sensors_IR_triggers.right_triggered = sum[SENSOR_RIGHT] >= IR_TRIGGER_VALUE * IR_N_SAMPLE_AVERAGE;
    	sensors_IR_triggers.back_triggered  = sum[SENSOR_RIGHT_BACK] >= IR_TRIGGER_VALUE * IR_N_SAMPLE_AVERAGE
    												|| sum[SENSOR_LEFT_BACK] >= IR_TRIGGER_VALUE * IR_N_SAMPLE_AVERAGE;

    	chThdSleepMilliseconds(IR_SAMPLE_PERIOD);
    }
}

// functions

void sensors_start(void)
{
	// image processing

    // caution: two lines asked for internal reasons
	if(po8030_advanced_config(FORMAT_RGB565, 0, IMAGE_LINE_HEIGHT, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1) != MSG_OK)
		chprintf((BaseSequentialStream *)&SD3, "Error po8030_advanced_config\n");

	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

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

void sensors_capture_and_search(void)
{
	// start capturing the image
	dcmi_capture_start();
	// wait until completed
	wait_image_ready();

	analyze_image();
}

void sensors_invert_rotation(void)
{
	s_sensors_invert_rotation = true;
}

bool sensors_search_clockwise(void)
{
	return s_sensors_clockwise_search;
}

bool sensors_is_ball_found(int16_t * ball_angle, int16_t * ball_seen_half_angle)
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
	// doesn't detect the ball because tof captor not horizontal
	// debug_send_uint32_to_computer(VL53L0X_get_dist_mm());

	return VL53L0X_get_dist_mm() > MOVE_SECURITY_SPACE;	// VL53L0X is now an opaque type outside this module.
}

// local functions

static void add_value_sum_buffer(uint32_t * sum, uint16_t * buffer, uint8_t next_value_index, uint16_t new_value)
{
	*sum -= buffer[next_value_index];
	buffer[next_value_index] = new_value;
	*sum += new_value;
}

static int16_t compute_angle_from_image(int16_t pos)
{
	static const int16_t precalculated_values[IMAGE_BUFFER_SIZE/2] =
		{2,		5,		7,		10,		13,		15,		18,		21,		23,		26,
		29,		31,		34,		37,		39,		42,		45,		47,		50,		53,
		55,		58,		61,		63,		66,		69,		71,		74,		77,		79,
		82,		85,		87,		90,		93,		95,		98,		101,	103,	106,
		109,	111,	114,	117,	119,	122,	125,	127,	130,	132,
		135,	138,	140,	143,	146,	148,	151,	154,	156,	159,
		162,	164,	167,	170,	172,	175,	177,	180,	183,	185,
		188,	191,	193,	196,	199,	201,	204,	207,	209,	212,
		214,	217,	220,	222,	225,	228,	230,	233,	235,	238,
		241,	243,	246,	249,	251,	254,	256,	259,	262,	264,
		267,	270,	272,	275,	277,	280,	283,	285,	288,	290,
		293,	296,	298,	301,	304,	306,	309,	311,	314,	317,
		319,	322,	324,	327,	330,	332,	335,	337,	340,	342,
		345,	348,	350,	353,	355,	358,	361,	363,	366,	368,
		371,	373,	376,	379,	381,	384,	386,	389,	391,	394,
		397,	399,	402,	404,	407,	409,	412,	415,	417,	420,
		422,	425,	427,	430,	432,	435,	437,	440,	443,	445,
		448,	450,	453,	455,	458,	460,	463,	465,	468,	470,
		473,	476,	478,	481,	483,	486,	488,	491,	493,	496,
		498,	501,	503,	506,	508,	511,	513,	516,	518,	521,
		523,	526,	528,	531,	533,	536,	538,	541,	543,	546,
		548,	551,	553,	555,	558,	560,	563,	565,	568,	570,
		573,	575,	578,	580,	583,	585,	587,	590,	592,	595,
		597,	600,	602,	605,	607,	609,	612,	614,	617,	619,
		622,	624,	626,	629,	631,	634,	636,	639,	641,	643,
		646,	648,	651,	653,	655,	658,	660,	663,	665,	667,
		670,	672,	675,	677,	679,	682,	684,	686,	689,	691,
		694,	696,	698,	701,	703,	705,	708,	710,	713,	715,
		717,	720,	722,	724,	727,	729,	731,	734,	736,	738,
		741,	743,	745,	748,	750,	752,	755,	757,	759,	762,
		764,	766,	768,	771,	773,	775,	778,	780,	782,	785,
		787,	789,	791,	794,	796,	798,	801,	803,	805,	807};

	return pos >= IMAGE_BUFFER_SIZE/2 ? -precalculated_values[pos-IMAGE_BUFFER_SIZE/2] : precalculated_values[IMAGE_BUFFER_SIZE/2-pos-1];
}

static void analyze_image(void)
{
	static uint8_t * img_raw_RGB565_pixels = NULL;
	static uint8_t   green_pixels[IMAGE_BUFFER_SIZE] = {0};
	static uint8_t   red_pixels[IMAGE_BUFFER_SIZE] = {0};

	// get the pointer to the array filled with the last image in RGB565
    img_raw_RGB565_pixels = dcmi_get_last_image_ptr();

    for(uint16_t i = 0, j; i < 2 * IMAGE_BUFFER_SIZE ; i += 2)
    {
    	j = s_sensors_clockwise_search ? i/2 : IMAGE_BUFFER_SIZE-1-i/2;	// revert buffer if rotation is counterclockwise
    	green_pixels[j] = (((img_raw_RGB565_pixels[i] & 7) << 3) | (img_raw_RGB565_pixels[i + 1] >> 5)) & 63;	// must fall when ball is captured
    	red_pixels[j] = (img_raw_RGB565_pixels[i] >> 3);	// ball is red, must not fall as much
    }

    detection_in_image(green_pixels, red_pixels);


	// debug_send_uint8_array_to_computer(green_pixels, IMAGE_BUFFER_SIZE);
	debug_send_for_printlinke_couple_uint8(img_raw_RGB565_pixels, 2 * IMAGE_BUFFER_SIZE);
}

static void detection_in_image(uint8_t * green_pixels, uint8_t * red_pixels)
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

                    chprintf((BaseSequentialStream *)&SD3, "ball is located in between: %d and %d\n", last_fall_angle, compute_angle_from_image(i));
                    chprintf((BaseSequentialStream *)&SD3, "angle is %d\n", sensors_ball_angle);
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

    // invert next detection
    if(s_sensors_invert_rotation)
    {
    	s_sensors_invert_rotation = false;
    	s_sensors_clockwise_search = !s_sensors_clockwise_search;
    }
}

