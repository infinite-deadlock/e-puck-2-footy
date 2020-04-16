#include "move.h"

//  Standard Library
#include <math.h>

// EPFL-MICRO 315 library
#include <motors.h>

// this project files
#include "debug.h"
#include "sensors.h"
#include "constantes.h"


//#define TRANSLATION_DURATION_FACTOR	1000 * MOTORS_STEPS_PER_TURN/(WHEEL_DIAMETER*M_PI)
#define TRANSLATION_DURATION_FACTOR	7763.65576058f //milli_step/mm -> must be multiplied by distance and divided by speed in step/s
//#define ROTATION_DURATION_FACTOR	1000 * ((2*M_PI / 360.) * (WHEEL_DISTANCE / 2))*(MOTOR_STEPS_PER_TURN / (M_PI * WHEEL_DIAMETER)) //=1000* (WHEEL_DISTANCE/WHEEL_DIAMETER) * (MOTOR_STEPS_PER_TURN/360)
#define ROTATION_DURATION_FACTOR	3590.78590786f //milli_step/deg -> must be multiplied by angle and divided by speed in step/s

#define OBSTACLE_DETECT_DELAY	150	// in ms

//Global variables
systime_t s_move_time_blocked;

// semaphores
static MUTEX_DECL(move_mutex_free_to_move);

// threaded functions
static THD_WORKING_AREA(wa_check_halt, 256);
static THD_FUNCTION(check_halt, arg)
{
	// this function is used in a thread
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    bool stopped = false;

	s_move_time_blocked = chVTGetSystemTime();

    while(1)
    {
    	if(!stopped && !sensors_can_move())//stop moving
    	{
    		chMtxLock(&move_mutex_free_to_move);

    		stopped = true;
    		s_move_time_blocked = chVTGetSystemTime();
    		left_motor_set_speed(0);
    		right_motor_set_speed(0);
    	}
    	if(stopped && sensors_can_move())
    	{
    		chMtxUnlock(&move_mutex_free_to_move);

    		stopped = false;
    	}
		chThdSleepMilliseconds(OBSTACLE_DETECT_DELAY);
    }
}


static void make_move(int16_t speed_left, int16_t speed_right, uint32_t duration)
{
	systime_t time_start;
	int32_t time_moved = 0;
	bool repeat = true;

	do
	{
		chMtxLock(&move_mutex_free_to_move);//if not blocked

	    time_start = chVTGetSystemTime();
		left_motor_set_speed(speed_left);
		right_motor_set_speed(speed_right);

		chMtxUnlock(&move_mutex_free_to_move);

		chThdSleepMilliseconds(duration);
		left_motor_set_speed(0);
		right_motor_set_speed(0);

		chMtxLock(&move_mutex_free_to_move);
		if(s_move_time_blocked <= time_start || duration > (time_moved=s_move_time_blocked-time_start))//move not blocked
			repeat = false;
		else//move was not done
			duration-=time_moved;
		chMtxUnlock(&move_mutex_free_to_move);
	}while(!repeat);
}

void move_init_threads(void)
{
	chThdCreateStatic(wa_check_halt, sizeof(wa_check_halt), NORMALPRIO, check_halt, NULL);
}

void move_until_obstacle(int16_t speed)
{
	bool is_moving = false;

	speed = speed > MOTOR_SPEED_LIMIT ? MOTOR_SPEED_LIMIT : speed;
	speed = speed < 0 ? 0 : speed;
	while(1)
	{
		if(sensors_can_move())
		{
			if(!is_moving)
			{
				left_motor_set_speed(speed);
				right_motor_set_speed(speed);
				is_moving = true;
			}
		}
		else
		{
			if(is_moving)
			{
				left_motor_set_speed(0);
				right_motor_set_speed(0);
				is_moving = false;
			}
		}
		chThdSleepMilliseconds(OBSTACLE_DETECT_DELAY);
	}
}

uint16_t move_rotate(float angle, int16_t speed)
{
	static uint32_t s_move_duration = 0;
	static int s_robot_speed = 0;

	static float s_angle_previous = 0;
	static int16_t s_speed_previous = 0;

	speed = speed > MOTOR_SPEED_LIMIT ? MOTOR_SPEED_LIMIT : speed;
	speed = speed < -MOTOR_SPEED_LIMIT ? -MOTOR_SPEED_LIMIT : speed;

	// recompute with floats only if required
	if(angle != s_angle_previous || speed != s_speed_previous)
	{
		s_angle_previous = angle;
		s_speed_previous = speed;

		s_move_duration = abs((angle * ROTATION_DURATION_FACTOR) / speed);
		s_robot_speed = speed;

		if(angle < 0)
			s_robot_speed *= -1;
	}

	systime_t time_start = chVTGetSystemTime();
	make_move(-s_robot_speed, s_robot_speed, s_move_duration);

	return s_move_duration - (chVTGetSystemTime() - time_start);
}

void move_straight(float distance, int16_t speed)
{
	static float s_dist_previous = 0;
	static int16_t s_speed_previous = 0;
	static uint32_t s_move_duration = 0;

	speed = speed > MOTOR_SPEED_LIMIT ? MOTOR_SPEED_LIMIT : speed;
	speed = speed < -MOTOR_SPEED_LIMIT ? -MOTOR_SPEED_LIMIT : speed;

	if(distance != s_dist_previous || speed != s_speed_previous)
	{
		s_dist_previous = distance;
		s_speed_previous = speed;

		s_move_duration = abs((distance * TRANSLATION_DURATION_FACTOR) / speed);
	}

	make_move(speed, speed, s_move_duration);
}
void move_round_about(float radius, int16_t speed_fast_wheel)
{
	static float s_radius_previous = 0;
	static int16_t s_speed_fast_wheel_previous = 0;
	static int16_t s_speed_slow_wheel = 0;
	static uint32_t s_move_duration = 0;

	speed_fast_wheel = speed_fast_wheel > MOTOR_SPEED_LIMIT ? MOTOR_SPEED_LIMIT : speed_fast_wheel;
	speed_fast_wheel = speed_fast_wheel < -MOTOR_SPEED_LIMIT ? -MOTOR_SPEED_LIMIT : speed_fast_wheel;

	if(radius != s_radius_previous || speed_fast_wheel != s_speed_fast_wheel_previous)
	{
		s_radius_previous = radius;
		s_speed_fast_wheel_previous = speed_fast_wheel;

		s_speed_slow_wheel = speed_fast_wheel/(1+WHEEL_DISTANCE/radius);

		s_move_duration = 180.f*ROTATION_DURATION_FACTOR*2/(speed_fast_wheel - s_speed_slow_wheel);//Half circle -> robot must rotate of 180° around his center
	}

	move_rotate(90.f, speed_fast_wheel);//rotate to be tangeant
	make_move(speed_fast_wheel, s_speed_slow_wheel, s_move_duration);//half circle
	move_rotate(-90.f, speed_fast_wheel);//face center
}
