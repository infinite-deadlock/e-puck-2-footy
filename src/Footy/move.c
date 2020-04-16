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

	right_motor_set_speed(s_robot_speed);
	left_motor_set_speed(-s_robot_speed);

	systime_t time_start = chVTGetSystemTime();

	chThdSleepMilliseconds(s_move_duration);
	left_motor_set_speed(0);
	right_motor_set_speed(0);

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

	left_motor_set_speed(speed);
	right_motor_set_speed(speed);

	chThdSleepMilliseconds(s_move_duration);
	left_motor_set_speed(0);
	right_motor_set_speed(0);

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

	//half circle
	left_motor_set_speed(speed_fast_wheel);
	right_motor_set_speed(s_speed_slow_wheel);

	chThdSleepMilliseconds(s_move_duration);
	left_motor_set_speed(0);
	right_motor_set_speed(0);

	move_rotate(-90.f, speed_fast_wheel);//face center
}
