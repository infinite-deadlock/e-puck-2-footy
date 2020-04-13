#include "move.h"

//  Standard Library
#include <math.h>

// EPFL-MICRO 315 library
#include <motors.h>

// this project files
#include "debug.h"
#include "sensors.h"
#include "constantes.h"


//#define MOVE_DURATION_FACTOR	1000 * ((M_PI / 180.) * (WHEEL_DISTANCE / 2))
#define MOVE_DURATION_FACTOR	462.512251777f

#define OBSTACLE_DETECT_DELAY	150	// in ms

// global variables to this module
static float s_robot_angle = 0.f;


void move_until_obstacle(int16_t speed)
{
	bool is_moving = false;

	speed = speed > MAX_SPEED_MMPS ? MAX_SPEED_MMPS : speed;
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

	speed = speed > MAX_SPEED_MMPS ? MAX_SPEED_MMPS : speed;

	// recompute with floats only if required
	if(angle != s_angle_previous || speed != s_speed_previous)
	{
		s_angle_previous = angle;
		s_speed_previous = speed;

		s_move_duration = abs((angle * MOVE_DURATION_FACTOR) / speed);
		s_robot_speed = speed * SPEED_FACTOR;
	}

	if(angle < 0)
		s_robot_speed *= -1;

	left_motor_set_speed(s_robot_speed);
	right_motor_set_speed(-s_robot_speed);


	s_robot_angle += angle;
	systime_t time_start = chVTGetSystemTime();

	chThdSleepMilliseconds(s_move_duration);
	left_motor_set_speed(0);
	right_motor_set_speed(0);

	return s_move_duration - (chVTGetSystemTime() - time_start);
}

float getAngle(void)
{
	return s_robot_angle;
}
