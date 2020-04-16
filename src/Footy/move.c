#include "move.h"

//  Standard Library
#include <math.h>

// EPFL-MICRO 315 library
#include <motors.h>

// this project files
#include "debug.h"
#include "sensors.h"
#include "constantes.h"


//#define MOVE_DURATION_FACTOR	1000 * ((2*M_PI / 360.) * (WHEEL_DISTANCE / 2))*(MOTOR_STEPS_PER_TURN / (M_PI * WHEEL_DIAMETER)) //=1000* (WHEEL_DISTANCE/WHEEL_DIAMETER) * (MOTOR_STEPS_PER_TURN/360)
#define MOVE_DURATION_FACTOR	3590.78590786f //milli_step/deg -> must be multiplied by angle and divided by speed in step/s

#define OBSTACLE_DETECT_DELAY	150	// in ms


void move_until_obstacle(int16_t default_speed)
{
	bool is_moving = false;
	int16_t actual_speed = 0;
	int16_t new_speed;

	default_speed = default_speed > MOTOR_SPEED_LIMIT ? MOTOR_SPEED_LIMIT : default_speed;
	default_speed = default_speed < 0 ? 0 : default_speed;
	while(1)
	{
		if(sensors_can_move())
		{
			new_speed = sensors_get_linear_speed(default_speed);
			if(new_speed != actual_speed)
			{
				actual_speed = new_speed;
				left_motor_set_speed(actual_speed);
				right_motor_set_speed(actual_speed);

				is_moving = true;
			}
		}
		else
		{
			if(is_moving)
			{
				actual_speed = 0;
				left_motor_set_speed(actual_speed);
				right_motor_set_speed(actual_speed);
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

		s_move_duration = abs((angle * MOVE_DURATION_FACTOR) / speed);
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
