#include "move.h"

//  Standard Library
#include <math.h>

// EPFL-MICRO 315 library
#include <motors.h>

// this project files
#include "debug.h"
#include "constantes.h"


//#define SPEED_FACTOR 			MOTOR_STEPS_PER_TURN / (2 * M_PI * (WHEEL_DIAMETER / 2))
//#define MOVE_DURATION_FACTOR	1000 * ((M_PI / 180.) * (WHEEL_DISTANCE / 2))
#define SPEED_FACTOR 			7.76365566253662109375f		// exact rounded value in float
#define MOVE_DURATION_FACTOR	462.512251777f


uint16_t move_translate(uint16_t dist, int16_t speed)
{
	return 0;
}

uint16_t move_rotate(uint16_t angle, int16_t speed)
{
	static uint32_t s_move_duration = 0;
	static int s_robot_speed = 0;

	static uint16_t s_angle_previous = 0;
	static int16_t s_speed_previous = 0;

	// recompute with floats only if required
	if(angle != s_angle_previous || speed != s_speed_previous)
	{
		s_angle_previous = angle;
		s_speed_previous = speed;

		s_move_duration = abs((angle * MOVE_DURATION_FACTOR) / speed);
		s_robot_speed = speed * SPEED_FACTOR;
	}

	left_motor_set_speed(s_robot_speed);
	right_motor_set_speed(-s_robot_speed);


	systime_t time_start = chVTGetSystemTime();

	chThdSleepMilliseconds(s_move_duration);
	left_motor_set_speed(0);
	right_motor_set_speed(0);

	return s_move_duration - (chVTGetSystemTime() - time_start);
}
