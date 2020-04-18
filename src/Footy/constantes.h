#ifndef CONSTANTES_H_
#define CONSTANTES_H_

#include <math.h>

#define WHEEL_DIAMETER				41			// wheel_diameter, in mm (official 41.2)
#define WHEEL_DISTANCE				53			// in mm
#define MOTOR_STEPS_PER_TURN		1000

#define EPUCK_LINEAR_RES			WHEEL_DIAMETER*M_PI/MOTOR_STEPS_PER_TURN//~=0.1288mm -> 7764 increments in 1m -> int16_t enough
#define MM2EPUCK(x)					x/EPUCK_LINEAR_RES
#define EPUCK2MM(x)					x*EPUCK_LINEAR_RES
#define EPUCK_ANGULAR_RES			EPUCK_LINEAR_RES*2/WHEEL_DISTANCE*180/M_PI//~=0.27849° -> ~162 increments in 45° -> int8_t enough for angles
#define DEG2EPUCK(a)				a/EPUCK_ANGULAR_RES
#define EPUCK2DEG(a)				a*EPUCK_ANGULAR_RES

#define BALL_DIAMETER				MM2EPUCK(37)			// ball diameter, in mm
#define ROTATION_MARGIN				MM2EPUCK(20)			// distance from ball when rotating around, in mm

#define MINIMALE_DISTANCE_BALL			60			//in mm, from camera to ball center
//#define EPUCK_SEARCH_ROTATION_ANGLE 	45-2*asin(BALL_DIAMETER/2/MINIMALE_DISTANCE_BALL)//condition to have at least once the ball fully in sight
#define EPUCK_SEARCH_ROTATION_ANGLE 	DEG2EPUCK(9)	//in epuck units

#endif // CONSTANTES_H_
