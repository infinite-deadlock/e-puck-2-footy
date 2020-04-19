#ifndef CONSTANTES_H_
#define CONSTANTES_H_

#include <math.h>

#define WHEEL_DIAMETER				41			// wheel_diameter, in mm (official 41.2)
#define WHEEL_DISTANCE				53			// in mm
#define MOTOR_STEPS_PER_TURN		1000

#define EPUCK_LINEAR_RES			(WHEEL_DIAMETER*M_PI/MOTOR_STEPS_PER_TURN)//Epuck unit ~=0.1288mm -> 7764 increments in 1m -> int16_t enough
#define MM2EPUCK(x)					((int16_t)((x)/EPUCK_LINEAR_RES))
#define EPUCK2MM(x)					((int16_t)((x)*EPUCK_LINEAR_RES))
#define ANGLE_TO_DIST_RES			5//in mm, wanted resolution for calculation of distance from angle on camera
#define MAX_RADIUS_FIELD			450//in mm
//#define ANGLE_TO_DIST_ANGLE_RES	(int16_t)DEG2EPUCK((asin(BALL_DIAMETER/2/(MAX_RADIUS_FIELD-ANGLE_TO_DIST_RES))-asin(BALL_DIAMETER/2/MAX_RADIUS_FIELD))/M_PI*180);//needed angle for resolution at minimum sensibility
#define ANGLE_TO_DIST_ANGLE_RES		1//in epuck angle unit
#define EPUCK_ANGULAR_RES			(EPUCK_LINEAR_RES*2/WHEEL_DISTANCE*180/M_PI)//~=0.27849°
#define ANGULAR_UNIT				(EPUCK_ANGULAR_RES/10)//Epuck unit, more precise than motors for calculations -> ~12927 increments in 360° -> int16_t
#define DEG2EPUCK(a)				((int16_t)((a)/ANGULAR_UNIT))

#define BALL_DIAMETER				MM2EPUCK(37)			// ball diameter, in epuck units
#define ROTATION_MARGIN				MM2EPUCK(50)			// distance from ball when rotating around, in epuck units
#define ROTATION_RADIUS				(BALL_DIAMETER/2+ROTATION_MARGIN)

#define MINIMALE_DISTANCE_BALL					60			//in mm, from camera to ball center
//#define MAX_HALF_ANGLE_BALL					DEG2EPUCK(asin(BALL_DIAMETER/2/MINIMALE_DISTANCE_BALL)*180/M_PI)
#define MAX_HALF_ANGLE_BALL						645//in epuck units
//#define MIN_HALF_ANGLE_BALL					DEG2EPUCK(asin(BALL_DIAMETER/2/MAX_RADIUS_FIELD)*180/M_PI)
#define MIN_HALF_ANGLE_BALL						84//in epuck units
#define N_PRECALCULATED_ANGLE_TO_DIST_VALUES 	(MAX_HALF_ANGLE_BALL-MIN_HALF_ANGLE_BALL)/ANGLE_TO_DIST_ANGLE_RES+1
//#define EPUCK_SEARCH_ROTATION_ANGLE 			DEG2EPUCK(45)-2*MAX_HALF_ANGLE_BALL//condition to have at least once the ball fully in sight
#define EPUCK_SEARCH_ROTATION_ANGLE 			DEG2EPUCK(9)	//in epuck units

#endif // CONSTANTES_H_
