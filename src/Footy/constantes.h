#ifndef CONSTANTES_H_
#define CONSTANTES_H_

#include <math.h>

/*
 * general purpose constants
 * original alternative expressions are also provided in comments
 */

#define WHEEL_DIAMETER				41			// wheel_diameter, in mm (official 41.2)
#define WHEEL_DISTANCE				53			// in mm
#define MOTOR_STEPS_PER_TURN		1000

#define EPUCK_LINEAR_RES			(WHEEL_DIAMETER*M_PI/MOTOR_STEPS_PER_TURN)	// epuck unit ~=0.1288mm -> 7764 increments in 1m -> int16_t enough
#define MM2EPUCK(x)					((int16_t)((x)/EPUCK_LINEAR_RES))
#define EPUCK2MM(x)					((int16_t)((x)*EPUCK_LINEAR_RES))

#define ANGLE_TO_DIST_RES			5	// in mm, wanted resolution for calculation of distance from angle on camera
#define MAX_RADIUS_FIELD			450	// in mm
//#define ANGLE_TO_DIST_ANGLE_RES	(int16_t)DEG2EPUCK((asin(BALL_DIAMETER/2/(MAX_RADIUS_FIELD-ANGLE_TO_DIST_RES))-asin(BALL_DIAMETER/2/MAX_RADIUS_FIELD))/M_PI*180);//needed angle for resolution at minimum sensitivity
#define ANGLE_TO_DIST_ANGLE_RES		1	// in epuck angle unit
#define EPUCK_ANGULAR_RES			(EPUCK_LINEAR_RES*2/WHEEL_DISTANCE*180/M_PI)	// ~=0.27849°
#define ANGULAR_UNIT				(EPUCK_ANGULAR_RES/10)	// e-puck unit, more precise than motors for calculations -> ~12927 increments in 360° -> int16_t
#define DEG2EPUCK(a)				((int16_t)((a)/ANGULAR_UNIT))

#define BALL_DIAMETER				MM2EPUCK(37)			// ball diameter, in e-puck units
#define ROTATION_MARGIN				MM2EPUCK(60)			// distance from ball when rotating around, in epuck units
#define ROTATION_RADIUS				(BALL_DIAMETER/2+ROTATION_MARGIN)

#define MINIMALE_DISTANCE_BALL					74		// in mm, from camera to ball center
//#define MAX_HALF_ANGLE_BALL					DEG2EPUCK(asin(BALL_DIAMETER/2/(MINIMALE_DISTANCE_BALL/correction_factor))*180/M_PI), correction_factor = (a+-sqrt(a^2-4*b*MINIMALE_DISTANCE_BALL))/2, a and b are correction factors
#define MAX_HALF_ANGLE_BALL						645		// in epuck units
//#define MIN_HALF_ANGLE_BALL					DEG2EPUCK(asin(BALL_DIAMETER/2/(MAX_RADIUS_FIELD/correction_factor))*180/M_PI), correction_factor = 1.583-4e-3*d+4e-8*d^2 = 0.606
#define MIN_HALF_ANGLE_BALL						51		// in epuck units
#define N_PRECALCULATED_ANGLE_TO_DIST_VALUES 	(MAX_HALF_ANGLE_BALL-MIN_HALF_ANGLE_BALL)/ANGLE_TO_DIST_ANGLE_RES+1
//#define EPUCK_SEARCH_ROTATION_ANGLE 			DEG2EPUCK(45)-2*MAX_HALF_ANGLE_BALL // condition to have at least once the ball fully in sight
#define EPUCK_SEARCH_ROTATION_ANGLE 			DEG2EPUCK(9)	// in epuck units
//#define BALL_MIN_SIZE_ACCEPT            		(int16_t)(640/tan(22.5)*tan(asin(BALL_DIAMETER/2/MAX_RADIUS_FIELD*2)))// security value of double the filed size
#define BALL_MIN_SIZE_ACCEPT            		31

#endif // CONSTANTES_H_
