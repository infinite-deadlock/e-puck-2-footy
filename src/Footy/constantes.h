#ifndef CONSTANTES_H_
#define CONSTANTES_H_

#include <math.h>

#define WHEEL_DIAMETER				41			// wheel_diameter, in mm (official 41.2)
#define WHEEL_DISTANCE				53			// in mm
#define MOTOR_STEPS_PER_TURN		1000

#define BALL_DIAMETER				37			// ball diameter, in mm
#define ROTATION_MARGIN				30			// distance from ball when rotating around, in mm

#define MINIMALE_DISTANCE_BALL			60			//in mm, from camera to ball center
//#define EPUCK_SEARCH_ROTATION_ANGLE 	45-2*asin(BALL_DIAMETER/2/MINIMALE_DISTANCE_BALL)//condition to have at least once the ball fully in sight
#define EPUCK_SEARCH_ROTATION_ANGLE 	90//9	//in degrees

#endif // CONSTANTES_H_
