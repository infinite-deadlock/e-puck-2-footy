#ifndef CONSTANTES_H_
#define CONSTANTES_H_

#include <math.h>

#define WHEEL_DIAMETER				41			// wheel_diameter, in mm (official 41.2)
#define WHEEL_DISTANCE				53			// in mm
#define MOTOR_STEPS_PER_TURN		1000

#define EPUCK_SEARCH_NB_ROTATION	15
#define EPUCK_SEARCH_ROTATION_ANGLE 24

#define EPUCK_LINEAR_RES			WHEEL_DIAMETER*M_PI*1000/MOTOR_STEPS_PER_TURN	// um per motor step, approximately 128.8um
#define LINEAR_RES					129			// um, for unit conversion
#define FIELD_RADIUS				4000		// approximately 1m diameter

#define BALL_DIAMETER				37			// ball diameter, in mm

#endif // CONSTANTES_H_
