#ifndef CONSTANTES_H_
#define CONSTANTES_H_

#include <math.h>

#define ANGLE_MAX					32760		// counterclockwise =360*91 max resolution for calculation without overflow int16 -> unit equals 1°/91~=39.6''
#define WHEEL_DIAMETER				41			// in mm
#define WHEEL_DISTANCE				53			// in mm
#define MOTOR_STEPS_PER_TURN		1000
#define EPUCK_ROTATION_RES			WHEEL_DIAMETER/(WHEEL_DISTANCE/2)/MOTOR_STEPS_PER_TURN*ANGLE_MAX/2 //angle by step of both motors (opposite direction) ~50.7 (same unit as ANGLE_MAX)
#define INVALID_ANGLE				ANGLE_MAX	// angle is cyclic, so never reached in theory

#define EPUCK_LINEAR_RES			WHEEL_DIAMETER*M_PI*1000/MOTOR_STEPS_PER_TURN	// um per motor step, approximately 128.8um
#define LINEAR_RES					129			// um, for unit conversion
#define FIELD_RADIUS				4000		// approximately 1m diameter

#define BALL_DIAMETER				38			// ball diameter, in mm (official: 38.50 mm)

#endif // CONSTANTES_H_
