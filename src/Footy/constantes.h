#ifndef CONSTANTES_H_
#define CONSTANTES_H_

#include <math.h>

#define WHEEL_DIAMETER				41			// wheel_diameter, in mm (official 41.2)
#define WHEEL_DISTANCE				53			// in mm
#define MOTOR_STEPS_PER_TURN		1000

#define EPUCK_LINEAR_RES			WHEEL_DIAMETER*M_PI*1000/MOTOR_STEPS_PER_TURN	// um per motor step, approximately 128.8um
#define LINEAR_RES					129			// um, for unit conversion
#define FIELD_RADIUS				4000		// approximately 1m diameter

#define BALL_DIAMETER				38			// ball diameter, in mm (official: 38.50 mm)

#endif // CONSTANTES_H_
