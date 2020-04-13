#ifndef MOVE_H_
#define MOVE_H_

#include <stdint.h>
#include <stdbool.h>

// EPFL-MICRO 315 library
#include <motors.h>

//#define SPEED_FACTOR 			MOTOR_STEPS_PER_TURN / (M_PI * WHEEL_DIAMETER)
#define SPEED_FACTOR 			7.76365566253662109375f		// exact rounded value in float
//#define MAX_SPEED_MMPS			MOTOR_SPEED_LIMIT/SPEED_FACTOR //~141.75
#define MAX_SPEED_MMPS			141

#ifdef __cplusplus
extern "C" {
#endif

/**
* @brief   Rotate the robot
*
* @param angle				Relative angle of the desired rotation in deg
* @param speed				Rotation speed of right wheel in mm/s
*
* @return					Time difference between planned and executed [ms]
*
*/
uint16_t move_rotate(float angle, int16_t speed);

/**
* @brief   Rotate the robot
*
* @param speed				Translation speed of wheels in step/s
*
*/
void move_until_obstacle(int16_t speed);

/**
* @brief   Get the absolute angle of the robot
*
* @return					Absolute angle of the robot
*
*/
float getAngle(void);

#ifdef __cplusplus
}
#endif

#endif // MOVE_H_
