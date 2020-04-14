#ifndef MOVE_H_
#define MOVE_H_

#include <stdint.h>
#include <stdbool.h>

// EPFL-MICRO 315 library
#include <motors.h>


#ifdef __cplusplus
extern "C" {
#endif

/**
* @brief   Rotate the robot
*
* @param angle				Relative angle of the desired rotation in deg
* @param speed				Rotation speed of right wheel in step/s
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
