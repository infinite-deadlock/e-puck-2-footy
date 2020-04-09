#ifndef MOVE_H_
#define MOVE_H_

#include <stdint.h>
#include <stdbool.h>

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
uint16_t move_rotate(uint16_t angle, int16_t speed);

/**
* @brief   Rotate the robot
*
* @param dist				Relative distance of the desired translation in cm
* @param speed				Translation speed of wheels in step/s
*
* @return					Absolute distance of the robot
*
*/
uint16_t move_translate(uint16_t dist, int16_t speed);

uint16_t move_get_angle(void);

#ifdef __cplusplus
}
#endif

#endif // MOVE_H_
