#ifndef MOVE_H
#define MOVE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
* @brief   Initializing thread to handle robot's movements
*
*/
void move_init(void);

/**
* @brief   Rotate the robot
*
* @param angle				Relative angle of the desired rotation
* @param speed				Rotation speed of right wheel in step/s
* @param wait				If true wait the end of the movement
*
* @return					Absolute angle of the robot
*
*/
uint16_t rotate(uint16_t angle, int16_t speed, bool wait);

/**
* @brief   Rotate the robot
*
* @param dist				Relative distance of the desired translation
* @param speed				Translation speed of wheels in step/s
* @param wait				If true wait the end of the movement
*
* @return					Absolute distance of the robot
*
*/
uint16_t translate(uint16_t dist, int16_t speed, bool wait);

/**
* @brief   Get the absolute angle of the robot
*
* @return					Absolute angle of the robot
*
*/
uint16_t getAngle(void);

#ifdef __cplusplus
}
#endif

#endif /* MOVE_H */
