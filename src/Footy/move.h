#ifndef MOVE_H_
#define MOVE_H_

#include <stdint.h>

// EPFL-MICRO 315 library
#include <motors.h>

#ifdef __cplusplus
extern "C" {
#endif

// enumerations

typedef enum{
	STATIC = 0,
	TRANSLATION,	// accelerated if back triggered
	ROTATION		// accelerated if triggered in same direction of movement. Rotation inversion if triggered in opposite direction
}Move_state;

// Move functions

/**
* @brief   Init module threads
*
*/
void move_init_threads(void);
/**
* @brief   Defines robot movement state, influences the way the robot answers to dynamic controls
*
* @param new_state			New state of the robot
*
*/
void move_change_state(Move_state new_state);
/**
* @brief   Moves the robot straight for a given distance
*
* @param distance			Distance in epuck units
* @param speed				Translation speed of center in step/s
*
*/
void move_straight(int16_t distance, int16_t speed);
/**
* @brief   Rotate the robot around his center
*
* @param angle				Relative angle of the desired rotation in epuck units
* @param speed				Rotation speed of right wheel in step/s
*
*/
void move_rotate(int16_t angle, int16_t speed);
/**
* @brief   Move the robot in an half circle around a point at radius in front of him, then face center
*
* @param radius				Circle radius in epuck units
* @param speed				Translation speed of center in step/s
*
*/
void move_round_about(int16_t radius, int16_t speed);

#ifdef __cplusplus
}
#endif

#endif // MOVE_H_
