#ifndef SENSORS_H_
#define SENSORS_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// structures
struct IR_triggers {
	bool left_triggered;
	bool right_triggered;
	bool back_triggered;
};

// Sensors functions

/**
* @brief   Start measurement threads
*
*/
void sensors_start(void);
/**
* @brief   Reset static variables to default value for a new search
*
*/
void sensors_set_ball_to_be_search(void);
/**
* @brief   Capture an image and analyze it to find the ball
*
*/
void sensors_capture_and_search(void);
/**
* @brief   Invert the direction of the search
*
*/
void sensors_invert_rotation(void);
/**
* @brief   Indicates if the search is made in clockwise direction
*
* @return			True if clockwise
*
*/
bool sensors_search_clockwise(void);
/**
* @brief   Indicates if the ball was found in last analysis
*
* @param ball_angle						Pointer to return the position of the ball in epuck angle units
* @param ball_seen_half_angle			Pointer to return the angle in epuck units corresponding to half the size of the ball seen by the camera
*
* @return			True if the ball was found
*
*/
bool sensors_is_ball_found(int16_t * ball_angle, int16_t * ball_seen_half_angle);
/**
* @brief   Get the state of the IR captors for use
*
* @return			The values of triggers, in a struct
*
*/
struct IR_triggers sensors_get_IR_triggers(void);
/**
* @brief   Indicates if the trajectory is obstructed or not
*
* @return			True if there is no object in sight
*
*/
bool sensors_can_move(void);
#ifdef __cplusplus
}
#endif


#endif //SENSORS_H_
