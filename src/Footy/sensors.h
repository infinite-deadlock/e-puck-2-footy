#ifndef SENSORS_H_
#define SENSORS_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

struct Analysis_result {
	uint16_t 				leftAngle;//relative angle of the left edge of the ball
	uint16_t 				rightAngle;//relative angle of the right edge of the ball
	bool					found;//true if the ball was found
	bool					leftCropped;//true if the falling edge is at the end of the detection window
	bool					rightCropped;//true if the falling edge is at the end of the detection window
};

void sensors_start(void);
void * sensors_get_semaphore_authorization_move(void);
void sensors_set_ball_to_be_search(void);
bool sensors_can_move(void);
bool sensors_is_ball_found(float * ball_angle, float * ball_seen_angle, bool clockwise, bool * invert_rotation);

#ifdef __cplusplus
}
#endif


#endif //SENSORS_H_
