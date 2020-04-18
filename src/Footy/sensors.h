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

void sensors_start(void);
void * sensors_get_semaphore_authorization_move(void);
void sensors_set_ball_to_be_search(void);
struct IR_triggers sensors_get_IR_triggers(void);
bool sensors_can_move(void);
bool sensors_is_ball_found(int8_t * ball_angle, int8_t * ball_seen_half_angle);
bool sensors_search_clockwise(void);
void sensors_invert_rotation(void);

#ifdef __cplusplus
}
#endif


#endif //SENSORS_H_
