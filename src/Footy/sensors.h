#ifndef SENSORS_H_
#define SENSORS_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void sensors_start(void);
void * sensors_get_semaphore_authorization_move(void);
void sensors_set_ball_to_be_search(void);
bool sensors_can_move(void);
bool sensors_is_ball_found(float * ball_angle);

#ifdef __cplusplus
}
#endif


#endif //SENSORS_H_
