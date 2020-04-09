#ifndef SENSORS_H_
#define SENSORS_H_

#ifdef __cplusplus
extern "C" {
#endif

void sensors_start(void);
void * central_get_semaphore_authorization_move(void);

#ifdef __cplusplus
}
#endif


#endif //SENSORS_H_
