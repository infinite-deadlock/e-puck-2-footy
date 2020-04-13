#ifndef CENTRAL_H_
#define CENTRAL_H_

#ifdef __cplusplus
extern "C" {
#endif

void central_control_loop(void);
void * central_get_semaphore_authorization_acquire(void);
void central_send_ball_found(float ball_angle);

#ifdef __cplusplus
}
#endif

#endif // CENTRAL_H_
