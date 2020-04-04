#include "move.h"
#include "constantes.h"

#include <motors.h>

#define MOVE_BUFFER_SIZE 10

struct Movement {
	int16_t 		speed_left_motor;	//in step/s
	int16_t 		speed_right_motor;	//in step/s
	uint32_t 		duration;			//in ms, uint32 because of small speed
};

static uint16_t s_robotAngle = 0;
static uint16_t s_robotDist = 0;

static struct Movement s_moves[MOVE_BUFFER_SIZE];	//buffer to store movements in queue
static uint8_t s_futur_move = 0;					//index of next movement to add in buffer
static uint8_t s_current_move = 0;					//index of next movement to do in buffer

static BSEMAPHORE_DECL(move_semaphore_new_move, TRUE);

static void make_move(int16_t speed_left, int16_t speed_right, uint32_t duration)
{
	//begin movement
	left_motor_set_speed(speed_left);
	right_motor_set_speed(speed_right);
	chThdSleepMilliseconds(duration);		//thread priority must be high enough for good precision
	//stop movement
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

static THD_WORKING_AREA(wa_handle_moves, 256);
static THD_FUNCTION(handle_moves, arg)
{
	// this function is used in a thread
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1)
    {
    	//wait for one or more new moves and execute them
    	chBSemWait(&move_semaphore_new_move);

    	while(s_current_move != s_futur_move)//loop on all available moves
    	{
    		make_move(s_moves[s_current_move].speed_left_motor,
    				s_moves[s_current_move].speed_right_motor,
					s_moves[s_current_move].duration);

    		s_current_move=(s_current_move+1)%MOVE_BUFFER_SIZE;
    	}
    }
}

static void push_move(int16_t speed_left, int16_t speed_right, uint32_t duration, bool wait)
{
	//do not add movement if buffer full
	if(wait)
	{
		make_move(speed_left, speed_right, duration);
	}
	else if(s_futur_move != s_current_move)	//do not add movement if buffer full
	{
		s_moves[s_futur_move].speed_left_motor = speed_left;
		s_moves[s_futur_move].speed_right_motor = speed_right;
		s_moves[s_futur_move].duration = duration;

		s_futur_move = (s_futur_move+1)%MOVE_BUFFER_SIZE;

    	chBSemSignal(&move_semaphore_new_move);
	}
}

void move_init(void)
{
	//high priority, to control movements correctly
	chThdCreateStatic(wa_handle_moves, sizeof(wa_handle_moves), HIGHPRIO, handle_moves, NULL);
}

uint16_t rotate(uint16_t angle, int16_t speed, bool wait)
{
	uint32_t moveDuration = (uint32_t)angle*1000/EPUCK_ROTATION_RES/speed; //in milliseconds
	//opposite direction of left wheel
	push_move(-speed, speed, moveDuration, wait);
	//update the absolute angle;
	return s_robotAngle = speed>=0? (s_robotAngle+angle)%ANGLE_MAX : (s_robotAngle+ANGLE_MAX-angle)%ANGLE_MAX;
}

uint16_t translate(uint16_t dist, int16_t speed, bool wait)
{
	uint32_t moveDuration = (uint32_t)dist*LINEAR_RES*1000/EPUCK_LINEAR_RES/speed; //in milliseconds

	push_move(speed, speed, moveDuration, wait);

	return s_robotDist += dist;
}

uint16_t getAngle(void)
{
	return s_robotAngle;
}
