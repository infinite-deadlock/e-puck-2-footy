#include "central.h"

// ChibiOS & others
#include "ch.h"					// main include files
#include <chprintf.h>					// mini printf-like functionality

// this project files
#include "move.h"
#include "sensors.h"
#include "constantes.h"

// local defines
#define SEARCH_SPEED	MOTOR_SPEED_LIMIT / 2

// semaphores
static BSEMAPHORE_DECL(central_semaphore_image_request, TRUE);

static float compute_distance(float ball_seen_half_angle)
{
	return	fabs(BALL_DIAMETER/2/sin(ball_seen_half_angle/180*M_PI));//x=R/sin(alpha/2)
}

void central_control_loop(void)
{
	float ball_angle = 0.f;
	float ball_seen_half_angle = 0.f;
	float ball_distance = 0.f;
	bool ball_found = false;

	//int16_t rotation_speed = MOTOR_SPEED_LIMIT / 2;
	while(1)
	{
		chThdSleepMilliseconds(5000);

		ball_found = false;
		sensors_set_ball_to_be_search();
		while(!ball_found)
		{
			if(sensors_search_clockwise())
				move_rotate(-EPUCK_SEARCH_ROTATION_ANGLE, SEARCH_SPEED);
			else
				move_rotate(EPUCK_SEARCH_ROTATION_ANGLE, SEARCH_SPEED);

			// wait for the end of the turn plus some inertia stability (e-puck is shaky)
			// meanwhile, image process can occur
			//chThdSleepMilliseconds(250);
			chThdSleepMilliseconds(1100);

			chBSemSignal(&central_semaphore_image_request);
			chBSemWait(sensors_get_semaphore_authorization_move());

			ball_found = sensors_is_ball_found(&ball_angle, &ball_seen_half_angle);
			if(ball_found)
				break;
		}

		move_rotate(ball_angle, SEARCH_SPEED);
		chThdSleepMilliseconds(1000);

        chprintf((BaseSequentialStream *)&SD3, "ball distance from robot %f mm\n", compute_distance(ball_seen_half_angle));
		ball_distance = compute_distance(ball_seen_half_angle);

		//fetch the ball
		move_straight(ball_distance-BALL_DIAMETER/2-ROTATION_MARGIN, SEARCH_SPEED);
		move_round_about(BALL_DIAMETER/2+ROTATION_MARGIN, SEARCH_SPEED);
		move_straight(ball_distance+BALL_DIAMETER, SEARCH_SPEED);
	}
}

void * central_get_semaphore_authorization_acquire(void)
{
	return &central_semaphore_image_request;
}
