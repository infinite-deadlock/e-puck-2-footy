#include "central.h"

// ChibiOS & others
#include "ch.h"					// main include files
#include <chprintf.h>					// mini printf-like functionality

// this project files
#include "move.h"
#include "sensors.h"
#include "constantes.h"

// local defines
#define DEFAULT_SPEED			MOTOR_SPEED_LIMIT/2
#define MANUAL_CONTROL_SPEED	MOTOR_SPEED_LIMIT

// semaphores
static BSEMAPHORE_DECL(central_semaphore_image_request, TRUE);

static float compute_distance(float ball_seen_half_angle)
{
	return	BALL_DIAMETER/2/sin(ball_seen_half_angle);//x=R/sin(alpha/2)
}

void central_control_loop(void)
{
	// Ce symptôme disparaît quand on essaye de connecter l'e-puck
	// chThdSleepMilliseconds(5000); // wait for user to place e-puck on ground

	float ball_angle = 0.f;
	float ball_seen_half_angle = 0.f;
	bool ball_found = false;
	bool rotate_left = false;
	bool rotate_right = false;

	while(1)
	{
		chThdSleepMilliseconds(5000);

		ball_found = false;
		sensors_set_ball_to_be_search();
		while(!ball_found)
		{
			if(sensors_manual_control(&rotate_left, &rotate_right) && rotate_left != rotate_right)
			{
				sensors_set_ball_to_be_search();//past values are not valid anymore
				if(rotate_left)
					move_rotate(MANUAL_CONTROL_ANGLE, MANUAL_CONTROL_SPEED);
				else
					move_rotate(-MANUAL_CONTROL_ANGLE, MANUAL_CONTROL_SPEED);
			}
			else
			{
				// speed 50 in 222 ms
				// speed 40 in 277 ms
				move_rotate(-EPUCK_SEARCH_ROTATION_ANGLE, sensors_get_rotation_speed(DEFAULT_SPEED));

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
		}

		move_rotate(ball_angle, DEFAULT_SPEED);
		chThdSleepMilliseconds(1000);

        chprintf((BaseSequentialStream *)&SD3, "ball distance from robot %f mm\n", compute_distance(ball_seen_half_angle));

		move_until_obstacle(DEFAULT_SPEED);
	}
	/*while(1)
	{
		chBSemSignal(&central_semaphore_image_request);
		chBSemWait(central_get_semaphore_authorization_move());
		chThdSleepMilliseconds(6000);
	}*/
}

void * central_get_semaphore_authorization_acquire(void)
{
	return &central_semaphore_image_request;
}
