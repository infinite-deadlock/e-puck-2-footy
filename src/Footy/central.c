#include "central.h"

// ChibiOS & others
#include "ch.h"					// main include files
#include <chprintf.h>					// mini printf-like functionality

// this project files
#include "move.h"
#include "sensors.h"
#include "constantes.h"

// local defines

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

	int16_t rotation_speed = MOTOR_SPEED_LIMIT / 2;
	while(1)
	{
		chThdSleepMilliseconds(5000);

		ball_found = false;
		sensors_set_ball_to_be_search();
		while(!ball_found)
		{
			// speed 50 in 222 ms
			// speed 40 in 277 ms
			move_rotate(-EPUCK_SEARCH_ROTATION_ANGLE, rotation_speed);

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

		move_rotate(ball_angle, rotation_speed);
		chThdSleepMilliseconds(1000);

        chprintf((BaseSequentialStream *)&SD3, "ball distance from robot %f mm\n", compute_distance(ball_seen_half_angle));

		move_until_obstacle(rotation_speed);
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
