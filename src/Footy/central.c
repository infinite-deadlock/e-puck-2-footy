#include "central.h"

// ChibiOS & others
#include "ch.h"					// main include files
#include <chprintf.h>					// mini printf-like functionality

// this project files
#include "move.h"
#include "sensors.h"
#include "constantes.h"

// local defines
#define NSTEPS 15
#define DEFAULT_SPEED_MMPS 50

// semaphores
static BSEMAPHORE_DECL(central_semaphore_image_request, TRUE);

static float compute_distance(float ball_seen_angle)
{
	return	BALL_DIAMETER/2/sin(ball_seen_angle/2);//x=R/sin(alpha/2)
}

void central_control_loop(void)
{
	//PI: pour une raison peu connue, le robot ne fera pas assez d'étapes au premier tour
	// Ce symptôme disparaît quand on essaye de connecter l'e-puck
	// chThdSleepMilliseconds(5000); // wait for user to place e-puck on ground

	float ball_angle = 0.f;
	float ball_seen_angle = 0.f;
	bool ball_found = false;
	while(1)
	{
		chThdSleepSeconds(5);

		ball_found = false;
		for(uint8_t i = 0 ; i < NSTEPS ; ++i)
		{
			chBSemSignal(&central_semaphore_image_request);
			chBSemWait(sensors_get_semaphore_authorization_move());

			ball_found = sensors_is_ball_found(&ball_angle, &ball_seen_angle);
			if(!ball_found)///@Pierre : ça m'a l'air plus propre que le break, de plus on a pas forcément besoin de tourner donc autant commencer par l'analyse
			{
				// speed 50 in 222 ms
				// speed 40 in 277 ms
				move_rotate(360/NSTEPS, DEFAULT_SPEED_MMPS);

				// wait for the end of the turn plus some inertia stability (e-puck is shaky)
				// meanwhile, image process can occur
				//chThdSleepMilliseconds(250);
				chThdSleepMilliseconds(1100);
			}
		}
		if(ball_found)
		{
			move_rotate(ball_angle-getAngle(), DEFAULT_SPEED_MMPS);
			chThdSleepMilliseconds(1000);

			///@Pierre juste pour tester si la distance calculée avec l'angle vue de la balle peut être exploitable
		    chprintf((BaseSequentialStream *)&SD3, "ball is located at %fmm from the robot\n", compute_distance(ball_seen_angle));


			move_until_obstacle(MAX_SPEED_MMPS);///@Pierre 500 c'était trop pour les moteurs, la valeur max vaut environ 141mm/s
		}
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
