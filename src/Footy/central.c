#include "central.h"

// ChibiOS & others
#include "ch.h"					// main include files

// this project files
#include "move.h"
#include "sensors.h"

// local defines

// semaphores
static BSEMAPHORE_DECL(central_semaphore_image_request, TRUE);

// global variable to this module
static float central_ball_angle = 0.f;
static bool central_ball_found = false;

void central_control_loop(void)
{
	//PI: pour une raison peu connue, le robot ne fera pas assez d'étapes au premier tour
	// Ce symptôme disparaît quand on essaye de connecter l'e-puck
	// chThdSleepMilliseconds(5000); // wait for user to place e-puck on ground

	while(1)
	{
		chThdSleepMilliseconds(5000);

		central_ball_found = false;
		for(uint8_t i = 0 ; i < 15 ; ++i)
		{
			// speed 50 in 222 ms
			// speed 40 in 277 ms
			move_rotate(24, 50);

			// wait for the end of the turn plus some inertia stability (e-puck is shaky)
			// meanwhile, image process can occur
			//chThdSleepMilliseconds(250);
			chThdSleepMilliseconds(1100);

			chBSemSignal(&central_semaphore_image_request);
			chBSemWait(central_get_semaphore_authorization_move());

			if(central_ball_found)
				break;
		}
		if(central_ball_found)
		{
			move_rotate(central_ball_angle, 50);
			chThdSleepMilliseconds(2000);
		}
	}
	/*while(1)
	{
		chBSemSignal(&central_semaphore_image_request);
		chBSemWait(central_get_semaphore_authorization_move());
		chThdSleepMilliseconds(6000);
	}*/
}

void central_send_ball_found(float ball_angle)
{
	central_ball_found = true;
	central_ball_angle = ball_angle;
}

void * central_get_semaphore_authorization_acquire(void)
{
	return &central_semaphore_image_request;
}
