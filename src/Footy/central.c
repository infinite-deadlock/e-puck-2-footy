#include "central.h"

// ChibiOS & others
#include "ch.h"					// main include files

// this project files
#include "move.h"
#include "sensors.h"

// semaphores
static BSEMAPHORE_DECL(central_semaphore_image_request, TRUE);

void central_control_loop(void)
{
	//PI: pour une raison peu connue, le robot ne fera pas assez d'étapes au premier tour
	// Ce symptôme disparaît quand on essaye de connecter l'e-puck
	chThdSleepMilliseconds(5000); // wait for user to place e-puck on ground

	while(1)
	{
		for(uint8_t i = 0 ; i < 15 ; ++i)
		{
			// speed 50 in 222 ms
			// speed 40 in 277 ms
			move_rotate(24, 50);

			// wait for the end of the turn plus some inertia stability (e-puck is shaky)
			// meanwhile, image process can occur
			chThdSleepMilliseconds(250);

			chBSemSignal(&central_semaphore_image_request);
			chBSemWait(central_get_semaphore_authorization_move());
		}
		chThdSleepMilliseconds(6000);

		//debug_am_i_responding();
	}
}

void * central_get_semaphore_authorization_acquire(void)
{
	return &central_semaphore_image_request;
}
