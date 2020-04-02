#include "sensors.h"

// ChibiOS & others
#include "ch.h"						// main include files
#include "hal.h"					// Hardware Abstraction Layer subsystem header
#include <usbcfg.h>					// USB services
#include <chprintf.h>				// mini printf-like functionality

// EPFL-MICRO 315 library
#include <camera/dcmi_camera.h>		// e-puck-2 Digital CaMera Interface
#include <camera/po8030.h>			// e-puck-2 main frontal camera

// this project files
#include "debug.h"

// local defines
#define IMAGE_BUFFER_SIZE	640		// size of the image acquired, in RGB565 pixel
#define BALL_DIAMETER		38		// ball diameter, in mm (official: 38.50 mm)

// semaphores
static BSEMAPHORE_DECL(sensors_semaphore_image_ready, TRUE);

// threaded functions
static THD_WORKING_AREA(wa_acquire_image, 256);
static THD_FUNCTION(acquire_image, arg)
{
	// this function is used in a thread
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	/// @PI: on prend les lignes 10 et 11 comme vu dans le cours ? Ou on tente quelque choses de mieux ? (Contrôler, Datasheet & cours)

    // acquire a line from the camera, pixel 0 in line 10 to pixel IMAGE_BUFFER_SIZE in line 11
    // caution: two lines asked for internal reasons
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1)
    {
    	// start capturing the image
		dcmi_capture_start();
		// wait until completed
		wait_image_ready();
		// signal completeness with the semaphore
		chBSemSignal(&sensors_semaphore_image_ready);
    }
}

static THD_WORKING_AREA(wa_process_image, 1024);
static THD_FUNCTION(process_image, arg)
{
	// this function is used in a thread
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t * img_raw_RGB565_pixels = NULL;
	uint8_t   image[IMAGE_BUFFER_SIZE] = {0};

	static bool send_computer = true;
    while(1)
    {
    	// await the end of an image capture to be signaled
        chBSemWait(&sensors_semaphore_image_ready);
		// get the pointer to the array filled with the last image in RGB565
        img_raw_RGB565_pixels = dcmi_get_last_image_ptr();

        for(uint16_t i = 0 ; i < 2 * IMAGE_BUFFER_SIZE ; i += 2)
        	image[i/2] = (uint8_t)img_raw_RGB565_pixels[i]&0xF8; // red pixels

		/// @PI: ici traiter l'image


        //send_computer = !send_computer;
        //if(send_computer)
        	//debug_send_uint8_array_to_computer(image, IMAGE_BUFFER_SIZE);
        	debug_send_for_printlinke_uint16(img_raw_RGB565_pixels, IMAGE_BUFFER_SIZE);
        //chSysLock();
        //debug_am_i_responding();
        //chSysUnlock();
    }
}

void sensors_start(void)
{
	// image
	chThdCreateStatic(wa_process_image, sizeof(wa_process_image), NORMALPRIO, process_image, NULL);
	chThdCreateStatic(wa_acquire_image, sizeof(wa_acquire_image), NORMALPRIO, acquire_image, NULL);
}
