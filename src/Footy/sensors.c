#include "sensors.h"

//  Standard Library
#include <math.h>

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
#include "move.h"
#include "central.h"

// local defines
#define IMAGE_BUFFER_SIZE	640		// size of the image acquired, in RGB565 pixel
#define IMAGE_LINE_HEIGHT	240		// captured line height
#define BALL_DIAMETER		38		// ball diameter, in mm (official: 38.50 mm)

// semaphores
static BSEMAPHORE_DECL(sensors_semaphore_image_ready_for_process, TRUE);
static BSEMAPHORE_DECL(sensors_semaphore_image_completed, TRUE);

// threaded functions
static THD_WORKING_AREA(wa_acquire_image, 256);
static THD_FUNCTION(acquire_image, arg)
{
	// this function is used in a thread
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	// acquire a line from the camera, from pixel 0 in line IMAGE_LINE_HEIGHT of size IMAGE_BUFFER_SIZE x 2 // @PI a revoir
    // caution: two lines asked for internal reasons

	if(po8030_advanced_config(FORMAT_RGB565, 0, IMAGE_LINE_HEIGHT, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1) != MSG_OK)
		chprintf((BaseSequentialStream *)&SD3, "Error po8030_advanced_config\n");
	//if(po8030_set_awb(0) != MSG_OK)
		//chprintf((BaseSequentialStream *)&SD3, "Error po8030_set_awb\n");

	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1)
    {
    	// await the end of an image capture to be signaled
    	chBSemWait(central_get_semaphore_authorization_acquire());
    	// start capturing the image
		dcmi_capture_start();
		// wait until completed
		wait_image_ready();
		// signal completeness with the semaphore
		chBSemSignal(&sensors_semaphore_image_completed);
		chBSemSignal(&sensors_semaphore_image_ready_for_process);
    }
}

static THD_WORKING_AREA(wa_process_image, 1024);
static THD_FUNCTION(process_image, arg)
{
	// this function is used in a thread
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t * img_raw_RGB565_pixels = NULL;
	uint8_t   red_pixels[IMAGE_BUFFER_SIZE] = {0};

	int8_t diff_pixels;

	int8_t best_diff_pixels_positiv;
	int8_t best_diff_pixels_negativ;
	float best_angle_positiv;
	float best_angle_negativ;

	//static bool s_send_computer = true;
    while(1)
    {
    	// await the end of an image capture to be signaled
        chBSemWait(&sensors_semaphore_image_ready_for_process);
		// get the pointer to the array filled with the last image in RGB565
        img_raw_RGB565_pixels = dcmi_get_last_image_ptr();


        for(uint16_t i = 0 ; i < 2 * IMAGE_BUFFER_SIZE ; i += 2)
        	red_pixels[i/2] = (img_raw_RGB565_pixels[i] >> 3) & 31; // red pixels

        // le problème est que maintenant qu'il y a l'ajout des angles, c'est le foncitonnement de detection qui semble ne plus jouer
        /*best_diff_pixels_positiv = 0;
        best_diff_pixels_negativ = 0;
        for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE - 3 ; ++i)
        {
        	diff_pixels = red_pixels[i] - red_pixels[i + 3];
        	if(abs(diff_pixels) >= 8)
        	{
        		chprintf((BaseSequentialStream *)&SD3, "found %d\n", diff_pixels >= 0);
        		if(diff_pixels > best_diff_pixels_positiv)
        		{
        			best_diff_pixels_positiv = diff_pixels;

					if(i <= 320)
						best_angle_positiv = -(180 * (atan((1-((float)i/320))*0.414213562)) / M_PI);
					else
						best_angle_positiv = (180 * (atan((((float)i/320)-1)*0.414213562)) / M_PI);
        		}
        		else if(diff_pixels < best_diff_pixels_negativ)
        		{
        			best_diff_pixels_negativ = diff_pixels;

					if(i <= 320)
						best_angle_negativ = -(180 * (atan((1-((float)i/320))*0.414213562)) / M_PI);
					else
						best_angle_negativ = (180 * (atan((((float)i/320)-1)*0.414213562)) / M_PI);
        		}

        	}
        }

        if(best_diff_pixels_positiv != 0)
        {
        	chprintf((BaseSequentialStream *)&SD3, "found + %f\n", best_angle_positiv);
        }
        if(best_diff_pixels_negativ != 0)
        {
        	chprintf((BaseSequentialStream *)&SD3, "found - %f\n", best_angle_negativ);
        }*/

		//debug_send_uint8_array_to_computer(red_pixels, IMAGE_BUFFER_SIZE);

		debug_send_for_printlinke_couple_uint8(img_raw_RGB565_pixels, 2 * IMAGE_BUFFER_SIZE);
    }
}

void sensors_start(void)
{
	// image
	chThdCreateStatic(wa_process_image, sizeof(wa_process_image), NORMALPRIO, process_image, NULL);
	chThdCreateStatic(wa_acquire_image, sizeof(wa_acquire_image), NORMALPRIO, acquire_image, NULL);
}

void * central_get_semaphore_authorization_move(void)
{
	return &sensors_semaphore_image_completed;
}
