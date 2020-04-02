//  Standard Library
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// ChibiOS & others
#include "ch.h"						// main include files
#include "hal.h"					// Hardware Abstraction Layer subsystem header
#include "memory_protection.h"		// Memory access permissions
#include <usbcfg.h>					// USB services
#include <chprintf.h>				// mini printf-like functionality

// EPFL-MICRO 315 library
#include <motors.h>					// e-puck-2 motors control
#include <camera/dcmi_camera.h>		// e-puck-2 Digital CaMera Interface
#include <camera/po8030.h>			// e-puck-2 main frontal camera

// this project files
#include "debug.h"
#include "sensors.h"


static void serial_start(void)
{
	static SerialConfig serial_config = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &serial_config); // communication: UART3
}

int main(void)
{
	halInit();			// Hardware Abstraction Layer subsystem init
	chSysInit();		// Initializes the kernel
	mpu_init();			// Initializes Memory Protection Unit

	serial_start();		// serial communications starting
	usb_start();		// Universal Serial Bus communications starting
	dcmi_start();		// Digital CaMera Interface starting
	po8030_start();		// camera clock generation starting (PixelPlus PO8030)
	motors_init();		// motors starting

	sensors_start();	// start all sensors for this project

	while(1)
	{
		chThdSleepMilliseconds(1000);
		//debug_am_i_responding();
	}
}

// stack guard
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("A lot of sadness: :-( :-( :-(\n   Stack smashing detected\n");
}
