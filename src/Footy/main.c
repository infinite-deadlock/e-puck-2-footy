//// ChibiOS & others
#include "hal.h"						// Hardware Abstraction Layer subsystem header
#include "memory_protection.h"			// Memory access permissions
#include "msgbus/messagebus.h"
#include <usbcfg.h>						// USB services
//
//// EPFL-MICRO 315 library
#include <motors.h>						// e-puck-2 motors control
#include <camera/dcmi_camera.h>			// e-puck-2 Digital CaMera Interface
#include <camera/po8030.h>				// e-puck-2 main frontal camera
#include <sensors/VL53L0X/VL53L0X.h>	// e-puck-2 Time Of Flight sensor
//
//// this project files
#include "sensors.h"
#include "move.h"
#include "central.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


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

	messagebus_init(&bus, &bus_lock, &bus_condvar);
	sensors_start();	// start all sensors for this project
	move_init_threads(); //start obstacle detection

	central_control_loop();
}

// stack guard
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("A lot of sadness: :-( :-( :-(\n   Stack smashing detected\n");
}
