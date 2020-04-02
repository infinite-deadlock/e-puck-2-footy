#include "debug.h"

void debug_send_uint8_array_to_computer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*) "START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*) &size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*) data, size);
}

void debug_am_i_responding(void)
{
	chprintf((BaseSequentialStream *)&SD3, "e-puck-2 is alive !\n");
}
