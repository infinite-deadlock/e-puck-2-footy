#include "debug.h"

void debug_send_uint8_array_to_computer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*) "START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*) &size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*) data, size);
}

void debug_send_uint32_to_computer(uint32_t data)
{
	chprintf((BaseSequentialStream *)&SD3, "new data: %d\n", data);
}

void debug_send_for_printlinke_couple_uint8(uint8_t * data, uint16_t size)
{
	chprintf((BaseSequentialStream *)&SD3, "NEW_IMAGE:\n");

	for(uint16_t i = 0 ; i < size ; i += 2)
		chprintf((BaseSequentialStream *)&SD3, "%d %d\n", data[i], data[i + 1]);

	if((size & 1) != 0)
		chprintf((BaseSequentialStream *)&SD3, "%d SINGLE !\n", data[(size / 2) + 1]);

	chprintf((BaseSequentialStream *)&SD3, "END_IMAGE:\n");
}

void debug_am_i_responding(void)
{
	chprintf((BaseSequentialStream *)&SD3, "e-puck-2 is alive !\n");
}
