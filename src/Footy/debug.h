#ifndef DEBUG_H_
#define DEBUG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>			// standard library
#include <string.h>			// string manipulation

#include "hal.h"			// Hardware Abstraction Layer subsystem header
#include <chprintf.h>		// mini printf-like functionality


void debug_am_i_responding(void);
void debug_send_uint32_to_computer(uint32_t data);
void debug_send_for_printlinke_couple_uint8(uint8_t * data, uint16_t size);
void debug_send_uint8_array_to_computer(uint8_t* data, uint16_t size);


#ifdef __cplusplus
}
#endif

#endif // DEBUG_H_
