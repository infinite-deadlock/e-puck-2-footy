#ifndef DEBUG_H_
#define DEBUG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>			// standard library
#include <string.h>			// string manipulation

#include "hal.h"			// Hardware Abstraction Layer subsystem header
#include <chprintf.h>		// mini printf-like functionality

// Debug functions

/**
* @brief   Sends a string to test communication
*		True if the ball was found
*
*/
void debug_am_i_responding(void);
/**
* @brief   Sends an integer
*
* @param data			Integer to send
*
*/
void debug_send_uint32_to_computer(uint32_t data);
/**
* @brief   Sends a buffer of integer
*
* @param data			Buffer to send
* @param size			Size of the buffer is nb uint8_t
*
*/
void debug_send_for_printlinke_couple_uint8(uint8_t * data, uint16_t size);
/**
* @brief   Sends a buffer of integer
*
* @param data			Buffer to send
* @param size			Size of the buffer
*
*/
void debug_send_uint8_array_to_computer(uint8_t* data, uint16_t size);


#ifdef __cplusplus
}
#endif

#endif // DEBUG_H_
