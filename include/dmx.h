/*
 * dmx512.h
 *
 *  Created on: 29 сент. 2021 г.
 *      Author: rbalykov
 */

#ifndef INCLUDE_DMX_H_
#define INCLUDE_DMX_H_

#include <linux/compiler_attributes.h>
#include <asm/byteorder.h>
#include <linux/types.h>

#include "util.h"

/*******************************************************************************
 * DATA FRAMING
 ******************************************************************************/

#define DMX_WORD_SIZE			(1)
#define DMX_PAYLOAD_MAX 		(512)
#define DMX_PAYLOAD_MIN			(24)
#define DMX_STARTCODE_BYTES 	(1)
#define DMX_STARTCODE_DMX512	(0)
#define DMX_FRAME_MAX 			(DMX_STARTCODE_BYTES + DMX_PAYLOAD_MAX)
#define DMX_FRAME_MIN 			(DMX_STARTCODE_BYTES + DMX_PAYLOAD_MIN)

/*******************************************************************************
 * PHYSICAL LAYER
 ******************************************************************************/

#define DMX_BAUDRATE 			(250000u)
#define DMX_FRAMERATE_MIN		(1u)
#define DMX_FRAMERATE_MAX		(44u)

// microseconds
#define DMX_BREAK_MIN			(92u)
#define DMX_BREAK_MAX			(1000000u)
#define DMX_MAB_MIN				(12u)
#define DMX_MAB_MAX				(1000000u)
#define DMX_SHORT_FRAME			(1204u)

// nanoseconds
#define DMX_SLOT_NSEC	(44000L)
#define DMX_SLOT_DUMMY	(49000L)


#define NORMALISE_BREAK(a) 		(a=TO_RANGE(a, DMX_BREAK_MIN, 	DMX_BREAK_MAX))
#define NORMALISE_MAB(a)   		(a=TO_RANGE(a, DMX_MAB_MIN, 	DMX_MAB_MAX))
#define NORMALISE_FRAMERATE(a)  (a=TO_RANGE(a, DMX_FRAMERATE_MIN, DMX_FRAMERATE_MAX))

#define DEFAULT_BREAK		(DMX_BREAK_MIN)
#define DEFAULT_MAB			(DMX_MAB_MIN)
#define DEFAULT_FRAMERATE	(DMX_FRAMERATE_MAX)

/*******************************************************************************
 * UART RX/TX STATE MACHINE
 ******************************************************************************/

typedef enum rx_state
{
	RX_IDLE = 0,
	RX_BREAK,
	RX_DATA,
	RX_FULL,
	RX_FAULT
}rx_state_t;

typedef enum tx_state
{
	TX_IDLE = 0,
	TX_ARMED,
	TX_BREAK,
	TX_MAB,
	TX_DATA
}tx_state_t;

struct uart_frame
{
	union __packed
	{
		uint8_t raw[DMX_FRAME_MAX];
		struct __packed
		{
		uint8_t startcode;
		uint8_t data[DMX_PAYLOAD_MAX];
		};
	};
	size_t size;
	rx_state_t state_rx;
	uint8_t flags;
};

/*******************************************************************************
 ******************************************************************************/

#endif /* INCLUDE_DMX_H_ */
