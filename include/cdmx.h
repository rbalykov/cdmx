/*
 * cdmx.h
 *
 *  Created on: 5 ????. 2021 ?.
 *      Author: rbalykov
 */

#ifndef CDMX_H_
#define CDMX_H_

#include "enttec.h"

/*******************************************************************************
 *
 * Common stuff
 *
 ******************************************************************************/

#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)
#define TO_RANGE(a,min,max) (MIN(max,MAX(min,a)))



#define CHMOD_RW	(S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH)
#define CHMOD_RO	(S_IRUSR | S_IRGRP | S_IROTH)

/*******************************************************************************
 *
 * Module general
 *
 ******************************************************************************/

#define CDMX_PORTS_MAX		(0x100u)
#define CDMX_PORTS_MIN		(0x1u)
#define CDMX_PORTS_DEFAULT	(0x4u)
#define BASE_MINOR			(0u)
#define PORT_INACTIVE		(-1)
#define CDMX_BUFFERING		(2)

/*******************************************************************************
 *
 * DMX hardware
 *
 ******************************************************************************/

#define NORMALISE_BREAK(a) 		(a=TO_RANGE(a, 88, 1000000))
#define NORMALISE_MAB(a)   		(a=TO_RANGE(a, 8, 1000000))
#define NORMALISE_FRAMERATE(a)  (a=TO_RANGE(a, 1, 44))

#define DEFAULT_BREAK		(88)
#define DEFAULT_MAB			(8)
#define DEFAULT_FRAMERATE	(44)

#define CDMX_RECEIVE_ROOM		(1024)

static char	USBPRO_VENDOR[] 	=
{
	ESTA_DMXKING_LSB, ESTA_DMXKING_MSB,
	'D', 'M', 'X', 'k', 'i', 'n', 'g'
};
static char	USBPRO_NAME[] =
{
	DMXKING_512_LSB, DMXKING_512_MSB,
	'U', 'S', 'B', ' ', 'D', 'M', 'X', '5', '1', '2', '-', 'A', ' ',
	'E', 'm', 'u', 'l', 'a', 't', 'i', 'o', 'n'
};

/*******************************************************************************
 *
 * UART section
 *
 ******************************************************************************/
typedef enum rx_states
{
	IDLE = 0,
	DATA,
	BREAK,
	FULL,
	FAULT
}rx_states_t;

typedef enum rx_flags
{
	CLEAR		= 0,
	OVERFLOW 	= 1,
	OVERRUN 	= 2
} rx_flags_t;

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
	rx_states_t state;
	uint8_t flags;
};

/*******************************************************************************
 *
 * Line discipline
 *
 ******************************************************************************/

#define CDMX_LD		(28)


/*******************************************************************************
 *
 * Character device
 *
 ******************************************************************************/

#define CDMX_EXCLUSIVE		(1)

struct ringbuffer
{
	uint8_t data[CDMX_RECEIVE_ROOM];
	size_t size;
	size_t read;
	size_t write;
};

struct cdmx_port
{
	int id;

	struct kobject kobj;
	unsigned int breaktime;
	unsigned int mabtime;
	unsigned int framerate;


	struct ent_widget widget;
	struct ringbuffer readfrom;

	struct mutex tx_write_lock;
	struct mutex rx_read_lock;
	wait_queue_head_t wait;

	struct uart_frame rx;

	struct cdev cdev;
	struct device *fsdev;
	struct tty_struct *tty;

	//TODO: check it once againg and refactor
	struct tty_file_private *ttyfp;

	//TODO: refactor exclusive access
	unsigned long flags;
};


struct port_attribute
{
	struct attribute attr;
	ssize_t (*show) (struct cdmx_port *port,
			struct port_attribute *attr,
			char *buf);
	ssize_t (*store)(struct cdmx_port *port,
			struct port_attribute *attr,
			const char *buf,
			size_t count);
};


#endif /* CDMX_H_ */
