/*
 * cdmx.h
 *
 *  Created on: 5 ????. 2021 ?.
 *      Author: rbalykov
 */

#ifndef CDMX_H_
#define CDMX_H_

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <uapi/asm-generic/posix_types.h>
#include <uapi/linux/stat.h>
#include <linux/tty.h>
#include <linux/tty_ldisc.h>

#include "enttec.h"

/*******************************************************************************
 *
 * Common stuff
 *
 ******************************************************************************/

#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)
#define TO_RANGE(a,min,max) (MIN(max,MAX(min,a)))

#define _pp(severity, format, args...) \
  printk(severity "%s: %d: %s: " format "\n", THIS_MODULE->name, __LINE__, __func__, ##args)

#define K_WARN(args...) _pp(KERN_WARNING, args)
#define K_NOTE(args...) _pp(KERN_NOTICE, args)
#define K_INFO(args...) _pp(KERN_INFO, args)
#define K_ERR(args...) 	_pp(KERN_ERR, args)
#define K_DEBUG(args...) _pp(KERN_DEBUG, args)


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

/*******************************************************************************
 *
 * DMX-512 protocol
 *
 ******************************************************************************/

#define DMX_WORD_SIZE			(1)
#define DMX_PAYLOAD_MAX 		(512)
#define DMX_PAYLOAD_MIN			(24)
#define DMX_STARTCODE_BYTES 	(1)
#define DMX_STARTCODE_DMX512	(0)
#define DMX_FRAME_MAX 			(DMX_STARTCODE_BYTES + DMX_PAYLOAD_MAX)
#define DMX_FRAME_MIN 			(DMX_STARTCODE_BYTES + DMX_PAYLOAD_MIN)
#define DMX_BAUDRATE 			(250000)
#define CDMX_RECEIVE_ROOM		(1024)

static char	USBPRO_VENDOR[] 	= {ESTA_DMXKING_LSB, ESTA_DMXKING_MSB, 'D', 'M', 'X', 'k', 'i', 'n', 'g'};
static char	USBPRO_NAME[] 		= {DMXKING_512_LSB, DMXKING_512_MSB, 'U', 'S', 'B', ' ', 'D', 'M', 'X', '5', '1', '2', '-', 'A', ' ',
										   'E', 'm', 'u', 'l', 'a', 't', 'i', 'o', 'n'};

/*******************************************************************************
 *
 * UART section
 *
 ******************************************************************************/
typedef enum rx_states
{
	BREAK = 0,
	DATA,
	OVER,
	FAULT
}rx_states_t;

typedef enum rx_flags
{
	CLEAR		= 0,
	OVERFLOW 	= 1,
	OVERRUN 	= 2
} rx_flags_t;

struct uart_frame_a
{
	uint8_t data[DMX_FRAME_MAX];
	size_t size;
	rx_states_t state;
	uint8_t flags;
//	bool pending;
	struct uart_frame_a *next;
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

struct dmx_frame_a
{
	// Enttec part
	uint8_t msglabel;
	uint16_t msgsize;
	uint8_t flags;
	uint8_t data[ENT_FRAME_MAX];

	// reading raw stopped at this pointer
	size_t rhead;
	bool pending;

	// chrdev part to WRITE
	uint8_t rawdata[ENT_FRAME_MAX];
	size_t rawsize;
	ent_states_t state;

	// writing stopped at this pointer
	size_t whead;
};


typedef struct cdmx_frame
{
	struct ent_frame ent;
	ent_states_t state;
	size_t size;
	size_t write_ptr;
	size_t read_ptr;
	bool pending;
}cdmx_frame_t;

typedef struct framering
{
	cdmx_frame_t data[CDMX_BUFFERING];
	size_t size;
	size_t write_ptr;
	size_t read_ptr;
//	bool overrun;
}framering_t;

typedef struct ringbuffer
{
	uint8_t data[CDMX_RECEIVE_ROOM];
	size_t size;
	size_t write_ptr;
	size_t read_ptr;
//	bool overrun;
}ringbuffer_t;


struct dmx_port
{
	int id;
	struct kobject kobj;
	unsigned int breaktime;
	unsigned int mabtime;
	unsigned int framerate;


	struct mutex tx_write_lock;
	struct mutex rx_read_lock;
	wait_queue_head_t wait;

	ringbuffer_t ring_rx;
	ringbuffer_t ring_wr;

	framering_t frames_tx;
	framering_t frames_rd;

	cdmx_frame_t frame_wr;
	cdmx_frame_t frame_rd;

	//TODO: check it once againg and refactor
	struct cdev cdev;
	struct device *fsdev;
	struct tty_struct *tty;
	char *ttyname;

	//TODO: refactor exclusive access
	unsigned long flags;

	//TODO: remove it
	struct dmx_frame_a read_from;
	struct dmx_frame_a write_to;
	struct uart_frame_a tx;
	struct uart_frame_a rx[CDMX_BUFFERING];
	uint8_t rx_current;
	struct uart_frame_a *rx_pending;

};


struct port_attribute
{
	struct attribute attr;
	ssize_t (*show) (struct dmx_port *port,
			struct port_attribute *attr,
			char *buf);
	ssize_t (*store)(struct dmx_port *port,
			struct port_attribute *attr,
			const char *buf,
			size_t count);
};


#endif /* CDMX_H_ */
