/*
 * cdmx.h
 */

#ifndef INCLUDE_CDMX_H_
#define INCLUDE_CDMX_H_

#include "enttec.h"

#include <linux/kthread.h>
#include <linux/hrtimer.h>

/*******************************************************************************
 * Module general
 ******************************************************************************/

#define CDMX_PORTS_MAX		(0x100u)
#define CDMX_PORTS_MIN		(0x1u)
#define CDMX_PORTS_DEFAULT	(0x4u)
#define CDMX_BASE_MINOR		(0u)
#define PORT_INACTIVE		(-1)

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
 * Line discipline
 ******************************************************************************/

#define CDMX_LD				(28)
#define CDMX_RECEIVE_ROOM	(1024)

/*******************************************************************************
 * Character device
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

	struct task_struct *thread;
	struct hrtimer timer;

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

/*******************************************************************************
 ******************************************************************************/

#endif /* INCLUDE_CDMX_H_ */
