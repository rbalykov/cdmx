/*
 * cdmx.h
 */

#ifndef INCLUDE_CDMX_H_
#define INCLUDE_CDMX_H_

#include "enttec.h"
#include "uart.h"

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


/*******************************************************************************
 * Line discipline
 ******************************************************************************/

#define CDMX_LD				(28)
#define CDMX_RECEIVE_ROOM	(1024)

/*******************************************************************************
 * Character device
 ******************************************************************************/

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

	// SysFS interface
	struct kobject kobj;

	// microseconds
	unsigned int breaktime;
	unsigned int mabtime;
	unsigned int maftime;
	unsigned int framerate;
	struct mutex sysfs_lock;

	// CHRDEV interface
	struct cdev cdev;
	struct device *fsdev;

	// Line Discipline
	struct mutex ld_lock;
	struct tty_struct *tty;

	// read() to TX
	struct mutex tx_read_lock;

	// RX to write()
	struct mutex rx_write_lock;

	// read()
	struct ringbuffer readfrom;
	wait_queue_head_t wait;

	// write()
	struct ent_widget widget;

	// UART RX
	struct uart_frame rx;

	// UART TX
	struct uart_tx tx;
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
