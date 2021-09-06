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
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/mutex.h>


#define _CDMX_DEBUG(x)	x
#define _CDMX_INFO(x)	x
#define _CDMX_ERR(x) 	x


//#define cdmx_port_count		(4u)
#define BASE_MINOR		(0u)

#define NOT_VALID		(-1)
#define CDEV_VALID		(1)

#define port_active(p) 		((p) != NOT_VALID)

#define CHMOD_RW	0666
#define CHMOD_RO	0444
//NOTE S_IRUGO


#define PORT_COUNT_MAX	(0xFFu)
#define PORT_COUNT_MIN	(0x4u)

#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)

#define DMX_WORD_SIZE			(1)
#define DMX_PAYLOAD_MAX 		(512)
#define DMX_PAYLOAD_MIN			(24)
#define DMX_STARTCODE_BYTES 	(1)
#define DMX_STARTCODE_DMX512	(0)
#define DMX_FRAME_MAX 			(DMX_STARTCODE_BYTES + DMX_PAYLOAD_MAX)
#define DMX_FRAME_MIN 			(DMX_STARTCODE_BYTES + DMX_PAYLOAD_MIN)
#define DMX_BAUDRATE 			(250000)

#define DMX_USBPRO_FRAME_MAX	(605)
#define DMX_USBPRO_PAYLOAD_MAX	(600)
#define DMX_USBPRO_MAGIC_START 	(0x7E)
#define DMX_USBPRO_MAGIC_END 	(0xE7)


#define DMX_R_BUFFERING	(2u)
#define DMX_W_BUFFERING	(2u)


static int cdmx_open 		(struct inode *, struct file *);
static int cdmx_release 	(struct inode *, struct file *);
static ssize_t cdmx_read 	(struct file*, char*, size_t, loff_t *);
static ssize_t cdmx_write 	(struct file*, const char*, size_t, loff_t *);
static int 	__init cdmx_init	(void);
static void __exit cdmx_exit	(void);

ssize_t	ttyname_show	(struct kobject *, struct attribute *, char *);
ssize_t	ttyname_store	(struct kobject *, struct attribute *, const char *, size_t);

static char *cdmx_devnode(struct device *dev, umode_t *mode);

typedef enum usb_states
{
	PRE_SOM = 0,
 	GOT_SOM = 1,
	GOT_LABEL = 2,
	GOT_DATA_LSB = 3,
	IN_DATA = 4,
	WAITING_FOR_EOM = 5,
}usb_states_t;

typedef enum uart_flags
{
	CLEAR		= 0,
	OVERFLOW 	= 1,
	OVERRUN 	= 2
} uart_flags_t;

typedef enum uart_states
{
	IDLE = 0,
	BREAK,
	SC,
	DATAFLOW,
	OVERFILL
}uart_states_t;

struct usb_frame
{
	uint8_t label;
	uint16_t datasize;
	uint8_t flags;
	uint8_t data[DMX_USBPRO_PAYLOAD_MAX];
	size_t size;
	size_t head;
	size_t stream;
	usb_states_t state;
};

struct uart_frame
{
	uint8_t data[DMX_FRAME_MAX];
	size_t size;
	size_t head;
	uart_states_t state;
	uart_flags_t flags;
};

struct ttyname
{
	struct attribute attr;
	char *file;
};

struct cdmx_data
{
	struct cdev cdev;
	bool cdev_valid;
	struct device *fsdev;
	int port_id;
	struct ttyname tty;

	struct mutex read_mutex;
	struct mutex write_mutex;

	struct usb_frame read_from;
	struct usb_frame write_to;

	struct uart_frame tx;
	struct uart_frame rx;
};



#endif /* CDMX_H_ */
