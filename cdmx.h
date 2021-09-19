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

/*******************************************************************************
 *
 * Enttec protocol
 *
 ******************************************************************************/

#define ENT_FRAME_MAX		(605)
#define ENT_PAYLOAD_MAX		(600)
#define ENT_SOM 			(0x7E)
#define ENT_EOM 			(0xE7)

#define ENT_HEADER_SIZE		(4)
#define ENT_FOOTER_SIZE 	(1)
#define ENT_FRAME_OVERHEAD 	(ENT_HEADER_SIZE + ENT_FOOTER_SIZE)

#define ENT_FW_DMX			(1)
#define ENT_FW_RDM			(2)
#define ENT_FW_RDMSNIFFER 	(3)
#define ENT_FW_LSB			(0)

#define ENT_FLASH_PAGE		(64)
#define ENT_FLASH_REPLY		(4)
#define ENT_FLASH_TRUE  	("TRUE")
#define ENT_FLASH_FALSE 	("FALS")

#define ENT_PARAMS_MIN		(5)
#define ENT_TIMEUNIT_NS		(10670)
#define ENT_TIMEUNIT_DIV 	(10)
#define ENT_TIMEUNIT_MULT 	(10)

#define ENT_TIME_MAX		(127)
#define ENT_BREAK_MIN		(9)
#define ENT_MAB_MIN			(1)
#define ENT_FRAMERATE_MAX	(40)

#define ENT_FLAG_CLEAR		(0)
#define ENT_FLAG_OVERRUN	(0x01)

#define ENT_SEND_ALWAYS		(0)
#define ENT_SEND_ONCHANGE	(1)

#define ENT_ONCHANGE_SLOTS	(40)
#define ENT_SERIAL_SIZE		(4)
#define ENT_DISCOVERY_SLOTS	(38)

#define ESTA_DMXKING_LSB	(0x6B)
#define ESTA_DMXKING_MSB	(0x6A)
#define DMXKING_512_LSB		(0x00)
#define DMXKING_512_MSB		(0x00)
#define ENT_NAME_MAX		(32)

static char	USBPRO_VENDOR[] 	= {ESTA_DMXKING_LSB, ESTA_DMXKING_MSB, 'D', 'M', 'X', 'k', 'i', 'n', 'g'};
static char	USBPRO_NAME[] 		= {DMXKING_512_LSB, DMXKING_512_MSB, 'U', 'S', 'B', ' ', 'D', 'M', 'X', '5', '1', '2', '-', 'A', ' ',
										   'E', 'm', 'u', 'l', 'a', 't', 'i', 'o', 'n'};

typedef enum usb_states
{
	PRE_SOM = 0,
 	GOT_SOM = 1,
	GOT_LABEL = 2,
	GOT_SIZE_LSB = 3,
	IN_DATA = 4,
	WAITING_FOR_EOM = 5,
}usb_states_t;

typedef enum usb_labels
{
	LABEL_FLASH_FW 		= 1,
	/* REQUEST: This message requests the Widget firmware to run the Widget
	 * bootstrap to enable reprogramming of the Widget firmware.
	 * */
	LABEL_FLASH_PAGE 	= 2,
	/* REQUEST: This message programs one Flash page of the Widget firmware.
	 * The Flash pages must be programmed in order from first to last Flash
	 * page, with the contents of the firmware binary file.
	 * REPLY: The Widget sends this message to the PC on completion of the
	 * Program Flash Page request.
	 * */
	LABEL_GET_PARAMS 	= 3,
	/* REQUEST:This message requests the Widget configuration.
	 * REPLY: The Widget sends this message to the PC in response to the
	 * Get Widget Parameters request.
	 * */
	LABEL_SET_PARAMS 	= 4,
	/* REQUEST: This message sets the Widget configuration. The Widget
	 * configuration is preserved when the Widget loses power.
	 * */
	LABEL_RECEIVED_DMX 	= 5,
	/* REPLY: The Widget sends this message to the PC unsolicited, whenever
	 * the Widget receives a DMX or RDM packet from the DMX port, and the
	 * Receive DMX on Change mode is 'Send always'.
	 * */
	LABEL_DMX_OUTPUT	= 6,
	/* REQUEST: This message requests the Widget to periodically send a DMX packet
	 * out of the Widget DMX port at the configured DMX output rate.
	 * This message causes the widget to leave the DMX port direction as output
	 * after each DMX packet is sent, so no DMX packets will be received as a result
	 * of this request.
	 * The periodic DMX packet output will stop and the Widget DMX port direction
	 * will change to input when the Widget receives any request message other than
	 * the Output Only Send DMX Packet request, or the Get Widget Parameters request.
	 * */
	LABEL_RDM_OUTPUT	= 7,
	/* REQUEST: This message requests the Widget to send an RDM packet out of the
	 * Widget DMX port, and then change the DMX port direction to input, so that
	 * RDM or DMX packets can be received.
	 * */
	LABEL_ONCHANGE		= 8,
	/* REQUEST: This message requests the Widget send a DMX packet to the PC only
	 * when the DMX values change on the input port. By default the widget will
	 * always send, if you want to send on change it must be enabled by sending
	 * this message.
	 * This message also reinitializes the DMX receive processing, so that if
	 * change of state reception is selected, the initial received DMX data is
	 * cleared to all zeros.
	 * */
	LABEL_DATA_UPDATE 	= 9,
	/* The Widget sends one or more instances of this message to the PC
	 * unsolicited, whenever the Widget receives a changed DMX packet from the
	 * DMX port, and the Receive DMX on Change mode is 'Send on data change only'.
	 * */
	LABEL_GET_SERIAL 		= 10,
	/* REQUEST: This message requests the Widget serial number, which should be
	 * the same as that printed on the Widget case.
	 * REPLY: The Widget sends this message to the PC in response to the
	 * Get Widget Serial Number request.
	 * */
	LABEL_RDM_DISCOVERY = 11,
	/* This message requests the Widget to send an RDM Discovery Request packet
	 * out of the Widget DMX port, and then receive an RDM Discovery Response
	 * (see Received DMX Packet).
	 * */
	LABEL_VENDOR 		= 77,
	/* This message requests the device manufacturer information from the widget.
	 * */
	LABEL_NAME 			= 78,
	/* This message requests the device name information from the widget.
	 * */
	LABEL_RDM 			= 82,
	/* No datasheet found on it
	 * */
	LABEL_UNIVERSE_0	= 100,
	LABEL_UNIVERSE_1	= 101
	/* The DMX King UltraDMX Pro supports 2 universes output and 1 input
	 * simultaneously. They have added extensions to cater for these separate
	 * output universes:
	 * Label = 100 Output Only Send DMX Packet Request Universe 1
	 * on outputs 1 & 2 (same format as label 6)
	 * Label = 101 Output Only Send DMX Packet Request Universe 2
	 * on outputs 3 & 4 (same format as label When Label 6 data is received
	 * the ultraDMX Pro reverts back to standard mode and outputs 1
	 * universe data on outputs 1,2,3&4.
	 * */
} usbpro_labels_t;


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
	usb_states_t state;

	// writing stopped at this pointer
	size_t whead;
};


typedef union dmx_frame
{
	uint8_t raw[ENT_FRAME_MAX];
	struct
	{
		uint8_t som;
		uint8_t label;
		uint8_t sz_lsb;
		uint8_t sz_msb;
		uint8_t data[ENT_PAYLOAD_MAX];
		uint8_t eom;
	};
}dmx_frame_t;

typedef struct cdmx_frame
{
	dmx_frame_t dmx;
	usb_states_t state;
	size_t write_ptr;
	size_t read_ptr;
}cdmx_frame_t;

typedef struct frame_ring
{
	cdmx_frame_t fr[CDMX_BUFFERING];
	size_t write_ptr;
	size_t read_ptr;
	bool overrun;
}frame_ring_t;

typedef struct ringbuffer
{
	uint8_t data[CDMX_RECEIVE_ROOM];
	size_t write_ptr;
	size_t read_ptr;
	bool overrun;
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

	frame_ring_t frames_tx;
	frame_ring_t frames_rd;

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
