/*
 * enttec.h
 *
 *  Created on: 21 сент. 2021 г.
 *      Author: rbalykov
 */

#ifndef ENTTEC_H_
#define ENTTEC_H_

#include <linux/compiler_attributes.h>
#include <asm/byteorder.h>
#include <linux/types.h>

#include "debug.h"

typedef enum ent_labels
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
	LABEL_PARAMS 	= 3,
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
	LABEL_SERIAL 		= 10,
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
} ent_labels_t;

enum ent_state
{
	PRE_SOM = 0,
 	GOT_SOM = 1,
	GOT_LABEL = 2,
	GOT_SIZE_LSB = 3,
	IN_DATA = 4,
	WAITING_FOR_EOM = 5,
};

enum ent_universe
{
	UNIVERSE_0 = 0,
	UNIVERSE_1,
	UNIVERSE_2,
	UNIVERSE_3
};

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
#define ENT_FLAG_BYTES		(1)

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


union __packed ent_frame
{
	uint8_t raw[ENT_FRAME_MAX];
	struct __packed
	{
		uint8_t som;
		uint8_t label;
		union __packed
		{
			__le16 size;
			struct __packed
			{
				uint8_t sz_lsb;
				uint8_t sz_msb;
			};
		};
		uint8_t data[ENT_PAYLOAD_MAX];
		uint8_t eom;
	};
};

struct ent_buffer
{
	union ent_frame dmx;
	size_t read_ptr;
	size_t write_ptr;
//	size_t size;
};

struct ent_widget
{
	struct ent_buffer wr;
	enum ent_state wr_state;
	size_t wr_total;

	struct ent_buffer rx;
	struct ent_buffer reply;
	struct ent_ops *ops;
};

struct ent_ops
{
	void (*set_params) (struct ent_widget *);
	void (*recv)	(struct ent_widget *, union ent_frame *);
	void (*tx) 		(struct ent_widget *, union ent_frame *, int universe);
	void (*params) 	(struct ent_widget *);
	void (*serial) 	(struct ent_widget *);
	void (*vendor) 	(struct ent_widget *);
	void (*name) 	(struct ent_widget *);
	void (*flash) 	(struct ent_widget *);

};

/*
 * Datasheet on labels ONCHANGE & UPDATE looks like a crap.
 * Neither forums nor other source found to make it clear,
 * so decision is made to leave ONCHANGE & UPDATE labes unsupported.
 *
 *  	void (*onchange)	(struct ent_widget *);
 *
 * This message also reinitializes the DMX receive processing,
 * so that if change of state reception is selected,
 * the initial received DMX data is cleared to all zeros.
 *
 * 		void (*update)  	(struct ent_widget *);
 *
 * It describes 1 byte as frame offset for 40-slot subframe.
 * So, 256 + 40 = insuffitient to address 512-slots frame.
 *
 * */

size_t 	frame_rawsize	(const union ent_frame *frame);

size_t 	ent_write 	(struct ent_widget *widget, const char *src, size_t len);
size_t 	ent_rx 	 	(struct ent_widget *widget, const char *src, size_t len, uint8_t flag);


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


#endif /* ENTTEC_H_ */

