/*
 * enttec.c
 */


#include "enttec.h"

/*******************************************************************************
 * HELPER FUNCTIONS FOR write()
 ******************************************************************************/

static inline void wr_flush (struct ent_widget *widget)
{
	widget->wr_state = PRE_SOM;
	widget->wr_total = 0;
	widget->wr.size = 0;
}

static inline int wr_push  (struct ent_buffer *buffer, uint8_t byte)
{
	if (buffer->size < DMX_FRAME_MAX )
	{
		buffer->dmx.data[buffer->size] = byte;
		buffer->size++;
		return 0;
	}
	return -1;
}

static inline int wr_full (struct ent_buffer *buffer)
{
	if (buffer->size >= DMX_FRAME_MAX) return 1;
	if (buffer->size >= __le16_to_cpu(buffer->dmx.size)) return 1;
	return 0;
}

static void ent_reply(struct ent_widget *widget)
{
	if (widget->ops->recv)
		widget->ops->recv(widget, &widget->reply.dmx);
}

/*******************************************************************************
 ******************************************************************************/

static void ent_wr_dispatch(struct ent_widget *widget)
{
//	K_DEBUG("label %d, size %d", widget->wr.dmx.label,
//			__le16_to_cpu(widget->wr.dmx.size));

	switch (widget->wr.dmx.label)
	{
		// labels that generate replies
		case LABEL_PARAMS:
			if (widget->ops->params)
			{
				widget->ops->params(widget);
				ent_reply(widget);
			}
			break;
		case LABEL_SERIAL:
			if (widget->ops->serial)
			{
				widget->ops->serial(widget);
				ent_reply(widget);
			}
			break;
		case LABEL_VENDOR:
			if (widget->ops->vendor)
			{
				widget->ops->vendor(widget);
				ent_reply(widget);
			}
			break;
		case LABEL_NAME:
			if (widget->ops->name)
			{
				widget->ops->name(widget);
				ent_reply(widget);
			}
			break;
		case LABEL_FLASH_PAGE:
			if (widget->ops->flash)
			{
				widget->ops->flash(widget);
				ent_reply(widget);
			}
			break;

		// labels that don't need replies
		case LABEL_SET_PARAMS:
			if (widget->ops->set_params)
				widget->ops->set_params(widget, &widget->wr.dmx);
			break;

		// labels that bring DMX/RDM data to TX
		case LABEL_RDM_DISCOVERY:
			break;
		case LABEL_RDM_OUTPUT:
			break;
		case LABEL_DMX_OUTPUT:
		case LABEL_UNIVERSE_0:
			if (widget->ops->tx)
				widget->ops->tx(widget, &widget->wr.dmx, UNIVERSE_0);
			break;
		case LABEL_UNIVERSE_1:
			if (widget->ops->tx)
				widget->ops->tx(widget, &widget->wr.dmx, UNIVERSE_1);
			break;

		/*
		// unsupported labels

		case LABEL_RECEIVED_DMX:	// CAN'T BE INCOMING
			break;					// brings data from RX to host
		case LABEL_FLASH_FW:		// IGNORED
			break;					// prepare for f/w flashing
		case LABEL_ONCHANGE: 		// UNSUPPORTED
			break;					// due lack of documentation
		case LABEL_DATA_UPDATE: 	// UNSUPPORTED
			break;					// due lack of documentation
		*/
		default:
		break;
	}
}

/*******************************************************************************
 * EXPORTED HELPER FUNCTIONS
 ******************************************************************************/

inline size_t frame_rawsize(const union ent_frame *frame)
{
	return __le16_to_cpu(frame->size) + ENT_FRAME_OVERHEAD;
}

size_t 	ent_rx (struct ent_widget *widget, const char *src, size_t len, uint8_t flag)
{
	union ent_frame *frame = &widget->rx.dmx;
	if (!widget->ops->recv)
		return -1;

	frame->data[0] = flag;
	memcpy(&frame->data[ENT_FLAG_BYTES], src, len);
	frame->som = ENT_SOM;
	frame->label = LABEL_RECEIVED_DMX;
	frame->size = cpu_to_le16(len + ENT_FLAG_BYTES);
	frame->data[len + ENT_FLAG_BYTES] = ENT_EOM;

	widget->ops->recv(widget, frame);

	return 0;
}

size_t 	ent_write 	(struct ent_widget *widget,
		const char *src, size_t len)
{
	union ent_frame *frame = &widget->wr.dmx;
	size_t i;
	uint8_t byte;
	uint16_t size;

	for (i = 0; i < len; i++)
	{
		byte = src[i];

		if (widget->wr_state != PRE_SOM)
			widget->wr_total++;

		switch (widget->wr_state)
		{
		case PRE_SOM:
			if (byte == ENT_SOM)
			{
				widget->wr_state = GOT_SOM;
				widget->wr_total = 1;
			}
		break;

		case GOT_SOM:
			frame->label = byte;
			widget->wr_state = GOT_LABEL;
		break;

		case GOT_LABEL:
			frame->sz_lsb = byte;
			widget->wr_state = GOT_SIZE_LSB;
		break;

		case GOT_SIZE_LSB:
			frame->sz_msb = byte;
			size = __le16_to_cpu(frame->size);

			if (size  == 0)
				widget->wr_state = WAITING_FOR_EOM;
			else if (size > DMX_FRAME_MAX)
				{
					K_INFO("dropping broken frame, size '%d' \
							is greater than possible", size);
					wr_flush(widget);
				}
			else
				widget->wr_state = IN_DATA;
		break;

		case IN_DATA:
			wr_push(&widget->wr, byte);

			if (wr_full(&widget->wr))
				widget->wr_state = WAITING_FOR_EOM;
		break;

		case WAITING_FOR_EOM:
			if (byte == ENT_EOM)
				{
				widget->wr_state = PRE_SOM;
				ent_wr_dispatch(widget);
				break;
				}
			else if (widget->wr_total >= ENT_FRAME_MAX)
			{
				K_INFO("dropping oversized frame");
				wr_flush(widget);
			}
		break;
		}
	}
	return len;
}

/*******************************************************************************
 ******************************************************************************/
