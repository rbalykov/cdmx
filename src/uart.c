/*
 * uart.c
 */

#include <linux/tty.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/mutex.h>

//#include <asm-generic/ioctl.h>
//#include <uapi/asm-generic/ioctl.h>
#include <uapi/asm-generic/ioctls.h>
#include <asm/current.h>

#include "dmx.h"
#include "enttec.h"
#include "cdmx.h"
#include "debug.h"

extern struct uart_tx_ops tx_ops;

/*******************************************************************************
 * UART DMX TRANSMITTER
 ******************************************************************************/

#define TIMEOUT_NSEC   ( 1000000000L )      //1 second in nano seconds
#define TIMEOUT_SEC    ( 4 )                //4 seconds


static int tx_break_native (struct uart_tx *tx, bool on)
{
	return tx->tty->ops->break_ctl(tx->tty, on ? -1 : 0);
}

static int tx_break_ioctl (struct uart_tx *tx, bool on)
{
	return tx->tty->ops->ioctl(tx->tty, on ? TIOCSBRK : TIOCCBRK, 0);
}

static int tx_break_none (struct uart_tx *tx, bool on)
{
	return -EINVAL;
}

static size_t tx_read (struct uart_tx *tx, uint8_t *data, size_t size)
{
	mutex_lock(&tx->lock);
		size = MIN(size, DMX_FRAME_MAX);
		memcpy(&tx->frame.raw, data, size);
		tx->frame.size = size;
	mutex_unlock(&tx->lock);

	return size;
}

int tx_send_fifo (struct uart_tx *tx)
{
	struct cdmx_port *port = container_of(tx, struct cdmx_port, tx);
	ktime_t sleep, t1, t2;
	size_t size = 0, offset = 0;
	uint8_t *data;
	int breaktime, mabtime, maftime, framerate;

	if (mutex_trylock(&port->sysfs_lock))
	{
		breaktime 	= port->breaktime;
		mabtime 	= port->mabtime;
		maftime		= port->maftime;
		framerate 	= port->framerate;
		mutex_unlock(&port->sysfs_lock);
	}

	t1 = tx->timer.base->get_time();
	if (0 != tx->ops.break_ctl(tx, true))
	{
		pr_err_once("BRK setting failure");
	}
	sleep = breaktime * NSEC_PER_USEC;
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_hrtimeout(&sleep, HRTIMER_MODE_REL);

	if (0 != tx->ops.break_ctl(tx, false))
	{
		pr_err_once("BRK clearing failure");
	}

	sleep = mabtime * NSEC_PER_USEC;
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_hrtimeout(&sleep, HRTIMER_MODE_REL);

	mutex_lock(&tx->lock);
		size = tx->frame.size;
		data = tx->frame.raw;
		while (size)
		{
			offset = tx->tty->ops->write(tx->tty, data, size);
			data += offset;
			size -= offset;
		}
	mutex_unlock(&tx->lock);

	if (0 != tx->ops.wait(tx, maftime * NSEC_PER_USEC))
		return -1;

	t2 = tx->timer.base->get_time();

	sleep = (NSEC_PER_SEC/framerate) - (t2 - t1);
	if (sleep > 0)
	{
		set_current_state(TASK_UNINTERRUPTIBLE);
		if (schedule_hrtimeout(&sleep, HRTIMER_MODE_REL))
			return -1;
	}
	return 0;
}

int tx_send_none (struct uart_tx *tx)
{
	return -EINVAL;
}

int tx_wait_until_sent (struct uart_tx *tx, ktime_t maf)
{
	(void) maf;
	tx->tty->ops->wait_until_sent(tx->tty, 0);
	return 0;
}

int tx_wait_chars (struct uart_tx *tx, ktime_t maf)
{
	unsigned int i = tx->tty->ops->chars_in_buffer(tx->tty);
	ktime_t t;

	while(i)
	{
		t = i * DMX_SLOT_NSEC;
		set_current_state(TASK_UNINTERRUPTIBLE);
		if (schedule_hrtimeout(&t, HRTIMER_MODE_REL))
			return -1;
		i = tx->tty->ops->chars_in_buffer(tx->tty);
	}

	set_current_state(TASK_UNINTERRUPTIBLE);
	if (schedule_hrtimeout(&maf, HRTIMER_MODE_REL))
		return -1;
	return 0;
}

int tx_wait_dummy (struct uart_tx *tx, ktime_t maf)
{
	ktime_t t = maf + DMX_SLOT_NSEC * DMX_FRAME_MAX;
	set_current_state(TASK_INTERRUPTIBLE);
	if (schedule_hrtimeout(&t, HRTIMER_MODE_REL))
		return -1;
	return 0;
}

static bool tx_tty_validate (struct uart_tx *tx, struct tty_struct *tty)
{
	if (tty->ops->break_ctl)
	{
		K_DEBUG("using break_ctl");
		tx->ops.break_ctl = tx_break_native;
	}
	else
	if (tty->ops->ioctl)
	{
		K_DEBUG("using ioctl");
		tx->ops.break_ctl = tx_break_ioctl;
	}
	else
	{
		K_ERR("TTY %s doesn't have neither 'break_ctl' nor 'ioctl'", tty->name);
		tx->ops.break_ctl = tx_break_none;
		return false;
	}

	if (tty->ops->write)
	{
		K_DEBUG("using write");
		tx->ops.send = tx_send_fifo;
	}
	else
	{
		tx->ops.send = tx_send_none;
		K_ERR("TTY %s doesn't have write() op", tty->name);
		return false;
	}

	if (tty->ops->chars_in_buffer)
	{
		K_DEBUG("using chars_in_buffer");
		tx->ops.wait = tx_wait_chars;
	}
	else if (tty->ops->wait_until_sent)
	{
		K_DEBUG("using wait_until_sent");
		tx->ops.wait = tx_wait_until_sent;
	}
	else
	{
		K_ERR("TTY %s doesn't have neither 'wait_until_sent' nor 'chars_in_buffer' op", tty->name);
		tx->ops.wait = tx_wait_dummy;
		return false;
	}
	return true;
}

static void tx_thread_loop (struct uart_tx *tx)
{
	while(!kthread_should_stop())
    {
		if (0 != tx->ops.send(tx))
		{
			K_INFO("TX %s stopped", tx->name);
			return;
		}
    }
}

static int tx_thread (void *arg)
{
	struct uart_tx *tx = (struct uart_tx *) arg;

	K_DEBUG("thread start");
	hrtimer_init(&tx->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);

	tx_thread_loop(tx);

	tx->ops.break_ctl(tx, false);
    hrtimer_cancel(&tx->timer);
	tx->thread = NULL;
	K_DEBUG("->>>");
    return 0;
}

static int tx_start (struct uart_tx *tx)
{
	if ( ! tx->compliant)
	{
		K_DEBUG("Trying to start non-compliant TX");
		return -1;
	}

   	tx->thread = kthread_run(tx_thread, tx, tx->name);
   	if (tx->thread)
   	{
   		return 0;
   	}
   	return -1;
}

static void tx_stop (struct uart_tx *tx)
{
	if (tx->thread)
	{
		kthread_stop(tx->thread);
		tx->thread = NULL;
	}
}


int tx_attach (struct uart_tx *tx, struct tty_struct *tty)
{
	struct cdmx_port *port = container_of(tx, struct cdmx_port, tx);
	if (!tty)
	{
		K_ERR("Attaching TX to NULL TTY");
		return -EINVAL;
	}

	memcpy(&tx->ops, &tx_ops, sizeof(struct uart_tx_ops));
	tx->compliant = tx_tty_validate(tx, tty);
	if (!tx->compliant)
	{
		return -EINVAL;
	}

	tx->tty = tty;
	scnprintf(tx->name, TX_NAME_MAX, "cdmx %03X", port->id);
	memset(&tx->frame, 0, sizeof(struct uart_frame));
	tx->frame.size = DMX_FRAME_MAX;
	mutex_init(&tx->lock);

	return tx->ops.start(tx);
}

void tx_detach (struct uart_tx *tx)
{
	tx->ops.stop (tx);
	tx->tty = NULL;
	tx->compliant = false;
}



struct uart_tx_ops tx_ops =
{
	.start 		= tx_start,
	.stop		= tx_stop,
	.read		= tx_read,
	.break_ctl	= tx_break_native,
	.send		= tx_send_fifo,
	.wait		= tx_wait_chars
};

void tx_transmit (struct uart_tx *tx, uint8_t *data, size_t size)
{
	if (tx->compliant)
	{
		tx->ops.read(tx, data, size);
	}
}
/*******************************************************************************
 * UART DMX RECEIVER
 ******************************************************************************/

static inline void rx_dispatch  (struct uart_frame *frame)
{
	struct cdmx_port *port = container_of(frame, struct cdmx_port, rx);
//	K_DEBUG("%02X %02X %02X", frame->startcode,
//			frame->data[0], frame->data[511]);
	ent_rx(&port->widget, frame->raw, frame->size, frame->flags);
}

static inline void rx_reset  (struct uart_frame *frame)
{
	frame->state_rx = RX_IDLE;
	frame->size = 0;
	frame->flags = ENT_RX_CLEAR;
}

static inline void rx_break (struct uart_frame *frame)
{
	if ( (frame->state_rx == RX_DATA) && (frame->size >= DMX_FRAME_MIN))
		rx_dispatch(frame);

	frame->state_rx = RX_BREAK;
	frame->size = 0;
}

static inline void rx_start (struct uart_frame *frame, uint8_t sc)
{
	frame->state_rx = RX_DATA;
	frame->startcode = sc;
	frame->size = 1;
}

static inline void rx_data  (struct uart_frame *frame, uint8_t ch)
{
	frame->raw[frame->size++] = ch;
	if (frame->size >= DMX_FRAME_MAX)
	{
		rx_dispatch(frame);
		rx_reset(frame);
	}
}

static inline void rx_fault  (struct uart_frame *frame)
{
	if ( (frame->state_rx == RX_DATA) && (frame->size >= DMX_FRAME_MIN))
	{
		frame->flags |= ENT_RX_OVERRUN;
		rx_dispatch(frame);
	}

	frame->state_rx = RX_IDLE;
	frame->size  = 0;
}


void rx_process (struct uart_frame *frame,
		uint8_t ch, uint8_t flag)
{
	switch(frame->state_rx)
	{
	case RX_IDLE:
		if (flag == TTY_BREAK)
			rx_break(frame);
		break;
	case RX_DATA:
		switch (flag)
		{
			case TTY_NORMAL:
				rx_data(frame, ch);
			break;

			case TTY_BREAK:
			case TTY_FRAME:
				rx_break(frame);
			break;

			case TTY_PARITY:
			case TTY_OVERRUN:
			default:
				rx_fault(frame);
			break;
		}
		break;
	case RX_BREAK:
		if (flag == TTY_NORMAL)
			rx_start(frame, ch);
		break;
	case RX_FULL:
		break;
	case RX_FAULT:
		break;
	}
}
