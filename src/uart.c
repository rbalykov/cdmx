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
#include <asm-generic/current.h>

#include "debug.h"
#include "dmx.h"
#include "enttec.h"
#include "cdmx.h"

extern struct uart_tx_ops tx_ops;

/*******************************************************************************
 * UART DMX TRANSMITTER
 ******************************************************************************/

#define TIMEOUT_NSEC   ( 1000000000L )      //1 second in nano seconds
#define TIMEOUT_SEC    ( 4 )                //4 seconds
/*
enum hrtimer_restart tx_timer (struct hrtimer *timer)
{
    hrtimer_forward_now(timer,ktime_set(TIMEOUT_SEC, TIMEOUT_NSEC));
    return HRTIMER_RESTART;
}

void tx_timer_start (struct cdmx_port *port)
{
    ktime_t ktime;

    ktime = ktime_set(TIMEOUT_SEC, TIMEOUT_NSEC);
    hrtimer_init(&port->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    port->timer.function = &tx_timer;
    hrtimer_start( &port->timer, ktime, HRTIMER_MODE_REL);
}

void tx_timer_stop (struct cdmx_port *port)
{
    hrtimer_cancel(&port->timer);
}

static void tty_write_unlock(struct tty_struct *tty)
{
	mutex_unlock(&tty->atomic_write_lock);
	wake_up_interruptible_poll(&tty->write_wait, EPOLLOUT);
}

static int tty_write_lock(struct tty_struct *tty, int ndelay)
{
	if (!mutex_trylock(&tty->atomic_write_lock))
	{
		if (ndelay)
			return -EAGAIN;

		if (mutex_lock_interruptible(&tty->atomic_write_lock))
			return -ERESTARTSYS;
	}
	return 0;
}

static int send_break(struct tty_struct *tty, unsigned int duration)
{
	int retval;

	if (tty->ops->break_ctl == NULL)
		return 0;

	if (tty->driver->flags & TTY_DRIVER_HARDWARE_BREAK)
		retval = tty->ops->break_ctl(tty, duration);
	else
	{
		//  Do the work ourselves
		if (tty_write_lock(tty, 0) < 0)
			return -EINTR;

		retval = tty->ops->break_ctl(tty, -1);
		if (retval)
			goto out;

		if (!signal_pending(current))
			msleep_interruptible(duration);

		retval = tty->ops->break_ctl(tty, 0);

	out:
		tty_write_unlock(tty);
		if (signal_pending(get_current()))
			retval = -EINTR;
	}
	return retval;
}
*/
/*
static int tx_set_break (struct tty_struct *tty, bool on)
{
	if (tty->ops->break_ctl)
	{
		pr_debug_once("tty %s, using BREAK op", tty->name);
		return tty->ops->break_ctl(tty, on ? -1 : 0 );

	}
	else if (tty->ops->ioctl)
	{
		pr_debug_once("tty %s has no BREAK op, using IOCTL", tty->name);
		return tty->ops->ioctl(tty, on ? TIOCSBRK : TIOCCBRK, 0);
	}
	pr_err_once("tty %s doesn't have neither BREAK nor IOCTL ops", tty->name);
	return -EINVAL;
}

void tx_send_frame (struct cdmx_port *port)
{
//	struct uart_frame *frame = &port->tx;
	struct tty_struct *tty = port->tty;
	ktime_t sleep, s1, s2;

	if (0 != tx_set_break(tty, true))
		return;

	sleep = port->breaktime * NSEC_PER_USEC;
	set_current_state(TASK_UNINTERRUPTIBLE);
	s1 = port->tx.timer.base->get_time();
	schedule_hrtimeout(&sleep, HRTIMER_MODE_REL);
	s2 = port->tx.timer.base->get_time();
	K_DEBUG("%lld", s2-s1);

	if (0 != tx_set_break(tty, false))
		return;

}
*/
/*
 * schedule_hrtimeout
 * schedule_hrtimeout_range
 * hrtimer_sleeper_start_expires
 *
 */


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
	int breaktime, mabtime, framerate;

	K_DEBUG("<---");
	if (mutex_trylock(&port->sysfs_lock))
	{
		breaktime 	= port->breaktime;
		mabtime 	= port->mabtime;
		framerate 	= port->framerate;
		mutex_unlock(&port->sysfs_lock);
	}

	t1 = tx->timer.base->get_time();
	if (0 != tx->ops.break_ctl(tx, true))
		return -1;

	sleep = breaktime * NSEC_PER_USEC;
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_hrtimeout(&sleep, HRTIMER_MODE_REL);

	if (0 != tx->ops.break_ctl(tx, false))
		return -1;

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
			tty_wait_until_sent(tx->tty, 0);
		}
		t2 = tx->timer.base->get_time();
	mutex_unlock(&tx->lock);

	sleep = (NSEC_PER_SEC/framerate) - (t2 - t1);
	if (sleep > 0)
	{
//		set_current_state(TASK_UNINTERRUPTIBLE);
//		schedule_hrtimeout(&sleep, HRTIMER_MODE_REL);
	}

	K_DEBUG("->>>");
	return 0;
}

int tx_send_none (struct uart_tx *tx)
{
	return -EINVAL;
}

static bool tx_tty_validate (struct uart_tx *tx, struct tty_struct *tty)
{
	K_DEBUG("<---");
	if (tty->ops->break_ctl)
		tx->ops.break_ctl = tx_break_native;
	else if (tty->ops->ioctl)
		tx->ops.break_ctl = tx_break_ioctl;
	else
	{
		K_ERR("TTY %s doesn't have neither break_ctl nor ioctl",
				tty->name);
		tx->ops.break_ctl = tx_break_none;
		return false;
	}

	if (tty->ops->write)
		tx->ops.send = tx_send_fifo;
	else
	{
		tx->ops.send = tx_send_none;
		K_ERR("TTY %s doesn't have write() op", tty->name);
		return false;
	}
	K_DEBUG("->>>");
	return true;
}

static int tx_thread (void *arg)
{
	struct uart_tx *tx = (struct uart_tx *) arg;

	K_DEBUG("<---");
	while(!kthread_should_stop())
    {
		tx->ops.send(tx);
    }
	K_DEBUG("->>>");
    return 0;
}

static int tx_start (struct uart_tx *tx)
{
	K_DEBUG("<---");
	if ( ! tx->compliant)
	{
		pr_debug("Trying to start non-compliant TX");
		return -1;
	}

	hrtimer_init(&tx->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
   	tx->thread = kthread_run(tx_thread, tx, tx->name);
   	if (tx->thread)
   	{
   		K_DEBUG("->>>");
   		return 0;
   	}
	K_DEBUG("no thread");
   	return -1;
}

static void tx_stop (struct uart_tx *tx)
{
	K_DEBUG("<---");
	if (tx->thread)
	{
		kthread_stop(tx->thread);
		tx->thread = NULL;
	}
    hrtimer_cancel(&tx->timer);
	K_DEBUG("->>>");
}


int tx_attach (struct uart_tx *tx, struct tty_struct *tty)
{
	struct cdmx_port *port = container_of(tx, struct cdmx_port, tx);
	K_DEBUG("<---");
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
	tx->frame.size = DMX_FRAME_MIN;
	mutex_init(&tx->lock);

	K_DEBUG("->>> return ops.start");
	return tx->ops.start(tx);
}

void tx_detach (struct uart_tx *tx)
{
	K_DEBUG("<---");
	tx->ops.stop (tx);
	tx->tty = NULL;
	tx->compliant = false;
	K_DEBUG("->>>");
}



struct uart_tx_ops tx_ops =
{
	.start 		= tx_start,
	.stop		= tx_stop,
	.read		= tx_read,
	.break_ctl	= tx_break_none,
	.send		= tx_send_none,
};
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
