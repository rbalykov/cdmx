/*******************************************************************************
 *
 * CDMX - Linux kernel module to turn any DMX capable UART
 * into Enttec-compatible device.
 *
 * (C) 2021 Neutrino
 * Licensed with GNU GPL v3 or any later version
 *
 * Usage:
 * - make sure your UART supports BRKINT (man termios) and 250'000 bps
 * - get Linux kernel v5
 * - build a module
 * - attach line discipline to UART
 * - use /dev/cdmx00x as Enttec widget
 * - tweak it using /sys/cdmx/port00x/
 *
 ******************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/tty.h>
#include <linux/tty_ldisc.h>

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/ctype.h>

#include <uapi/linux/stat.h>
#include <uapi/linux/eventpoll.h>
#include <uapi/asm-generic/posix_types.h>
#include <uapi/asm-generic/errno-base.h>
#include <uapi/asm-generic/ioctls.h>
#include <uapi/asm-generic/termbits.h>

#include "cdmx.h"

/*******************************************************************************
 ******************************************************************************/

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Neutrino");
MODULE_DESCRIPTION("CDMX character device");
MODULE_VERSION("0.01");

/*******************************************************************************
 ******************************************************************************/

/*
 * NUMBER OF I/O PORTS
 * use it while inserting module or kernel boot cmdline
 * min 1, max 256, 4 by default
 */
int cdmx_port_count = CDMX_PORTS_DEFAULT;
module_param(cdmx_port_count, hexint, CHMOD_RO);
static struct cdmx_port 	**cdmx_ports;

// CHARACTER DEVICE
static struct class 		*cdmx_devclass 	= NULL;
static dev_t 				cdmx_device_id;

// SYSFS ATTRIBUTES
static struct kset 			*cdmx_ports_kset;

// LINE DISCIPLINE
//TODO: check out if it's useful or not
static struct mutex 		cld_lock;

// IN-FUNCTION BUFFER (see cdmx_read/write())
//TODO: tweak it
#define	LOCALBUF_SIZE	(16)

/*******************************************************************************
 * RING BUFFER FOR read()
 ******************************************************************************/

static inline size_t ring_freespace (const struct ringbuffer *ring)
{
	return CDMX_RECEIVE_ROOM - ring->size;
}

static inline void ring_do_push (struct ringbuffer *ring, uint8_t ch)
{
	ring->data[ring->write++] = ch;
	ring->size++;
	ring->write %= CDMX_RECEIVE_ROOM;
}

static inline void ring_do_pop (struct ringbuffer *ring, uint8_t *ch)
{
	*ch = ring->data[ring->read++];
	ring->size--;
	ring->read %= CDMX_RECEIVE_ROOM;
}

static size_t ring_push (struct ringbuffer *ring,
		const uint8_t *buf, size_t len)
{
	size_t i = 0;
//	K_DEBUG("len %d", len);

	if ( (!len ) || (len > ring_freespace(ring)) )
		return -1;

	for (; i < len; i++)
		ring_do_push(ring, buf[i]);

//	K_DEBUG("space left %d", ring_freespace(ring));
	return 0;
}

static size_t ring_pop (struct ringbuffer *ring, uint8_t *buf, size_t len)
{
	size_t i, size = MIN (len, ring->size);
//	K_DEBUG("len %d", len);
	for (i = 0; i < size; i++)
	{
		ring_do_pop(ring, buf+i);
	}
//	K_DEBUG("done %d", size);
	return size;
}

static inline void ring_unpop (struct ringbuffer *ring, size_t len)
{
//	K_DEBUG("len %d", len);
	ssize_t read = ring->read - len;
	if ((ring->size + len) > CDMX_RECEIVE_ROOM)
		return;
	ring->read = (read < 0) ? (CDMX_RECEIVE_ROOM + read) : read;
	ring->size += len;
}

/*******************************************************************************
 * UART DMX TRANSMITTER
 ******************************************************************************/

#define TIMEOUT_NSEC   ( 1000000000L )      //1 second in nano seconds
#define TIMEOUT_SEC    ( 4 )                //4 seconds

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

int tx_thread (void *arg)
{
	struct cdmx_port *port = (struct cdmx_port *) arg;
	(void) port;

	return 0;
}

int tx_thread_create (struct cdmx_port *port)
{
	char *name = "cdmx tx___";
	sprintf(name, "cdmx tx%03X", port->id);

#if 1
    port->thread = kthread_create(tx_thread, port, name);
    if(port->thread)
    {
        wake_up_process(port->thread);
        return 0;
    }
#else
    port->thread = kthread_run(tx_thread, port, name);
    if(!port->thread)
    	return 0;
#endif
    else
    	return -1;
}

void tx_thread_stop (struct cdmx_port *port)
{
	kthread_stop(port->thread);
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


static inline void rx_process (struct uart_frame *frame,
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

/*******************************************************************************
 * ENTTEC PROTOCOL HANDLERS
 ******************************************************************************/

static void cdmx_enttec_getparams (struct ent_widget *widget)
{
	struct cdmx_port *port = container_of(widget, struct cdmx_port, widget);
	union ent_frame *frame = &widget->reply.dmx;

	frame->som = ENT_SOM;
	frame->label = LABEL_PARAMS;
	frame->size = cpu_to_le16 (ENT_PARAMS_MIN);
	frame->data[ENT_PARAMS_MIN] = ENT_EOM;

	frame->data[0] = ENT_FW_LSB;
	frame->data[1] = ENT_FW_DMX;
	frame->data[2] = (port->breaktime * 1000) / ENT_TIMEUNIT_NS + 1;
	frame->data[3] = (port->mabtime * 1000) / ENT_TIMEUNIT_NS + 1;
	frame->data[4] = port->framerate;
}

static void cdmx_enttec_setparams (struct ent_widget *widget,
		union ent_frame *frame)
{
	struct cdmx_port *port = container_of(widget, struct cdmx_port, widget);
	int a;

//	frame->data[0]
//	frame->data[1]
//	Datasheet says it's "User defined configuration data size, LSB & MSB"
//	No more information how to use it. Ignoring.

	a =(frame->data[2] * ENT_TIMEUNIT_NS ) / 1000;
	NORMALISE_BREAK(a);
	port->breaktime = a;

	a = (frame->data[3] * ENT_TIMEUNIT_NS ) / 1000;
	NORMALISE_MAB(a);
	port->mabtime = a;

	a = frame->data[4];
	NORMALISE_FRAMERATE(a);
	port->framerate = a;

//	K_DEBUG("break %d, mab %d, frame %d", port->breaktime,
//			port->mabtime, port->framerate);
}

static void cdmx_enttec_getserial (struct ent_widget *widget)
{
	struct cdmx_port *port = container_of(widget, struct cdmx_port, widget);
	union ent_frame *frame = &widget->reply.dmx;
	uint8_t serial[ENT_SERIAL_SIZE] = {port->id, 0xFF,0xFF,0xFF};

	frame->som = ENT_SOM;
	frame->label = LABEL_SERIAL;
	frame->size = cpu_to_le16 (ENT_SERIAL_SIZE);
	frame->data[ENT_SERIAL_SIZE] = ENT_EOM;

	memcpy(&frame->data, serial, ENT_SERIAL_SIZE);
}

static void cdmx_enttec_getvendor (struct ent_widget *widget)
{
	union ent_frame *frame = &widget->reply.dmx;
	unsigned int size = sizeof(USBPRO_VENDOR);

	frame->som = ENT_SOM;
	frame->label = LABEL_VENDOR;
	frame->size = cpu_to_le16 (size);
	frame->data[size] = ENT_EOM;

	memcpy(frame->data, USBPRO_VENDOR, size);
}

static void cdmx_enttec_getname (struct ent_widget *widget)
{
	union ent_frame *frame = &widget->reply.dmx;
	unsigned int size = sizeof(USBPRO_NAME);

	frame->som = ENT_SOM;
	frame->label = LABEL_NAME;
	frame->size = cpu_to_le16 (size);
	frame->data[size] = ENT_EOM;

	memcpy(&frame->data, USBPRO_NAME, size);
}

static void cdmx_enntec_receive (struct ent_widget *widget,
		union ent_frame *frame)
{
	struct cdmx_port *port = container_of(widget, struct cdmx_port, widget);

//	K_DEBUG("label %d, size %d", frame->label, __le16_to_cpu(frame->size));
	if (ring_push(&port->readfrom, frame->raw, frame_rawsize(frame)))
	{
		//TODO: Handle ENT_RX_OVERFLOW flag
//		K_DEBUG("no space in ringbuffer, dropping frame");
		return;
	}
	wake_up(&port->wait);
}

void cdmx_enntec_tx (struct ent_widget *widget,
		union ent_frame * frame, int universe)
{
	K_DEBUG("TX not implemented yet");
}

struct ent_ops cdmx_ent_ops =
{
	.set_params = cdmx_enttec_setparams,
	.params = cdmx_enttec_getparams,
	.serial = cdmx_enttec_getserial,
	.vendor = cdmx_enttec_getvendor,
	.name 	= cdmx_enttec_getname,
	.tx		= cdmx_enntec_tx,
	.recv 	= cdmx_enntec_receive
};

/*******************************************************************************
 * LINE DISCIPLINE
 ******************************************************************************/

static int cld_open(struct tty_struct *tty)
{
	int i;
	struct ktermios kt;

	mutex_lock(&cld_lock);
	for (i=0; i< cdmx_port_count; i++)
	{
		if (cdmx_ports[i]->tty == NULL)
		{
			cdmx_ports[i]->tty = tty_kref_get(tty);
			if (cdmx_ports[i]->tty)
			{
				K_INFO("attaching %s to port %d", tty->name, i);
				tty->disc_data = cdmx_ports[i];
				tty->receive_room = CDMX_RECEIVE_ROOM;

				kt = tty->termios;
				kt.c_iflag &= ~(IGNBRK|IGNPAR|PARMRK|ISTRIP|INLCR|\
						IGNCR|ICRNL|IUCLC|IXON|IXANY|IXOFF|IMAXBEL|IUTF8);
				kt.c_iflag |= (BRKINT|INPCK);
				kt.c_oflag &= ~(OPOST);
				kt.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
				kt.c_cflag &= ~(CSIZE|PARENB|CBAUD|CRTSCTS);
				kt.c_cflag |= (CS8|BOTHER|CSTOPB|CREAD|CLOCAL);
				kt.c_ospeed	= DMX_BAUDRATE;
				kt.c_ispeed = kt.c_ospeed;

				if (tty_set_termios(tty, &kt))
				{
					K_ERR("failed to setup UART: %s", tty->name);
					cdmx_ports[i]->tty = NULL;
					tty_kref_put(tty);
					mutex_unlock(&cld_lock);
					return -EINVAL;
				}
				tty_driver_flush_buffer(tty);
				mutex_unlock(&cld_lock);
				return 0;
			}
		}
	}
	K_INFO("no place to attach %s", tty->name);
	mutex_unlock(&cld_lock);
	return -EINVAL;
}

static void cld_close(struct tty_struct *tty)
{
	struct cdmx_port *port = tty->disc_data;
	if (port)
	{
		mutex_lock(&cld_lock);

		tty_driver_flush_buffer(port->tty);
		tty_kref_put(port->tty);
		port->tty = NULL;
		tty->disc_data = NULL;

		mutex_unlock(&cld_lock);
		K_DEBUG("detached %s", tty->name);
	}
}

static ssize_t cld_read(struct tty_struct *tty, struct file *file,
				  unsigned char *buf, size_t nr,
				  void **cookie, unsigned long offset)
{
	return -EINVAL;
}

static ssize_t cld_write(struct tty_struct *tty, struct file *file,
				   const unsigned char *buf, size_t nr)
{
	return -EINVAL;
}

int	cld_receive_buf2(struct tty_struct *tty, const unsigned char *cp,
		char *fp, int count)
{
	struct cdmx_port *port = tty->disc_data;
	struct uart_frame *frame = &port->rx;
	int i;
	uint8_t flag;

	for (i = 0; i<count; i++)
	{
		flag = fp ? fp[i] : TTY_NORMAL;
		rx_process(frame, cp[i], flag);
	}
	return count;
}

void cld_set_termios (struct tty_struct *tty, struct ktermios *old)
{
//	K_DEBUG("cflags: %X, ispeed: %d", old->c_cflag, old->c_ispeed);
}

int	cld_ioctl(struct tty_struct *tty, struct file *file,
		 unsigned int cmd, unsigned long arg)
{
//	K_DEBUG("tty: %s, cmd: 0x%04X, arg: 0x%lx", tty->name, cmd, arg);
	return n_tty_ioctl_helper(tty, file, cmd, arg);
}

static struct tty_ldisc_ops cld_ops =
{
	.owner			= THIS_MODULE,
	.magic			= TTY_LDISC_MAGIC,
	.name			= "cdmx",
	.open			= cld_open,
	.close			= cld_close,
	.read			= cld_read,
	.write			= cld_write,
	.receive_buf2	= cld_receive_buf2,
	.set_termios	= cld_set_termios,
	.ioctl			= cld_ioctl,
};

/*******************************************************************************
 * SYSFS ATTRIBUTES
 ******************************************************************************/

static ssize_t port_attr_show(struct kobject *kobj,
			     struct attribute *attr,
			     char *buf)
{
	struct port_attribute *attribute;
	struct cdmx_port *port;

	attribute = container_of (attr, struct port_attribute, attr);
	port = container_of(kobj, struct cdmx_port, kobj);

	if (!attribute->show)
		return -EIO;

	return attribute->show(port, attribute, buf);
}

static ssize_t port_attr_store(struct kobject *kobj,
			      struct attribute *attr,
			      const char *buf, size_t len)
{
	struct port_attribute *attribute;
	struct cdmx_port *port;

	attribute = container_of (attr, struct port_attribute, attr);
	port = container_of(kobj, struct cdmx_port, kobj);

	if (!attribute->store)
		return -EIO;

	return attribute->store(port, attribute, buf, len);

}

static const struct sysfs_ops port_sysfs_ops = {
	.show = port_attr_show,
	.store = port_attr_store,
};

static void port_release(struct kobject *kobj)
{
	struct cdmx_port *port;
	port = container_of(kobj, struct cdmx_port, kobj);
	kfree(port);
}

static ssize_t port_show(struct cdmx_port *port,
		struct port_attribute *attr,
		char *buf)
{
	int var = 0;

	if (strcmp(attr->attr.name, "breaktime") == 0)
		var = port->breaktime;
	else if (strcmp(attr->attr.name, "mabtime") == 0)
		var = port->mabtime;
	else if (strcmp(attr->attr.name, "framerate") == 0)
		var = port->framerate;

	return sprintf(buf, "%d\n", var);
}

static ssize_t port_store(struct cdmx_port *port,
		struct port_attribute *attr,
		const char *buf,
		size_t count)
{
	int ret, var;

	ret = kstrtoint(buf, 10, &var);
	K_INFO("port %d, '%s'=%d", port->id, attr->attr.name, var);
	if (ret < 0)
		return ret;

	if (strcmp(attr->attr.name, "breaktime") == 0)
		port->breaktime = NORMALISE_BREAK(var);
	else if (strcmp(attr->attr.name, "mabtime") == 0)
		port->mabtime = NORMALISE_MAB(var);
	else if (strcmp(attr->attr.name, "framerate") == 0)
		port->framerate = NORMALISE_FRAMERATE(var);

	//TODO: need semaphore
	return count;
}


static struct port_attribute attr_breaktime =
	__ATTR(breaktime, 0664, port_show, port_store);

static struct port_attribute attr_mabtime =
	__ATTR(mabtime, 0664, port_show, port_store);

static struct port_attribute attr_framerate =
	__ATTR(framerate, 0664, port_show, port_store);

static struct attribute *port_attrs[] = {
	&attr_breaktime.attr,
	&attr_mabtime.attr,
	&attr_framerate.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};
ATTRIBUTE_GROUPS(port);

/* ATTRIBUTE_GROUPS(port) equivalent to:
static const struct attribute_group port_group =
{
	.attrs = port_attrs,
};
static const struct attribute_group *port_groups[] =
{
	&port_group,
	NULL
};
*/

static struct kobj_type port_ktype =
{
	.sysfs_ops = &port_sysfs_ops,
	.release = port_release,
	.default_groups = port_groups,
};

/*******************************************************************************
 * CHARACTER DEVICE
 ******************************************************************************/

static char *cdmx_devnode(struct device *dev, umode_t *mode)
{
	if (!mode)
		return NULL;
	if ( MAJOR(dev->devt) == MAJOR(cdmx_device_id))
		*mode = CHMOD_RW;
	return NULL;
}

static int cdmx_open 	(struct inode *node, struct file *f)
{
	struct cdmx_port *port;
	port = container_of(node->i_cdev, struct cdmx_port, cdev);
//	K_DEBUG("file %s, port %p dev_t %X",
//			f->f_path.dentry->d_name.name, port, port->cdev.dev);

	f->private_data = port;
	try_module_get(THIS_MODULE);

//	K_DEBUG( "->>> done");
	return 0;
}

static int cdmx_release (struct inode *node, struct file *f)
{
	struct cdmx_port *port;
	port = (struct cdmx_port *) f->private_data;
//	K_DEBUG("file %s, port %p dev_t %X",
//			f->f_path.dentry->d_name.name, port, port->cdev.dev);

	module_put(THIS_MODULE);
	return 0;
}

static ssize_t cdmx_write (struct file *f, const char* src,
		size_t len, loff_t * offset)
{
	struct cdmx_port *port = f->private_data;
	uint8_t buffer[LOCALBUF_SIZE];
	ssize_t result = 0, size = 0;

//	K_DEBUG("<<<- port %d, max %d bytes", port->id, len);
	if (mutex_lock_interruptible(&port->tx_write_lock))
		return -ERESTARTSYS;
	if (mutex_lock_interruptible(&port->rx_read_lock))
	{
		mutex_unlock(&port->tx_write_lock);
		return -ERESTARTSYS;
	}
	if (!port->fsdev)
	{
		K_ERR( "device #%d doesn't exist", port->id);
		result = -ENODEV;
		goto out;
	}

	while(len)
	{
		size = MIN(len, LOCALBUF_SIZE);
		if (copy_from_user(buffer, &src[result], size))
			break;
		size = ent_write(&port->widget, &src[result], size);
		result += size;
		len -= size;
	}

	out:
	mutex_unlock(&port->rx_read_lock);
	mutex_unlock(&port->tx_write_lock);
//	K_DEBUG("->>> port %d, bytes written %d",
//			 port->id, result);
	return result;

}

static ssize_t cdmx_read 	(struct file *f, char* dest,
		size_t len, loff_t * offset)
{
	struct cdmx_port *port = (struct cdmx_port *) f->private_data;
	uint8_t buffer[LOCALBUF_SIZE];
	ssize_t result = 0, size = 0, missed = 0;

//	K_DEBUG("port %d, max %d bytes to read", port->id, len);
	if (mutex_lock_interruptible(&port->rx_read_lock))
		return -ERESTARTSYS;

	if (!port->fsdev)
	{
		K_ERR( "device #%d doesn't exist", port->id);
		result = -ENODEV;
		goto out;
	}

	while(len && port->readfrom.size)
	{
		size = MIN(len, LOCALBUF_SIZE);
		size = ring_pop(&port->readfrom, buffer, size);
		if (!size)
			break;

		missed = copy_to_user(dest + result, buffer, size);
		result += (size - missed);
		len -= (size - missed);
		if (missed)
		{
			ring_unpop(&port->readfrom, missed);
			break;
		}
	}

	out:
	mutex_unlock(&port->rx_read_lock);

//	K_DEBUG("->>> total %d bytes", result);
	return result;
}

__poll_t cdmx_poll (struct file *f, struct poll_table_struct *p)
{
	__poll_t mask = 0;
	struct cdmx_port *port = (struct cdmx_port *) f->private_data;
	size_t w=0;

	mutex_lock(&port->tx_write_lock);
	mutex_lock(&port->rx_read_lock);

	poll_wait(f, &port->wait, p);
	if (port->readfrom.size)
		mask |= (EPOLLIN | EPOLLRDNORM | EPOLLPRI );

	//TODO: handle out
	(void) w;
//	w = cdmx_space_left(port);
//	if( w > 0)
//		mask |= EPOLLOUT | EPOLLWRNORM;
	mutex_unlock(&port->rx_read_lock);
	mutex_unlock(&port->tx_write_lock);
//	K_DEBUG("port %d, to read: %d, to write: %d",
//	port->id, port->readfrom.size, w);

	return mask;
}

long cdmx_ioctl (struct file *f, unsigned int cmd, unsigned long arg)
{
	struct cdmx_port *port = (struct cdmx_port *) f->private_data;
	void __user *p = (void __user *)arg;
	int i;

//	K_DEBUG("cmd 0x%X, param 0x%lX", cmd, arg);

	switch(cmd)
	{
	case TIOCEXCL:
		set_bit(CDMX_EXCLUSIVE, &port->flags);
//		K_DEBUG("set exclusive");
		return 0;
	case TIOCNXCL:
//		K_DEBUG("cleared exclusive");
		clear_bit(CDMX_EXCLUSIVE, &port->flags);
		return 0;
	case 0x5440: //TIOCGEXCL:
		i =  test_bit(CDMX_EXCLUSIVE, &port->flags);
//		K_DEBUG("is exclusive: %d", i);
		return put_user(i, (int __user *)p);

	case TIOCINQ:
		i = port->readfrom.size;
//		K_DEBUG("bytes to read: %d", i);
		return put_user(i, (int __user *)p);
/*	case TIOCOUTQ:
		i = port->write_to.rawsize;
		K_DEBUG("bytes queued: %d", i);
		return put_user(i, (int __user *)p);
*/
	case TCGETS:
//		if (port->tty)
//			return cld_ioctl(port->tty, f, cmd, arg);
//		K_DEBUG("TCGETS, ignoring");
		return 0;
	case TCSETS:
//		K_DEBUG("TCSETS, ignoring");
		return 0;
	default:
		return -EINVAL;
	}
}

static struct file_operations cdmx_ops =
{
	.owner		= THIS_MODULE,
	.read		= cdmx_read,
	.write		= cdmx_write,
	.open		= cdmx_open,
	.release 	= cdmx_release,
	.poll		= cdmx_poll,
	.unlocked_ioctl = cdmx_ioctl,
};

/*******************************************************************************
 * MODULE CORE
 ******************************************************************************/

static struct cdmx_port *cdmx_create_port_obj(int id)
{
	struct cdmx_port *port;
	int retval;
	char template[] = "port%03X";

	if (id < 0)
	{
		K_ERR("port ID=%d, can't be negative", id);
		return NULL;
	}
	else if (id > CDMX_PORTS_MAX)
	{
		K_ERR("port ID=%d greater than maximum %d", id, CDMX_PORTS_MAX);
		return NULL;
	}

	port = kzalloc(sizeof(*port), GFP_KERNEL);
	if (!port)
		return NULL;

	mutex_init(&port->tx_write_lock);
	mutex_init(&port->rx_read_lock);
	init_waitqueue_head(&port->wait);
	port->breaktime = DEFAULT_BREAK;
	port->mabtime 	= DEFAULT_MAB;
	port->framerate = DEFAULT_FRAMERATE;
	port->id = PORT_INACTIVE;
	port->widget.ops = &cdmx_ent_ops;

	/*
	 * As we have a kset for this kobject, we need to set it before calling
	 * the kobject core.
	 */
	port->kobj.kset = cdmx_ports_kset;

	/*
	 * Initialize and add the kobject to the kernel.  All the default files
	 * will be created here.  As we have already specified a kset for this
	 * kobject, we don't have to set a parent for the kobject, the kobject
	 * will be placed beneath that kset automatically.
	 */
	retval = kobject_init_and_add(&port->kobj, &port_ktype, NULL, template, id);
	if (retval)
	{
		kobject_put(&port->kobj);
		return NULL;
	}

	// mark port as active
	port->id = id;

	/*
	 * We are always responsible for sending the uevent that the kobject
	 * was added to the system.
	 */
	kobject_uevent(&port->kobj, KOBJ_ADD);

	return port;
}

static void cdmx_destroy_port_obj(struct cdmx_port *port)
{
	port->id = PORT_INACTIVE;
	kobject_put(&port->kobj);
}


static int cdmx_create_cdevs (void)
{
	int i, c, err;
	const char 	*chrdev_name 	= "cdmx";

	// Returns zero or a negative error code.
	err = alloc_chrdev_region(&cdmx_device_id,
			CDMX_BASE_MINOR, cdmx_port_count, chrdev_name);
	if (err)
	{
		K_ERR("failed to allocate chrdev region for '%s'", chrdev_name);
		err = -ENOENT;
		goto failure1;
	}

	// Returns &struct class pointer on success, or ERR_PTR() on error.
	cdmx_devclass = class_create(THIS_MODULE, chrdev_name);
	if (IS_ERR(cdmx_devclass))
	{
		K_ERR("failed to create class '%s'", chrdev_name);
		err = -ENOENT;
		goto failure2;
	}
	// handles chrdev CHMOD
	cdmx_devclass->devnode = cdmx_devnode;


	for (i=0; i< cdmx_port_count; i++)
	{
		cdev_init( &cdmx_ports[i]->cdev, &cdmx_ops);
		cdmx_ports[i]->cdev.owner = THIS_MODULE;
		//  A negative error code is returned on failure.
		err = cdev_add (&cdmx_ports[i]->cdev,
				MKDEV(MAJOR(cdmx_device_id), (i+CDMX_BASE_MINOR)), 1);

		if (err < 0)
		{
			K_ERR("cdev_add failed");
			for (c = i - 1; c >= 0; c--)
			{
				cdev_del(&cdmx_ports[c]->cdev);
			}
			err = -ENOENT;
			goto failure3;
		}
	}

	for (i=0; i< cdmx_port_count; i++)
	{
		// Returns &struct device pointer on success, or ERR_PTR() on error.
		cdmx_ports[i]->fsdev = \
			device_create(cdmx_devclass, NULL, \
			MKDEV(MAJOR(cdmx_device_id), (i + CDMX_BASE_MINOR)),
			NULL, "cdmx%03X", (i + CDMX_BASE_MINOR));

		if (IS_ERR(cdmx_ports[i]->fsdev))
		{
			K_ERR("failed create device");
			for (c = i - 1; c >= 0; c--)
				device_destroy(cdmx_devclass,
						MKDEV(MAJOR(cdmx_device_id), (c + CDMX_BASE_MINOR)));
			err = -ENOENT;
			goto failure4;
		}

	}

//	K_DEBUG("->>> done");
	return 0;

failure4:
	for (i=0; i< cdmx_port_count; i++)
		cdev_del(&cdmx_ports[i]->cdev);
failure3:
	class_destroy(cdmx_devclass);
failure2:
	unregister_chrdev_region(MKDEV(MAJOR(cdmx_device_id), CDMX_BASE_MINOR),
			cdmx_port_count);
failure1:
	K_ERR( "->>> cleanup done");
	return err;
}

static void cdmx_remove_cdevs (void)
{
	int i;
	for (i=0; i< cdmx_port_count; i++)
	{
		device_destroy(cdmx_devclass,
				MKDEV(MAJOR(cdmx_device_id), (i + CDMX_BASE_MINOR)));
		cdev_del(&cdmx_ports[i]->cdev);
	}
	class_destroy(cdmx_devclass);
	unregister_chrdev_region(MKDEV(MAJOR(cdmx_device_id), CDMX_BASE_MINOR),
			cdmx_port_count);
}



static int __init cdmx_init (void)
{
	int i, c, p, err;

	mutex_init(&cld_lock);
	p = TO_RANGE(cdmx_port_count, CDMX_PORTS_MIN, CDMX_PORTS_MAX);
	if (p != cdmx_port_count)
	{
		K_INFO("port count %d is out of range(%d-%d)",
				cdmx_port_count, CDMX_PORTS_MIN, CDMX_PORTS_MAX);
		cdmx_port_count = p;
	}
	K_INFO("using port count %d", cdmx_port_count);

	cdmx_ports_kset = kset_create_and_add("cdmx", NULL, NULL);
	if (!cdmx_ports_kset)
	{
		K_ERR("out of memory");
		return -ENOMEM;
	}
	cdmx_ports = kzalloc(cdmx_port_count * sizeof(*cdmx_ports), GFP_KERNEL);
	if (!cdmx_ports)
	{
		K_ERR("out of memory");
		err = -ENOMEM;
		goto failure1;
	}
	for (i=0; i<cdmx_port_count; i++)
	{
		cdmx_ports[i] = cdmx_create_port_obj(i + CDMX_BASE_MINOR);
		if (!cdmx_ports[i])
		{
			for (c = 0; c < i; c++)
				cdmx_destroy_port_obj(cdmx_ports[c]);
			K_ERR("out of memory");
			err = -ENOMEM;
			goto failure2;
		}
	}
	err = cdmx_create_cdevs();
	if (err)
		goto failure2;

	err = tty_register_ldisc(CDMX_LD, &cld_ops);
	if (err)
		goto failure3;

	K_DEBUG("->>> init done");
	return 0;

failure3:
	cdmx_remove_cdevs();
failure2:
	kfree(cdmx_ports);
failure1:
	kset_unregister(cdmx_ports_kset);
	K_ERR("resources freed after failure, exiting");
	return err;
}

static void __exit cdmx_exit(void)
{
	int i;

	tty_unregister_ldisc(CDMX_LD);
	cdmx_remove_cdevs();
	for (i=0; i<cdmx_port_count; i++)
		cdmx_destroy_port_obj(cdmx_ports[i]);
	kfree(cdmx_ports);
	kset_unregister(cdmx_ports_kset);

	K_DEBUG("->>> done");
}

module_init(cdmx_init); // @suppress("Unused function declaration")
module_exit(cdmx_exit); // @suppress("Unused function declaration")

/*******************************************************************************
 ******************************************************************************/
