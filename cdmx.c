
#include "cdmx.h"
#include <uapi/linux/eventpoll.h>
#include <linux/ctype.h>
#include <linux/printk.h>
#include <uapi/asm-generic/errno-base.h>
#include <uapi/asm-generic/ioctls.h>
#include <uapi/asm-generic/fcntl.h>
#include <uapi/asm-generic/termbits.h>
//#include <asm-generic/bitops/atomic.h>
#include <linux/iomap.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
//#include <asm-generic/uaccess.h>
//#include <linux/list.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Neutrino");
MODULE_DESCRIPTION("DMX character device");
MODULE_VERSION("0.01");


int cdmx_port_count = CDMX_PORTS_DEFAULT;
module_param(cdmx_port_count, hexint, CHMOD_RO);


static struct class *cdmx_devclass 	= NULL;
static dev_t 		cdmx_device_id;

static struct dmx_port **dmx_ports;
static struct kset *dmx_ports_kset;
static struct mutex cld_lock;

/*
static inline size_t ring_spaceleft(ringbuffer_t *ring)
{
	ssize_t left = ring->write_ptr - ring->read_ptr;
	return (left < 0) ? left + CDMX_RECEIVE_ROOM : left;
}

static inline size_t ring_spacebusy(ringbuffer_t *ring)
{
	ssize_t left = ring->write_ptr - ring->read_ptr;
	return (left < 0) ? -left : CDMX_RECEIVE_ROOM - left ;
}

static size_t ring_touser(ringbuffer_t *ring, uint8_t *dest, size_t len)
{
	size_t result = 0, size = 0;
	if ( (!len) || (ring->size == 0) )
		return result;

	if (ring->read_ptr > ring->write_ptr)
	{
		size = MIN (len, ring->size);
		if (copy_to_user(dest, &ring->data[ring->read_ptr], size))
			return result;
		ring->read_ptr = (ring->read_ptr + size) % CDMX_RECEIVE_ROOM;
		ring->size -= size;
		result += size;
		len -= size;
	}
	if(len)
	{
		size = MIN (len, ring->size);
		if (copy_to_user(&dest[result], &ring->data[ring->read_ptr], size) )
			return result;
		ring->read_ptr += size;
		ring->size -= size;
		result += size;
	}

	return result;
}
*/
static size_t ring_fromuser(ringbuffer_t *ring, const uint8_t *src, size_t len)
{
	size_t result = 0, size = 0;
	if ( (!len) || ( ring->size == CDMX_RECEIVE_ROOM) )
		return result;

	if (ring->write_ptr > ring->read_ptr)
	{
		size = CDMX_RECEIVE_ROOM - ring->size;
		size = MIN (size, len);
		if (copy_from_user(&ring->data[ring->write_ptr], src, size))
			return result;
		//K_DEBUG("chunk 1, %d bytes", size);
		ring->write_ptr = (ring->write_ptr + size) % CDMX_RECEIVE_ROOM;
		ring->size += size;
		result += size;
		len -= size;
	}
	if(len)
	{
		size = CDMX_RECEIVE_ROOM - ring->size;
		size = MIN (size, len);
		if (copy_from_user(&ring->data[ring->write_ptr], &src[result], size))
			return result;
		//K_DEBUG("chunk 2, %d bytes", size);
		ring->write_ptr = (ring->write_ptr + size) % CDMX_RECEIVE_ROOM;
		ring->size += size;
		result += size;
	}
	//K_DEBUG("total %d bytes", result);
	return result;
}

static int ring_pop(ringbuffer_t *ring, uint8_t *ch)
{
	if (ring->size == 0)
		return -1;
	*ch = ring->data[ring->read_ptr++];
	ring->read_ptr = CDMX_RECEIVE_ROOM;
	ring->size--;
	return 0;
}

static inline void frame_reset(cdmx_frame_t *frame)
{
	frame->state = PRE_SOM;
	frame->size = 0;
	frame->write_ptr = 0;
	frame->read_ptr = 0;
	frame->pending = false;
}

static int frame_push(cdmx_frame_t *frame, uint8_t ch)
{
	if (frame->size >= ENT_FRAME_MAX)
		return -1;

	frame->ent.raw[frame->write_ptr++] = ch;
	frame->size++;
	return 0;
}

static size_t frame_read(cdmx_frame_t *frame, uint8_t *dest, size_t len)
{
	size_t result = 0, size = 0;

	size = frame->size - frame->read_ptr;
	size = MIN (size, len);
	if (copy_to_user(dest, &frame->ent.raw[frame->read_ptr], size))
		return result;
	frame->read_ptr += size;
	if (frame->read_ptr == frame->size)
		frame_reset(frame);
	return result;
}

static size_t framering_read(framering_t *ring, uint8_t *dest, size_t len)
{
	size_t result = 0;
	cdmx_frame_t *frame;
	if ( (!len) || ( ring->size == 0) )
		return result;

	frame = &ring->data[ring->read_ptr];
	if (!frame->pending)
		return result;

	result = frame_read(frame, dest, len);
	if (!frame->pending)
	{
		ring->read_ptr = (ring->read_ptr + 1) % CDMX_BUFFERING;
		ring->size--;
	}
	return result;
}

static int framering_enqueue(framering_t *ring, const uint8_t *src, size_t len)
{
	cdmx_frame_t *frame;
	if (ring->size == CDMX_BUFFERING)
		return -ENOSPC;

	frame = &ring->data[ring->write_ptr];
	memcpy(&frame->ent.raw, src, len);
	ring->write_ptr = (ring->write_ptr + 1) % CDMX_BUFFERING;
	ring->size++;

	return 0;
}

static __always_inline
uint16_t ent_expected_size(cdmx_frame_t *frame)
{
	return (frame->ent.sz_msb << 8 ) | frame->ent.sz_lsb;
}
static __always_inline
int ent_frame_full(cdmx_frame_t *frame)
{
	return (frame->size >= ent_expected_size(frame) - ENT_HEADER_SIZE);
}
static __always_inline
void ent_terminate_frame(cdmx_frame_t *frame)
{
	frame_push(frame, ENT_EOM);
}



static inline void ent_mkframe_header(cdmx_frame_t *frame, uint8_t label, uint16_t size)
{
	frame_reset(frame);
	frame_push(frame, ENT_SOM);
	frame_push(frame, label);
	frame_push(frame, size & 0x00FF);
	frame_push(frame, size >> 8);
}

static void cdmx_enttec_getparams (struct dmx_port *port)
{
	cdmx_frame_t *frame = &port->frame_rd;

	ent_mkframe_header(frame, LABEL_GET_PARAMS, ENT_PARAMS_MIN);

	frame_push(frame, ENT_FW_LSB);
	frame_push(frame, ENT_FW_DMX);
	frame_push(frame, (port->breaktime * 1000) / ENT_TIMEUNIT_NS + 1);
	frame_push(frame, (port->mabtime * 1000) / ENT_TIMEUNIT_NS + 1);
	frame_push(frame, port->framerate);

	ent_terminate_frame(frame);
}
/*
static void cdmx_enttec_setparams (struct dmx_port *port)
{
	struct dmx_frame_a *frame = &port->read_from;
	int a;

//	uint16_t user_data_size;
//	user_data_size = (frame->data[1] << 8) & frame->data[0];
//	Datasheet says it's "User defined configuration data size, LSB & MSB"
//	No more information if it ever useful or not. Ignoring.

	a =(frame->data[2] * ENT_TIMEUNIT_NS ) / 1000;
	NORMALISE_BREAK(a);
	port->breaktime = a;

	a = (frame->data[3] * ENT_TIMEUNIT_NS ) / 1000;
	NORMALISE_MAB(a);
	port->mabtime = a;

	a = frame->data[4];
	NORMALISE_FRAMERATE(a);
	port->framerate = a;

	K_DEBUG("break %d, mab %d, frame %d", port->breaktime, port->mabtime, port->framerate);
}

static void cdmx_enttec_getserial (struct dmx_port *port)
{
	struct dmx_frame_a *frame = &port->read_from;
	uint8_t serial[ENT_SERIAL_SIZE] = {port->id, 0,0,0};

	memcpy(&frame->data[ENT_HEADER_SIZE], serial, ENT_SERIAL_SIZE);
	cdmx_enttec_mkframe(frame, LABEL_GET_SERIAL, ENT_SERIAL_SIZE);

	K_DEBUG("message %d bytes", frame->msgsize);
}


static void cdmx_enttec_getvendor (struct dmx_port *port)
{
	struct dmx_frame_a *frame = &port->read_from;
	unsigned int size = sizeof(USBPRO_VENDOR);

	// supply vendor name and ESTA codes
	memcpy(&frame->data[ENT_HEADER_SIZE], USBPRO_VENDOR, size);
	cdmx_enttec_mkframe(frame, LABEL_VENDOR, size);

	K_DEBUG("message %d bytes", frame->msgsize);
}

static void cdmx_enttec_getname (struct dmx_port *port)
{
	struct dmx_frame_a *frame = &port->read_from;
	unsigned int size = sizeof(USBPRO_NAME);

	// supply device name and ESTA codes
	memcpy(&frame->data[ENT_HEADER_SIZE], USBPRO_NAME, size);
	cdmx_enttec_mkframe(frame, LABEL_NAME, size);

	K_DEBUG("message %d bytes", frame->msgsize);
}

static void cdmx_enttec_flash (struct dmx_port *port)
{
	struct dmx_frame_a *frame = &port->read_from;

	// report failed firmware flashing
	memcpy(&frame->data[ENT_HEADER_SIZE], ENT_FLASH_FALSE, ENT_FLASH_REPLY);
	cdmx_enttec_mkframe(frame, LABEL_NAME, ENT_FLASH_REPLY);

	K_DEBUG("message %d bytes", frame->msgsize);
}
*/

/* static void cdmx_enttec_onchange (struct dmx_port *port)
 *
 * This message also reinitializes the DMX receive processing,
 * so that if change of state reception is selected,
 * the initial received DMX data is cleared to all zeros.
 * */

/* static void cdmx_enttec_update (struct dmx_port *port)
 *
 * Datasheet on this label looks like a crap.
 * It describes 1 byte as frame offset for 40-slot subframe.
 * So, 256 + 40 = insuffitient to address 512-slots frame.
 * Neither forums nor other source found to make it clear,
 * so decision is made to leave ONCHANGE & UPDATE labes unsupported.
 * */


static void ent_handle_msg(struct dmx_port *port)
{
	switch (port->frame_wr.ent.label)
	{
		// labels that generate replies
		case LABEL_GET_PARAMS:
			cdmx_enttec_getparams(port);
			break;
		case LABEL_GET_SERIAL:
			//cdmx_enttec_getserial(port);
			break;
		case LABEL_VENDOR:
			//cdmx_enttec_getvendor(port);
			break;
		case LABEL_NAME:
			//cdmx_enttec_getname(port);
			break;
		case LABEL_FLASH_PAGE:
			//cdmx_enttec_flash(port);
			break;

		// labels that don't need replies
		case LABEL_SET_PARAMS:
			//cdmx_enttec_setparams(port);
			break;

		// labels that bring DMX/RDM data to TX
		case LABEL_RDM_DISCOVERY:
			break;
		case LABEL_RDM_OUTPUT:
			break;
		case LABEL_DMX_OUTPUT:
			break;
		case LABEL_UNIVERSE_0:
			break;
		case LABEL_UNIVERSE_1:
			break;

		// unsupported labels
		case LABEL_RECEIVED_DMX:	// CAN'T BE INCOMING
			break;					// brings data from RX to host
		case LABEL_FLASH_FW:		// IGNORED
			break;					// prepare for f/w flashing
		case LABEL_ONCHANGE: 		// UNSUPPORTED
			break;					// due lack of documentation
		case LABEL_DATA_UPDATE: 	// UNSUPPORTED
			break;					// due lack of documentation
		default:
		/*	K_INFO("port %d: unhandled label %d, raw data %d bytes",
					port->id, port->write_to.msglabel, port->write_to.whead);
		 */ // There exist some more vendor-specific labels, just ignoring
		break;
	}
	//TODO: implement TX
	K_DEBUG("->>> TX not implemented yet");
}


static void ent_parse(struct dmx_port *port)
{
	ringbuffer_t *ring = &port->ring_wr;
	cdmx_frame_t *frame = &port->frame_wr;
	uint8_t byte;
	uint16_t size;

	while (ring_pop(ring, &byte) == 0)
	{
		switch (frame->state)
		{
		case PRE_SOM:
			if (byte == ENT_SOM)
			{
				frame_push(frame, byte);
				frame->state = GOT_SOM;
			}
		break;
		case GOT_SOM:
			frame_push(frame, byte);
			frame->state = GOT_LABEL;
		break;
		case GOT_LABEL:
			frame_push(frame, byte);
			frame->state = GOT_SIZE_LSB;
		break;
		case GOT_SIZE_LSB:
			frame_push(frame, byte);
			size = ent_expected_size(frame);
			if (size == 0)
				frame->state = WAITING_FOR_EOM;
			else if (size > DMX_FRAME_MAX)
				{
					K_INFO("dropping broken usb frame, size '%d' \
							is greater than possible", size);
					frame->state = PRE_SOM;
				}
			else
				frame->state = IN_DATA;
		break;
		case IN_DATA:
			frame_push(frame, byte);

			if (ent_frame_full(frame))
			{
				frame->state = WAITING_FOR_EOM;
				ent_handle_msg(port);
			}
		break;
		case WAITING_FOR_EOM:
			if (byte == ENT_EOM)
				{
				frame->state = PRE_SOM;
				ent_handle_msg(port);
				}
		break;
		}
	}
}

static void cdmx_rx_enqueue(struct dmx_port *port)
{
	/*
	struct uart_frame_a *pending;

	mutex_lock(&port->rx_read_lock);
	pending = port->rx_pending;
	if(!pending)
	{
		port->rx_pending = &port->rx[port->rx_current++];
		port->rx_current %= CDMX_BUFFERING;
	}
	else
	{
		while (pending->next)
			pending = pending->next;

		pending->next = &port->rx[port->rx_current++];
		port->rx_current %= CDMX_BUFFERING;
	}
	mutex_unlock(&port->rx_read_lock);
	*/
}

static int cld_open(struct tty_struct *tty)
{
	int i;
	struct ktermios kt;

	mutex_lock(&cld_lock);
	for (i=0; i< cdmx_port_count; i++)
	{
		if (dmx_ports[i]->tty == NULL)
		{
			dmx_ports[i]->tty = tty_kref_get(tty);
			if (dmx_ports[i]->tty)
			{
				K_INFO("attaching %s to port %d", tty->name, i);
				tty->disc_data = dmx_ports[i];
				tty->receive_room = CDMX_RECEIVE_ROOM;

				kt = tty->termios;
				kt.c_iflag &= ~(IGNBRK|IGNPAR|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IUCLC|IXON|IXANY|IXOFF|IMAXBEL|IUTF8);
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
					dmx_ports[i]->tty = NULL;
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
	struct dmx_port *port = tty->disc_data;
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
	K_DEBUG("->>>");
	return -EINVAL;
}

static ssize_t cld_write(struct tty_struct *tty, struct file *file,
				   const unsigned char *buf, size_t nr)
{
	K_DEBUG("->>>");
	return -EINVAL;
}

int	cld_receive_buf2(struct tty_struct *tty, const unsigned char *cp,
			char *fp, int count)
{
	struct dmx_port *port = tty->disc_data;
	struct uart_frame_a *frame = &port->rx[port->rx_current];
	int i;
	uint8_t flag;

	count = MIN(count, tty->receive_room);
	for (i=0; i<count; i++)
	{
		flag = fp ? fp[i] : TTY_NORMAL;

		if (flag == TTY_NORMAL)
		{
			switch(frame->state)
			{
			case BREAK:
				frame->state = DATA;
				frame->data[frame->size++] = cp[i];
				break;

			case DATA:
				frame->data[frame->size] = cp[i];
				frame->size++;
				if (frame->size >= DMX_FRAME_MAX)
				{
					frame->state = OVER;
					cdmx_rx_enqueue(port);
					//TODO: handle frame
				}
				break;
			default:
				break;
			}
		}
		else if (flag == TTY_BREAK)
		{
			if(frame->state == DATA)
			{
				cdmx_rx_enqueue(port);
			}
			frame->state = BREAK;
		}
		else
		{
			frame->state = FAULT;
		}
	}
	return count;
}

void cld_set_termios (struct tty_struct *tty, struct ktermios *old)
{
	K_DEBUG("cflags: %X, ispeed: %d", old->c_cflag, old->c_ispeed);
}
int	cld_ioctl(struct tty_struct *tty, struct file *file,
		 unsigned int cmd, unsigned long arg)
{
	K_DEBUG("tty: %s, cmd: 0x%04X, arg: 0x%lx", tty->name, cmd, arg);
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
//	.receive_buf	= cld_receive_buf,
	.receive_buf2	= cld_receive_buf2,
	.set_termios	= cld_set_termios,
	.ioctl			= cld_ioctl,
};


static ssize_t port_attr_show(struct kobject *kobj,
			     struct attribute *attr,
			     char *buf)
{
	struct port_attribute *attribute;
	struct dmx_port *port;

	attribute = container_of (attr, struct port_attribute, attr);
	port = container_of(kobj, struct dmx_port, kobj);

	if (!attribute->show)
		return -EIO;

	return attribute->show(port, attribute, buf);
}

static ssize_t port_attr_store(struct kobject *kobj,
			      struct attribute *attr,
			      const char *buf, size_t len)
{
	struct port_attribute *attribute;
	struct dmx_port *port;

	attribute = container_of (attr, struct port_attribute, attr);
	port = container_of(kobj, struct dmx_port, kobj);

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
	struct dmx_port *port;
	port = container_of(kobj, struct dmx_port, kobj);
	kfree(port);
	K_DEBUG("->>> done");
}

static ssize_t port_show(struct dmx_port *port,
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

static ssize_t port_store(struct dmx_port *port,
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

static ssize_t port_str_show(struct dmx_port *port,
		struct port_attribute *attr,
		char *buf)
{
	if (!port->ttyname)
		return 0;
	return scnprintf(buf, PAGE_SIZE, "%s", port->ttyname); // @suppress("Symbol is not resolved")
}

static ssize_t port_str_store(struct dmx_port *port,
		struct port_attribute *attr,
		const char *buf,
		size_t count)
{
	ssize_t size;
	const char *start, *end;
	char *newname;

	K_DEBUG ("start, port=%d, count=%d", port->id, count);
	if (!count)
		return count;

	newname = kzalloc(count, GFP_KERNEL);
	if (!newname)
	{
		K_ERR("out of memory");
		return -ENOMEM;
	}

	size = count;
	start = buf;
	end = buf + size - 1;

	while (end >= buf && isspace(*end))
		{	end--; size--;	}
	if (size < 1) return count;
	while (isspace(*start))
		{ 	start++; size--; 	}
	if (size < 1) return count;

	memcpy(newname, start, size);
	newname[size] = '\0';

	if (port->ttyname)
	{
		if (strcmp(port->ttyname, newname))
		{
			K_INFO("port %d re-attaching TTY from '%s' to '%s'", \
					port->id, port->ttyname, newname);

			//TODO: close previous TTY here
			kfree(port->ttyname);
			//TODO: open new TTY here
			port->ttyname = newname;
		}
		else
		{
			K_DEBUG("port %d: new TTY name is the same, nothing to do", \
					port->id);
			kfree(newname);
		}
	}
	else
	{
		K_INFO("port %d attaching to TTY '%s'", port->id, newname);
		//TODO: open new TTY here
		port->ttyname = newname;
	}

	return count;
}

static struct port_attribute attr_breaktime =
	__ATTR(breaktime, 0664, port_show, port_store);

static struct port_attribute attr_mabtime =
	__ATTR(mabtime, 0664, port_show, port_store);

static struct port_attribute attr_framerate =
	__ATTR(framerate, 0664, port_show, port_store);

static struct port_attribute attr_ttyname =
	__ATTR(ttyname, 0664, port_str_show, port_str_store);


static struct attribute *port_attrs[] = {
	&attr_breaktime.attr,
	&attr_mabtime.attr,
	&attr_framerate.attr,
	&attr_ttyname.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};
ATTRIBUTE_GROUPS(port);
/*
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
	struct dmx_port *port;
	port = container_of(node->i_cdev, struct dmx_port, cdev);
	K_DEBUG("file %s, port %p dev_t %X",
			f->f_path.dentry->d_name.name, port, port->cdev.dev);

	f->private_data = port;
	try_module_get(THIS_MODULE);

	K_DEBUG( "->>> done");
	return 0;
}

static int cdmx_release (struct inode *node, struct file *f)
{
	struct dmx_port *port;
	port = (struct dmx_port *) f->private_data;
	K_DEBUG("file %s, port %p dev_t %X",
			f->f_path.dentry->d_name.name, port, port->cdev.dev);

	module_put(THIS_MODULE);
	return 0;
}




static inline size_t cdmx_space_left(struct dmx_port *port)
{
	return (sizeof(port->write_to.rawdata) - port->write_to.rawsize );
}

static int cdmx_bytes_to_read(struct dmx_port *port)
{
	struct dmx_frame_a *frame = &port->read_from;
	if (!frame->pending)
		return 0;
	return frame->msgsize - frame->rhead;
}

static ssize_t cdmx_write (struct file *f, const char* src,
		size_t len, loff_t * offset)
{
	struct dmx_port *port = f->private_data;
	ssize_t result;

	K_DEBUG("<<<- port %d, %d bytes", port->id, len);
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

	result = ring_fromuser(&port->ring_wr, src, len);
	ent_parse(port);

	out:
	mutex_unlock(&port->rx_read_lock);
	mutex_unlock(&port->tx_write_lock);
	K_DEBUG("->>> port %d, bytes written %d", \
			 port->id, result);
	return result;

}


static ssize_t cdmx_read 	(struct file *f, char* dest,
		size_t len, loff_t * offset)
{
	struct dmx_port *port = (struct dmx_port *) f->private_data;
	ssize_t result = 0;

	K_DEBUG("port %d, max %d bytes to read", port->id, len);
	if (mutex_lock_interruptible(&port->rx_read_lock))
		return -ERESTARTSYS;

	if (!port->fsdev)
	{
		K_ERR( "device #%d doesn't exist", port->id);
		result = -ENODEV;
		goto out;
	}

	result = framering_read(&port->frames_rd, dest, len);

	out:
	mutex_unlock(&port->rx_read_lock);

	return result;
}

__poll_t cdmx_poll (struct file *f, struct poll_table_struct *p)
{
	struct dmx_port *port = (struct dmx_port *) f->private_data;
	__poll_t mask = 0;
	int r, w;

	mutex_lock(&port->tx_write_lock);
	mutex_lock(&port->rx_read_lock);

	poll_wait(f, &port->wait, p);
	/*
	r = cdmx_bytes_to_read(port);
	if (r > 0)
		mask |= (EPOLLIN | EPOLLRDNORM | EPOLLPRI );

	w = cdmx_space_left(port);
	if( w > 0)
		mask |= EPOLLOUT | EPOLLWRNORM;
*/
	mutex_unlock(&port->rx_read_lock);
	mutex_unlock(&port->tx_write_lock);
	K_DEBUG("port %d, to read: %d, to write: %d", port->id, r, w);
	return mask;
}

long cdmx_ioctl (struct file *f, unsigned int cmd, unsigned long arg)
{
	struct dmx_port *port = (struct dmx_port *) f->private_data;
	void __user *p = (void __user *)arg;
	int i;

	K_DEBUG("cmd 0x%X, param 0x%lX", cmd, arg);

	switch(cmd)
	{
	case TIOCEXCL:
		set_bit(CDMX_EXCLUSIVE, &port->flags);
		K_DEBUG("set exclusive");
		return 0;
	case TIOCNXCL:
		K_DEBUG("cleared exclusive");
		clear_bit(CDMX_EXCLUSIVE, &port->flags);
		return 0;
	case 0x5440: //TIOCGEXCL:
		i =  test_bit(CDMX_EXCLUSIVE, &port->flags);
		K_DEBUG("is exclusive: %d", i);
		return put_user(i, (int __user *)p);
	case TIOCINQ:
		i = cdmx_bytes_to_read(port);
		K_DEBUG("bytes to read: %d", i);
		return put_user(i, (int __user *)p);
	case TIOCOUTQ:
		i = port->write_to.rawsize;
		K_DEBUG("bytes queued: %d", i);
		return put_user(i, (int __user *)p);
	case TCGETS:
		K_DEBUG("TCGETS, doing nothing");
		return 0;
	case TCSETS:
		K_DEBUG("TCSETS, doing nothing");
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

static struct dmx_port *cdmx_create_port_obj(int id)
{
	struct dmx_port *port;
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

	/*
	 * As we have a kset for this kobject, we need to set it before calling
	 * the kobject core.
	 */
	port->kobj.kset = dmx_ports_kset;

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

static void cdmx_destroy_port_obj(struct dmx_port *port)
{
	K_DEBUG("port %d", port->id);
	port->id = PORT_INACTIVE;
	kobject_put(&port->kobj);
}


static int cdmx_create_cdevs (void)
{
	int i, c, err;
	const char 	*chrdev_name 	= "cdmx";

	K_DEBUG("start");
	// Returns zero or a negative error code.
	err = alloc_chrdev_region(&cdmx_device_id,
			BASE_MINOR, cdmx_port_count, chrdev_name);
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
		cdev_init( &dmx_ports[i]->cdev, &cdmx_ops);
		dmx_ports[i]->cdev.owner = THIS_MODULE;
		//  A negative error code is returned on failure.
		err = cdev_add (&dmx_ports[i]->cdev,
				MKDEV(MAJOR(cdmx_device_id), (i+BASE_MINOR)), 1);

		if (err < 0)
		{
			K_ERR("cdev_add failed");
			for (c = i - 1; c >= 0; c--)
			{
				cdev_del(&dmx_ports[c]->cdev);
			}
			err = -ENOENT;
			goto failure3;
		}
	}

	for (i=0; i< cdmx_port_count; i++)
	{
		// Returns &struct device pointer on success, or ERR_PTR() on error.
		dmx_ports[i]->fsdev = \
			device_create(cdmx_devclass, NULL, \
			MKDEV(MAJOR(cdmx_device_id), (i + BASE_MINOR)),
			NULL, "cdmx%03X", (i + BASE_MINOR));

		if (IS_ERR(dmx_ports[i]->fsdev))
		{
			K_ERR("failed create device");
			for (c = i - 1; c >= 0; c--)
				device_destroy(cdmx_devclass,
						MKDEV(MAJOR(cdmx_device_id), (c + BASE_MINOR)));
			err = -ENOENT;
			goto failure4;
		}

	}

	K_DEBUG("->>> done");
	return 0;

failure4:
	for (i=0; i< cdmx_port_count; i++)
		cdev_del(&dmx_ports[i]->cdev);
failure3:
	class_destroy(cdmx_devclass);
failure2:
	unregister_chrdev_region(MKDEV(MAJOR(cdmx_device_id), BASE_MINOR),
			cdmx_port_count);
failure1:
	K_ERR( "->>> cleanup done");
	return err;
}

static void cdmx_remove_cdevs (void)
{
	int i;
	K_DEBUG("start");
	for (i=0; i< cdmx_port_count; i++)
	{
		device_destroy(cdmx_devclass,
				MKDEV(MAJOR(cdmx_device_id), (i + BASE_MINOR)));
		cdev_del(&dmx_ports[i]->cdev);
	}
	class_destroy(cdmx_devclass);
	unregister_chrdev_region(MKDEV(MAJOR(cdmx_device_id), BASE_MINOR),
			cdmx_port_count);
	K_DEBUG( "->>> done");
}



static int __init cdmx_init (void)
{
	int i, c, p, err;

	K_DEBUG("start");
	mutex_init(&cld_lock);
	p = TO_RANGE(cdmx_port_count, CDMX_PORTS_MIN, CDMX_PORTS_MAX);
	if (p != cdmx_port_count)
	{
		K_INFO("port count %d is out of range(%d-%d)",
				cdmx_port_count, CDMX_PORTS_MIN, CDMX_PORTS_MAX);
		cdmx_port_count = p;
	}
	K_INFO("using port count %d", cdmx_port_count);

	dmx_ports_kset = kset_create_and_add("cdmx", NULL, NULL);
	if (!dmx_ports_kset)
	{
		K_ERR("out of memory");
		return -ENOMEM;
	}
	dmx_ports = kzalloc(cdmx_port_count * sizeof(*dmx_ports), GFP_KERNEL);
	if (!dmx_ports)
	{
		K_ERR("out of memory");
		err = -ENOMEM;
		goto failure1;
	}
	for (i=0; i<cdmx_port_count; i++)
	{
		dmx_ports[i] = cdmx_create_port_obj(i + BASE_MINOR);
		if (!dmx_ports[i])
		{
			for (c = 0; c < i; c++)
				cdmx_destroy_port_obj(dmx_ports[c]);
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

	K_DEBUG("->>> done");
	return 0;

failure3:
	cdmx_remove_cdevs();
failure2:
	kfree(dmx_ports);
failure1:
	kset_unregister(dmx_ports_kset);
	K_ERR("resources freed after failure, exiting");
	return err;
}

#ifndef notrace
#define notrace
#warning Re-defining notrace
#endif

static void __exit cdmx_exit(void)
{
	int i;
	K_DEBUG("clean up");

	tty_unregister_ldisc(CDMX_LD);
	cdmx_remove_cdevs();
	for (i=0; i<cdmx_port_count; i++)
		cdmx_destroy_port_obj(dmx_ports[i]);
	kfree(dmx_ports);
	kset_unregister(dmx_ports_kset);

	K_DEBUG("->>> done");
}


module_init(cdmx_init); // @suppress("Unused variable declaration in file scope")
module_exit(cdmx_exit); // @suppress("Unused variable declaration in file scope")

