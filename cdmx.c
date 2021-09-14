
#include "cdmx.h"
#include <uapi/linux/eventpoll.h>
#include <linux/ctype.h>
#include <linux/printk.h>
#include <uapi/asm-generic/errno-base.h>
#include <uapi/asm-generic/ioctls.h>
#include <uapi/asm-generic/fcntl.h>
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


static int cld_open(struct tty_struct *tty)
{
	int retval = -EEXIST;

/*
	mutex_lock(&routelock);
	if (tr_data->opencalled == 0) {

		tr_data->kref_tty = tty_kref_get(tty);
		if (tr_data->kref_tty == NULL) {
			retval = -EFAULT;
		} else {
			tr_data->opencalled = 1;
			tty->disc_data      = tr_data;
			tty->receive_room   = RECEIVE_ROOM;
			tty_driver_flush_buffer(tty);
			retval = 0;
		}
	}
	mutex_unlock(&routelock);
	*/
	K_DEBUG("->>>");
	return retval;
}

static void cld_close(struct tty_struct *tty)
{
/*	struct tracerouter_data *tptr = tty->disc_data;

	mutex_lock(&routelock);
	WARN_ON(tptr->kref_tty != tr_data->kref_tty);
	tty_driver_flush_buffer(tty);
	tty_kref_put(tr_data->kref_tty);
	tr_data->kref_tty = NULL;
	tr_data->opencalled = 0;
	tty->disc_data = NULL;
	mutex_unlock(&routelock);
*/
	K_DEBUG("->>>");
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

static void cld_receive_buf(struct tty_struct *tty,
					const unsigned char *cp,
					char *fp, int count)
{
	/*
	mutex_lock(&routelock);
	n_tracesink_datadrain((u8 *) cp, count);
	mutex_unlock(&routelock);
	*/
	K_DEBUG("->>>");
}

static struct tty_ldisc_ops cld_ops =
{
	.owner			= THIS_MODULE,
	.magic			= TTY_LDISC_MAGIC, // @suppress("Symbol is not resolved")
	.name			= "cdmx",
	.open			= cld_open,
	.close			= cld_close,
	.read			= cld_read,
	.write			= cld_write,
	.receive_buf	= cld_receive_buf,
};
/*
	struct file *f = filp_open("/path/to/file", FLAGS, MODE);

	mm_segment_t oldfs;
	oldfs = get_fs();
	set_fs (KERNEL_DS);

//    oldfs = get_fs();
//    set_fs(get_ds());
//

	vfs_read(f, buf, len, &f->f_pos);
	vfs_write(f, otherbuf, otherlen, &f->f_pos);

	set_fs(oldfs);

	fput(f);


#ifdef CONFIG_CONSOLE_POLL


*/

extern struct char_device_struct *chrdevs[255];

static int cld_attach(struct dmx_port *pt, char *ttyname)
{
//	struct file *f = filp_open(ttyname, O_RDWR|O_NOCTTY|O_EXCL, 0);
	struct file* port;
	struct tty_struct *tty;
	struct char_device_struct *ch = chrdevs[0];

	K_DEBUG("%p", ch);
//	mm_segment_t oldfs;
//
//	oldfs=get_fs(); // @suppress("Field cannot be resolved")
//	set_fs(KERNEL_DS); // @suppress("Type cannot be resolved")
//
//	port = filp_open(ttyname, O_RDWR | O_NONBLOCK | O_NOCTTY, 0);
//
//	if (IS_ERR (port))
//	{
//		K_ERR("PORT: can't open PHY port '%s'\n", ttyname);
//		goto exit;
//	}
//	else
//	{
//		tty=(struct tty_struct*)port->private_data;
//		K_DEBUG("%d", tty->ldisc->ops->magic);
//		filp_close(port, 0);
//	}
//
//
//
//	exit:
//	        set_fs(oldfs);
	        return 0;
}

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

	//TODO: handle value changes
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

			//TODO: close previous TTY
			kfree(port->ttyname);
			//TODO: open new TTY
			cld_attach(port, newname);
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
		cld_attach(port, newname);
		//TODO: open new TTY
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
	struct dmx_port *cdata;
	cdata = container_of(node->i_cdev, struct dmx_port, cdev);
	K_DEBUG("file %s, cdata %p dev_t %X",
			f->f_path.dentry->d_name.name, cdata, cdata->cdev.dev);

	f->private_data = cdata;
	try_module_get(THIS_MODULE);

	K_DEBUG( "->>> done");
	return 0;
}

static int cdmx_release (struct inode *node, struct file *f)
{
	struct dmx_port *cdata;
	cdata = (struct dmx_port *) f->private_data;
	K_DEBUG("file %s, cdata %p dev_t %X",
			f->f_path.dentry->d_name.name, cdata, cdata->cdev.dev);

	module_put(THIS_MODULE);
	return 0;
}

static void cdmx_enttec_mkframe(struct usb_frame *frame, uint8_t label, uint16_t size)
{
	frame->data[0] = ENT_SOM;
	frame->data[1] = label;
	frame->data[2] = size & 0x00FF;
	frame->data[3] = (size >> 8) & 0x00FF;
	frame->data[ENT_HEADER_SIZE + size] = ENT_EOM;

	frame->msgsize = size + ENT_FRAME_OVERHEAD;
	frame->rhead = 0;
	frame->pending = true;
}

static void cdmx_enttec_getparams (struct dmx_port *port)
{
	struct usb_frame *frame = &port->read_from;

	// supply timing info and firmware vervion
	// ENT_FW_DMX, here we pretend to support just DMX, not RDM
	frame->data[4] = ENT_FW_LSB;
	frame->data[5] = ENT_FW_DMX;
	frame->data[6] = (port->breaktime * 1000) / ENT_TIMEUNIT_NS + 1;
	frame->data[7] = (port->mabtime * 1000) / ENT_TIMEUNIT_NS + 1;
	frame->data[8] = port->framerate;

	cdmx_enttec_mkframe(frame, LABEL_GET_PARAMS, ENT_PARAMS_MIN);
}

static void cdmx_enttec_setparams (struct dmx_port *port)
{
	struct usb_frame *frame = &port->read_from;
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
	struct usb_frame *frame = &port->read_from;
	uint8_t serial[ENT_SERIAL_SIZE] = {port->id, 0,0,0};

	memcpy(&frame->data[ENT_HEADER_SIZE], serial, ENT_SERIAL_SIZE);
	cdmx_enttec_mkframe(frame, LABEL_GET_SERIAL, ENT_SERIAL_SIZE);

	K_DEBUG("message %d bytes", frame->msgsize);
}


static void cdmx_enttec_getvendor (struct dmx_port *port)
{
	struct usb_frame *frame = &port->read_from;
	unsigned int size = sizeof(USBPRO_VENDOR);

	// supply vendor name and ESTA codes
	memcpy(&frame->data[ENT_HEADER_SIZE], USBPRO_VENDOR, size);
	cdmx_enttec_mkframe(frame, LABEL_VENDOR, size);

	K_DEBUG("message %d bytes", frame->msgsize);
}

static void cdmx_enttec_getname (struct dmx_port *port)
{
	struct usb_frame *frame = &port->read_from;
	unsigned int size = sizeof(USBPRO_NAME);

	// supply device name and ESTA codes
	memcpy(&frame->data[ENT_HEADER_SIZE], USBPRO_NAME, size);
	cdmx_enttec_mkframe(frame, LABEL_NAME, size);

	K_DEBUG("message %d bytes", frame->msgsize);
}

static void cdmx_enttec_flash (struct dmx_port *port)
{
	struct usb_frame *frame = &port->read_from;

	// report failed firmware flashing
	memcpy(&frame->data[ENT_HEADER_SIZE], ENT_FLASH_FALSE, ENT_FLASH_REPLY);
	cdmx_enttec_mkframe(frame, LABEL_NAME, ENT_FLASH_REPLY);

	K_DEBUG("message %d bytes", frame->msgsize);
}


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


static int cdmx_enttec_msg(struct dmx_port *port)
{
	int result = 1;
	if (port->read_from.pending)
	{
		K_ERR("output frame is pending, message is dropped: port %d, label %d",
				port->id, port->write_to.msglabel);
		return -EBUSY;
	}

	
	K_DEBUG("port %d: received label %d, size %d bytes",
			port->id, port->write_to.msglabel, port->write_to.whead);
	switch (port->write_to.msglabel)
	{
		// labels that generate replies
		case LABEL_GET_PARAMS:
			cdmx_enttec_getparams(port);
			break;
		case LABEL_GET_SERIAL:
			cdmx_enttec_getserial(port);
			break;
		case LABEL_VENDOR:
			cdmx_enttec_getvendor(port);
			break;
		case LABEL_NAME:
			cdmx_enttec_getname(port);
			break;
		case LABEL_FLASH_PAGE:
			cdmx_enttec_flash(port);
			break;

		// labels that don't need replies
		case LABEL_SET_PARAMS:
			cdmx_enttec_setparams(port);
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
	port->write_to.whead = 0;
	if (port->read_from.pending)
	{
		K_DEBUG("new message pending, label=%d", port->write_to.msglabel);
		wake_up(&port->wait);
	}

	//TODO: implement TX
	K_DEBUG("->>> TX not implemented yet");
	return result;
}

static void cdmx_enttec_parse(struct dmx_port *port)
{
	struct usb_frame *write_to = &port->write_to;
	ssize_t count;
	uint8_t nextbyte;

	for (count=0; count < write_to->rawsize; count++)
	{
		nextbyte = write_to->rawdata[count];
		switch (write_to->state)
		{
		case PRE_SOM:
			if (nextbyte == ENT_SOM)
				write_to->state = GOT_SOM;
		break;
		case GOT_SOM:
			write_to->msglabel = nextbyte;
			write_to->state = GOT_LABEL;
		break;
		case GOT_LABEL:
			write_to->rhead = 0;
			write_to->msgsize = nextbyte;
			write_to->state = GOT_SIZE_LSB;
		break;
		case GOT_SIZE_LSB:
			write_to->msgsize += (nextbyte << 8);

			if (write_to->msgsize == 0)
				write_to->state = WAITING_FOR_EOM;
			else if (write_to->msgsize > DMX_FRAME_MAX)
				{
					K_INFO("dropping broken usb frame, size '%d' \
							is greater than possible", write_to->msgsize);
					write_to->state = PRE_SOM;
				}
			else
				write_to->state = IN_DATA;
		break;
		case IN_DATA:
			write_to->data[write_to->whead] = nextbyte;
			write_to->whead++;

			if (write_to->whead == write_to->msgsize)
				write_to->state = WAITING_FOR_EOM;
		break;
		case WAITING_FOR_EOM:
			if (nextbyte == ENT_EOM)
				{
				write_to->state = PRE_SOM;
				cdmx_enttec_msg(port);
				}
		break;
		}
	}
	write_to->rawsize = 0;
}

static inline size_t cdmx_space_left(struct dmx_port *cdata)
{
	return (sizeof(cdata->write_to.rawdata) - cdata->write_to.rawsize );
}

static int cdmx_bytes_to_read(struct dmx_port *cdata)
{
	struct usb_frame *frame = &cdata->read_from;
	if (!frame->pending)
		return 0;
	return frame->msgsize - frame->rhead;
}

static ssize_t cdmx_write (struct file *f, const char* src,
		size_t len, loff_t * offset)
{
	struct dmx_port *cdata = f->private_data;
	struct usb_frame *write_to = &cdata->write_to;
	ssize_t result, size;

	K_DEBUG("port %d", cdata->id);
	if (mutex_lock_interruptible(&cdata->lock))
		return -ERESTARTSYS;

	if (!cdata->fsdev)
	{
		K_ERR( "device #%d doesn't exist", cdata->id);
		result = -ENODEV;
		goto out;
	}
	size = sizeof(write_to->rawdata) - write_to->rawsize;
	if (size < 0)
	{
		K_ERR("rawsize greater than buffer size: %d bytes", write_to->rawsize);
		result = -EINVAL;
		goto out;
	}

	size = MIN(size, len);
	if (copy_from_user(&write_to->rawdata[write_to->rawsize], src, size))
	{
		result = -EFAULT;
		goto out;
	}
	write_to->rawsize += size;
	result = size;

	cdmx_enttec_parse(cdata);

	out:
	mutex_unlock(&cdata->lock);
	K_DEBUG("->>> port %d, bytes written %d", \
			 cdata->id, result);
	return result;
}


static ssize_t cdmx_read 	(struct file *f, char* dest,
		size_t len, loff_t * offset)
{
	struct dmx_port *cdata = (struct dmx_port *) f->private_data;
	struct usb_frame *frame = &cdata->read_from;
	ssize_t result = 0, size = 0;

	K_DEBUG("port %d, max %d bytes to read", cdata->id, len);
	if (mutex_lock_interruptible(&cdata->lock))
		return -ERESTARTSYS;

	if (!cdata->fsdev)
	{
		K_ERR( "device #%d doesn't exist", cdata->id);
		result = -ENODEV;
		goto out;
	}

	if (!frame->pending)
		goto out;

	size = frame->msgsize - frame->rhead;
	K_DEBUG("message %d bytes, %d done", frame->msgsize, frame->rhead);
	size = MIN(size, len);

	if (copy_to_user(dest, &frame->data[frame->rhead], size))
		goto out;

	frame->rhead += size;
	if (frame->rhead == frame->msgsize)
	{
		K_DEBUG("unpending");
		frame->pending = false;
	}
	result = size;

	out:
	mutex_unlock(&cdata->lock);

	K_DEBUG( "->>> port %d, done read: %d, head: %d", cdata->id, size, frame->rhead);
	return size;

}

__poll_t cdmx_poll (struct file *f, struct poll_table_struct *p)
{
	struct dmx_port *cdata = (struct dmx_port *) f->private_data;
	__poll_t mask = 0;
	int r, w;

	mutex_lock(&cdata->lock);

	poll_wait(f, &cdata->wait, p);
	r = cdmx_bytes_to_read(cdata);
	if (r > 0)
		mask |= (EPOLLIN | EPOLLRDNORM | EPOLLPRI );

	w = cdmx_space_left(cdata);
	if( w > 0)
		mask |= EPOLLOUT | EPOLLWRNORM;

	mutex_unlock(&cdata->lock);
	K_DEBUG("port %d, to read: %d, to write: %d", cdata->id, r, w);
	return mask;
}

long cdmx_ioctl (struct file *f, unsigned int cmd, unsigned long arg)
{
	struct dmx_port *cdata = (struct dmx_port *) f->private_data;
	void __user *p = (void __user *)arg;
	int i;

	K_DEBUG("cmd 0x%X, param 0x%lX", cmd, arg);

	switch(cmd)
	{
	case TIOCEXCL:
		set_bit(CDMX_EXCLUSIVE, &cdata->flags);
		K_DEBUG("set exclusive");
		return 0;
	case TIOCNXCL:
		K_DEBUG("cleared exclusive");
		clear_bit(CDMX_EXCLUSIVE, &cdata->flags);
		return 0;
	case 0x5440: //TIOCGEXCL:
		i =  test_bit(CDMX_EXCLUSIVE, &cdata->flags);
		K_DEBUG("is exclusive: %d", i);
		return put_user(i, (int __user *)p);
	case TIOCINQ:
		i = cdmx_bytes_to_read(cdata);
		K_DEBUG("bytes to read: %d", i);
		return put_user(i, (int __user *)p);
	case TIOCOUTQ:
		i = cdata->write_to.rawsize;
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

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------


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

static struct dmx_port *create_port_obj(int id)
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

	mutex_init(&port->lock);
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

static void destroy_port_obj(struct dmx_port *port)
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
		dmx_ports[i] = create_port_obj(i + BASE_MINOR);
		if (!dmx_ports[i])
		{
			for (c = 0; c < i; c++)
				destroy_port_obj(dmx_ports[c]);
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

static void  cdmx_exit(void)
{
	int i;
	K_DEBUG("clean up");

	tty_unregister_ldisc(CDMX_LD);
	cdmx_remove_cdevs();
	for (i=0; i<cdmx_port_count; i++)
		destroy_port_obj(dmx_ports[i]);
	kfree(dmx_ports);
	kset_unregister(dmx_ports_kset);

	K_DEBUG("->>> done");
}


module_init(cdmx_init);
module_exit(cdmx_exit);

