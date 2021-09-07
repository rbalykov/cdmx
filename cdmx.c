
#include "cdmx.h"
#include <uapi/linux/eventpoll.h>
#include <linux/ctype.h>
#include <linux/printk.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Neutrino");
MODULE_DESCRIPTION("DMX character device");
MODULE_VERSION("0.01");


int cdmx_port_count = CDMX_PORTS_DEFAULT;
module_param(cdmx_port_count, hexint, CHMOD_RO);


static struct class *cdmx_devclass 	= NULL;
static dev_t 		cdmx_device_id;
//struct cdev cdmx_cdev;

static inline bool port_active(struct dmx_port * port)
{
	return (port->id >= 0);
}

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
	return scnprintf(buf, PAGE_SIZE, "%s", port->ttyname);
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

static struct kset *dmx_ports_kset;
static struct dmx_port **dmx_ports;

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

	port->breaktime = DEFAULT_BREAK;
	port->mabtime 	= DEFAULT_MAB;
	port->framerate = DEFAULT_FRAMERATE;
	port->id = PORT_INACTIVE;

	mutex_init(&port->read_mutex);
	mutex_init(&port->write_mutex);
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

	// set port active
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


//const char cdmx_ttytemplate[] = "tty%04X";
//const char cdmx_devtemplate[] = "cdmx%04X";
//char cdmx_ttyfile[] = "tty0000";

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

	K_DEBUG("->>> done");
	return 0;

failure2:
	kfree(dmx_ports);
failure1:
	kset_unregister(dmx_ports_kset);
	K_ERR("resources freed after failure, exiting");
	return err;
}

static void __exit cdmx_exit(void)
{
	int i;
	K_DEBUG("start");

	cdmx_remove_cdevs();
	for (i=0; i<cdmx_port_count; i++)
		destroy_port_obj(dmx_ports[i]);
	kfree(dmx_ports);
	kset_unregister(dmx_ports_kset);

	K_DEBUG("->>> done");
}




// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------


static struct file_operations cdmx_ops =
{
	.owner		= THIS_MODULE,
	.read		= cdmx_read,
	.write		= cdmx_write,
	.open		= cdmx_open,
	.release 	= cdmx_release
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
	K_DEBUG("file %s, cdata %p dev_t %X", f->f_path.dentry->d_name.name, cdata, cdata->cdev.dev);

	f->private_data = cdata;
	try_module_get(THIS_MODULE);

	K_DEBUG( "->>> done");
	return 0;
}

static int cdmx_release (struct inode *node, struct file *f)
{
	struct dmx_port *cdata;
	cdata = (struct dmx_port *) f->private_data;
	K_DEBUG("file %s, cdata %p dev_t %X", f->f_path.dentry->d_name.name, cdata, cdata->cdev.dev);

	module_put(THIS_MODULE);
	return 0;
}

static ssize_t cdmx_read 	(struct file *f, char* dest, size_t len, loff_t * offset)
{
	struct dmx_port *cdata;
	cdata = (struct dmx_port *) f->private_data;
	K_DEBUG("file %s, cdata %p dev_t %X", f->f_path.dentry->d_name.name, cdata, cdata->cdev.dev);

	/*
	struct dmx_port *cdata = (struct dmx_port *) f->private_data;
	ssize_t queue, result;

	K_DEBUG("file %s", f->f_path.dentry->d_name.name);
	mutex_lock(&cdata->read_mutex);
	queue = cdata->read_from.size - cdata->read_from.head;
	queue = MIN (queue, len);
	if (queue < 1 )
	{
		K_DEBUG("empty queue");
		mutex_unlock(&cdata->read_mutex);
		return 0;
	}
	result = copy_to_user(dest, &cdata->read_from.data[cdata->read_from.head], queue);
	cdata->read_from.head += result;
	mutex_unlock(&cdata->read_mutex);

	K_DEBUG( "returning size %d", result);
	return result;
*/	return -EIO;
}
void n_dmx_usbpro_msg (struct usb_frame *frame)
{
	K_DEBUG( "LABEL=%d, PAYLOAD=%d", \
			frame->label, frame->datasize );
}

static ssize_t cdmx_write (struct file *f, const char* src, size_t len, loff_t * offset)
{
	struct dmx_port *cdata;
	struct usb_frame *write_to;
	ssize_t count, result;
	uint8_t nextbyte = 0;

	cdata = (struct dmx_port *) f->private_data;

	K_DEBUG("file %s, cdata %p dev_t %X id %d", \
			f->f_path.dentry->d_name.name, cdata, cdata->cdev.dev, cdata->id);
	K_DEBUG( "%d bytes to process", len);

	write_to = &cdata->write_to;
	mutex_lock(&cdata->write_mutex);
	mutex_lock(&cdata->read_mutex);

	for (count=0; count<len; count++)
	{
		nextbyte = src[count];
		switch (write_to->state)
		{
		case PRE_SOM:
			if (nextbyte == DMX_USBPRO_MAGIC_START)
			{
				write_to->state = GOT_SOM;
			//	write_to->stream = 1;
			}
		break;
		case GOT_SOM:
			write_to->label = nextbyte;
			write_to->state = GOT_LABEL;
		break;
		case GOT_LABEL:
			write_to->head = 0;
			write_to->datasize = nextbyte;
			write_to->state = GOT_DATA_LSB;
		break;
		case GOT_DATA_LSB:
			write_to->datasize += (nextbyte << 8);
			if (write_to->datasize == 0)
				write_to->state = WAITING_FOR_EOM;
			else
				write_to->state = IN_DATA;

			write_to->datasize = MIN(write_to->datasize, DMX_FRAME_MAX);
			write_to->datasize = MAX(write_to->datasize, DMX_FRAME_MIN);
		break;
		case IN_DATA:
			// handle frame shorter than expected
			if (nextbyte == DMX_USBPRO_MAGIC_END)
				{
				write_to->state = PRE_SOM;
				n_dmx_usbpro_msg(write_to);
				}
			write_to->data[write_to->head] = nextbyte;
			write_to->head++;

			//TODO
			if (write_to->head == write_to->datasize)
			{
				write_to->state = WAITING_FOR_EOM;
			}
		break;
		case WAITING_FOR_EOM:
			if (nextbyte == DMX_USBPRO_MAGIC_END)
				{
				write_to->state = PRE_SOM;
				n_dmx_usbpro_msg(write_to);
				}
		break;
		}
	}

	mutex_unlock(&cdata->read_mutex);
	mutex_unlock(&cdata->write_mutex);

	//TODO
	result = len;
	K_DEBUG( "%d bytes processed", result);
	return result;
}
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------



static int cdmx_create_cdevs (void)
{
	int i, c, err;
	const char 	*chrdev_name 	= "cdmx";

	K_DEBUG("start");
	// Returns zero or a negative error code.
	err = alloc_chrdev_region(&cdmx_device_id, BASE_MINOR, cdmx_port_count, chrdev_name);
	if (err)
	{
		K_ERR("failed to allocate chrdev region for '%s'", chrdev_name);
		err = -ENOENT;
		goto failure1;
	}
	K_DEBUG("allocated chrdev region, creating class");

	// Returns &struct class pointer on success, or ERR_PTR() on error.
	cdmx_devclass = class_create(THIS_MODULE, chrdev_name);
	if (IS_ERR(cdmx_devclass))
	{
		K_ERR("failed to create class '%s'", chrdev_name);
		err = -ENOENT;
		goto failure2;
	}
	// handles chrdev CHMOD
	K_DEBUG("created class, adding cdev");
	cdmx_devclass->devnode = cdmx_devnode;


	for (i=0; i< cdmx_port_count; i++)
	{
		cdev_init( &dmx_ports[i]->cdev, &cdmx_ops);
		dmx_ports[i]->cdev.owner = THIS_MODULE;
		//  A negative error code is returned on failure.
		err = cdev_add (&dmx_ports[i]->cdev, MKDEV(MAJOR(cdmx_device_id), (i+BASE_MINOR)), 1);

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
	K_DEBUG("cdev added, creating devices");

//	cdmx_ports = kzalloc(cdmx_port_count * sizeof(struct cdmx_data), GFP_KERNEL);
//	if (!cdmx_ports)
//	{
//		K_ERR("out of memory");
//		err = -ENOMEM;
//		goto failure4;
//	}
	for (i=0; i< cdmx_port_count; i++)
	{
		// Returns &struct device pointer on success, or ERR_PTR() on error.
		dmx_ports[i]->fsdev = \
			device_create(cdmx_devclass, NULL, \
			MKDEV(MAJOR(cdmx_device_id), (i + BASE_MINOR)), NULL, "cdmx%03X", (i + BASE_MINOR));

		if (IS_ERR(dmx_ports[i]->fsdev))
		{
			K_ERR("failed create device");
			for (c = i - 1; c >= 0; c--)
				device_destroy(cdmx_devclass, MKDEV(MAJOR(cdmx_device_id), (c + BASE_MINOR)));
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
	unregister_chrdev_region(MKDEV(MAJOR(cdmx_device_id), BASE_MINOR), cdmx_port_count);
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
		device_destroy(cdmx_devclass, MKDEV(MAJOR(cdmx_device_id), (i + BASE_MINOR)));
		cdev_del(&dmx_ports[i]->cdev);
	}
	class_destroy(cdmx_devclass);
	unregister_chrdev_region(MKDEV(MAJOR(cdmx_device_id), BASE_MINOR), cdmx_port_count);
	K_DEBUG( "->>> done");
}



// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
/*
static int __init cdmx_init (void)
{
	int i, p, err=0;
	struct attribute **ttyattr;

	K_DEBUG( "CDMX init");
	p = MAX(PORT_COUNT_MIN, cdmx_port_count);
	p = MIN(PORT_COUNT_MAX, cdmx_port_count);
	K_DEBUG( "CDMX init: port count = %d", p);
	cdmx_port_count = p;

	ttyattr = (struct attribute **) \
			kzalloc( (cdmx_port_count+1) * sizeof(struct attribute *), GFP_KERNEL);
	if (ttyattr == NULL)
	{
		K_ERR("CDMX init: %s: %d: out of memory", __FILE__, __LINE__);
		err = -ENOMEM;
		goto failure1;
	}

	cdmx_ports = (struct cdmx_data *) \
			kzalloc( cdmx_port_count * sizeof(struct cdmx_data), GFP_KERNEL);
	if (cdmx_ports == NULL)
	{
		K_ERR("CDMX init: %s: %d: out of memory", __FILE__, __LINE__);
		err = -ENOMEM;
		goto failure2;
	}
	for (i=0; i< cdmx_port_count; i++)
	{
		cdmx_ports[i].port_id = i;
		snprintf(cdmx_ttyfile, sizeof(cdmx_ttytemplate), cdmx_ttytemplate, i);
		cdmx_ports[i].tty.attr.name = kstrdup(cdmx_ttyfile, GFP_KERNEL);
		cdmx_ports[i].tty.attr.mode = CHMOD_RW;
		cdmx_ports[i].tty.file = kstrdup("", GFP_KERNEL);
		ttyattr[i] = &(cdmx_ports[i].tty.attr);

//		if ((cdmx_ports[i].tty.attr.name == NULL) ||
//			(cdmx_ports[i].tty.file == NULL))
//		{
//			K_ERR("CDMX init: %s: %d: out of memory", __FILE__, __LINE__);
//			err = -ENOMEM;
//			goto failure6;
//		}

		mutex_init(&cdmx_ports[i].read_mutex);
		mutex_init(&cdmx_ports[i].write_mutex);
		ttyattr[i] = &cdmx_ports[i].tty.attr;
	}
	ttyattr[cdmx_port_count] = NULL;
	ttykobj.default_attrs = ttyattr;

	// Returns zero or a negative error code.
	err = alloc_chrdev_region(&cdmx_device_id, BASE_MINOR, cdmx_port_count, cdmx_devname);
	if (err)
	{
		K_ERR("CDMX init: %s: %d: failed alloc_chrdev_region", __FILE__, __LINE__);
		err = -ENOENT;
		goto failure3;
	}

	// Returns &struct class pointer on success, or ERR_PTR() on error.
	cdmx_devclass = class_create(THIS_MODULE, cdmx_devname);
	if (IS_ERR(cdmx_devclass))
	{
		K_ERR("CDMX init: %s: %d: failed class_create", __FILE__, __LINE__);
		err = -ENOENT;
		goto failure4;
	}
	cdmx_devclass->devnode = cdmx_devnode;

//	cdev_init( &(cdmx_ports[0].cdev), &cdmx_ops);
//	cdmx_ports[0].cdev.owner = THIS_MODULE;
//
//	//  A negative error code is returned on failure.
//	err = cdev_add (&cdmx_ports[i].cdev, MKDEV(MAJOR(cdmx_device_id), BASE_MINOR), cdmx_port_count);
//	if (err < 0)
//	{
//		K_ERR("CDMX init: %s: %d: failed cdev_add", __FILE__, __LINE__);
//		goto failure5;
//	}

	for (i=0; i< cdmx_port_count; i++)
	{
		cdev_init( &(cdmx_ports[i].cdev), &cdmx_ops);
		cdmx_ports[i].cdev.owner = THIS_MODULE;

		err = cdev_add (&cdmx_ports[i].cdev, \
				MKDEV(MAJOR(cdmx_device_id), i), 1);
		if (err < 0)
		{
			K_ERR("CDMX init: %s: %d: failed cdev_add", __FILE__, __LINE__);
			goto failure5;
		}
		cdmx_ports[i].cdev_valid = true;
	}

	for (i=0; i< cdmx_port_count; i++)
	{
		cdmx_ports[i].port_id = NOT_VALID;
		// Returns &struct device pointer on success, or ERR_PTR() on error.
		cdmx_ports[i].fsdev = \
			device_create(cdmx_devclass, NULL, \
			MKDEV(MAJOR(cdmx_device_id), i), NULL, cdmx_devtemplate, i);
		if (IS_ERR(cdmx_ports[i].fsdev))
		{
			K_ERR("CDMX init: %s: %d: failed device_create", __FILE__, __LINE__);
			err = -ENODEV;
			goto failure6;
		}

	}



	kobject_init(&kobj, &ttykobj);

//	 * Return: If this function returns an error, kobject_put() must be
//	 *         called to properly clean up the memory associated with the
//	 *         object.  Under no instance should the kobject that is passed
//	 *         to this function be directly freed with a call to kfree(),
//	 *         that can leak memory.
//	 *
//	 *         If this function returns success, kobject_put() must also be called
//	 *         in order to properly clean up the memory associated with the object.

	err = kobject_add(&kobj, NULL, "%s", "cdmx_p");
	if (err)
	{
		K_ERR("CDMX init: %s: %d: failed kobject_add", __FILE__, __LINE__);
		goto failure7;
	}

	K_DEBUG( "CDMX init done");
	return 0;

	failure7:
		kobject_put(&kobj);
	failure6:
		for (i = cdmx_port_count - 1; i > -1; i--)
		{
			if (cdmx_ports[i].tty.file != NULL) kfree(cdmx_ports[i].tty.file);
			if (cdmx_ports[i].tty.attr.name != NULL) kfree(cdmx_ports[i].tty.attr.name);
			if (!IS_ERR(cdmx_ports[i].fsdev))
			{
				device_destroy(cdmx_devclass, MKDEV(MAJOR(cdmx_device_id), i));
			}
		}
	failure5:
		for (i = cdmx_port_count - 1; i > -1; i--)
		{
			if (cdmx_ports[i].cdev_valid) cdev_del(&cdmx_ports[i].cdev);
		}
		class_destroy(cdmx_devclass);
	failure4:
		unregister_chrdev_region(MKDEV(MAJOR(cdmx_device_id), BASE_MINOR), cdmx_port_count);
	failure3:
		kfree(cdmx_ports);
	failure2:
		kfree(ttyattr);
	failure1:
	K_DEBUG( "CDMX init failed");
		return err;

}

static void __exit cdmx_exit(void)
{
	int i;

	K_DEBUG( "CDMX exit");
	kobject_put(&kobj);
	for (i = cdmx_port_count - 1; i > -1; i--)
	{
		if (cdmx_ports[i].tty.file != NULL) kfree(cdmx_ports[i].tty.file);
		if (cdmx_ports[i].tty.attr.name != NULL) kfree(cdmx_ports[i].tty.attr.name);
		if (!IS_ERR(cdmx_ports[i].fsdev))
		{
			device_destroy(cdmx_devclass, MKDEV(MAJOR(cdmx_device_id), i));
		}
	}
	for (i = cdmx_port_count - 1; i > -1; i--)
	{
		if (cdmx_ports[i].cdev_valid) cdev_del(&cdmx_ports[i].cdev);
	}
	class_destroy(cdmx_devclass);
	unregister_chrdev_region(MKDEV(MAJOR(cdmx_device_id), BASE_MINOR), cdmx_port_count);
	kfree(cdmx_ports);
//	kfree(ttykobj.default_attrs);
	K_DEBUG( "CDMX exit done");
}
*/
module_init(cdmx_init);
module_exit(cdmx_exit);

