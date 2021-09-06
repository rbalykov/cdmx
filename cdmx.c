
#include "cdmx.h"
#include <uapi/linux/eventpoll.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Neutrino");
MODULE_DESCRIPTION("DMX character device");
MODULE_VERSION("0.01");


int cdmx_port_count = PORT_COUNT_MIN;
module_param(cdmx_port_count, hexint, CHMOD_RO);



static struct sysfs_ops ttyname_ops =
{
	.store = ttyname_store,
	.show = ttyname_show
};

static struct kobj_type ttykobj = {
    .sysfs_ops = &ttyname_ops,
    .default_attrs = NULL
};

__poll_t cdmx_poll (struct file *f, struct poll_table_struct *wait)
{
	struct cdmx_data *cdata = (struct cdmx_data *) f->private_data;
	__poll_t mask = 0;
	if (( cdata->read_from.size - cdata->read_from.head) > 0)
	{
		mask |= EPOLLIN | EPOLLRDNORM;
		_CDMX_DEBUG (printk(KERN_INFO "N_DMX poll: data ready\n"); )
	}
	else
	{
		_CDMX_DEBUG (printk(KERN_INFO "N_DMX poll: no data\n"); )
	}
	return mask;
}

static struct file_operations cdmx_ops =
{
	.owner		= THIS_MODULE,
	.read		= cdmx_read,
	.write		= cdmx_write,
	.open		= cdmx_open,
	.release 	= cdmx_release,
	.poll		= cdmx_poll
};


const char cdmx_ttytemplate[] = "tty%04X";
const char cdmx_devtemplate[] = "cdmx%04X";
char cdmx_ttyfile[] = "tty0000";

static struct kobject kobj = {0};

static struct cdmx_data	*cdmx_ports;

static struct class *cdmx_devclass 	= NULL;
static const char 	*cdmx_devname 	= "cdmx";
static dev_t 		cdmx_device_id;

static char *cdmx_devnode(struct device *dev, umode_t *mode)
{
	if (!mode)
		return NULL;
	if ( MAJOR(dev->devt) == MAJOR(cdmx_device_id))
		*mode = CHMOD_RW;
	return NULL;
}

ssize_t	ttyname_show	(struct kobject *kobj, struct attribute *attr, char *buf)
{
    struct ttyname *a = container_of(attr, struct ttyname, attr);
	_CDMX_DEBUG (printk(KERN_DEBUG "CDMX ttyname show: %s=\"%s\"\n", attr->name, a->file); )
    return scnprintf(buf, PAGE_SIZE, a->file);
}

ssize_t	ttyname_store	(struct kobject *kobj, struct attribute *attr, const char *buf, size_t len)
{
    struct ttyname *a = container_of(attr, struct ttyname, attr);
    char *newname;

    newname = kmalloc(len, GFP_KERNEL);

    //TODO: deal with white-space characters in filename
    sscanf(buf, "%s", newname);

    if (strcmp(a->file, newname))
    {
    	kfree(a->file);
    	a->file = newname;
    	_CDMX_DEBUG (printk(KERN_DEBUG "CDMX ttyname_store: name differs, re-attaching LDISC\n"); )
    	//TODO: re-attach LDISC
    }
    else
    {
    	kfree(newname);
    }
    _CDMX_DEBUG (printk(KERN_DEBUG "CDMX ttyname_store: %s=\"%s\"\n", attr->name, a->file); )
    return len;
}

static int cdmx_open 	(struct inode *node, struct file *file)
{
	struct cdmx_data *cdata;

	_CDMX_DEBUG (printk(KERN_DEBUG "CDMX open file %s\n", file->f_path.dentry->d_name.name); )
	cdata = container_of(node->i_cdev, struct cdmx_data, cdev);
	file->private_data = cdata;
	_CDMX_DEBUG (printk(KERN_DEBUG "CDMX open: CDATA=%lx", (unsigned long) file->private_data); )
	try_module_get(THIS_MODULE);
	return 0;
}

static int cdmx_release (struct inode *node, struct file *file)
{
	_CDMX_DEBUG (printk(KERN_DEBUG "CDMX release file %s\n", file->f_path.dentry->d_name.name); )
	module_put(THIS_MODULE);
	return 0;
}

static ssize_t cdmx_read 	(struct file *f, char* dest, size_t len, loff_t * offset)
{/*
	struct cdmx_data *cdata = (struct cdmx_data *) f->private_data;
	ssize_t queue, result;

	if (!port_active(cdata->port_id))
	{
		_CDMX_DEBUG (printk(KERN_DEBUG "CDMX read failure: port id not valid\n"); )
		return -EIO;
	}

	mutex_lock(&cdata->read_mutex);
	queue = cdata->read_from.size - cdata->read_from.head;
	queue = MIN (queue, len);
	if (queue < 1 )
		return 0;

	result = copy_to_user(dest, &cdata->read_from.data[cdata->read_from.head], queue);
	cdata->read_from.head += result;
	mutex_unlock(&cdata->read_mutex);

	_CDMX_DEBUG (printk(KERN_DEBUG "CDMX read: %d bytes\n", result); )
	return result;
*/
	return -EIO;
}
void n_dmx_usbpro_msg (struct usb_frame *frame)
{
	_CDMX_DEBUG (printk(KERN_DEBUG "CDMX message: LABEL=%d, PAYLOAD=%d\n", \
			frame->label, frame->datasize ); )
}

static ssize_t cdmx_write (struct file *f, const char* src, size_t len, loff_t * offset)
{
	struct cdmx_data *cdata;
	struct usb_frame *write_to;
	ssize_t count, result;
	uint8_t nextbyte = 0;

	if (!f)
	{
		_CDMX_ERR (printk(KERN_ERR "CDMX write: null FILE pointer\n"); )
		return -EIO;
	}
	_CDMX_DEBUG (printk(KERN_DEBUG "CDMX write: %d bytes to process, pd=%lx\n", len, (unsigned long)f->private_data); )

	cdata = (struct cdmx_data *) f->private_data;
	if (!cdata)
	{
		_CDMX_ERR (printk(KERN_ERR "CDMX write: CDATA = %lx\n", (unsigned long)cdata); )
		return -EIO;
	}
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
	_CDMX_DEBUG (printk(KERN_DEBUG "CDMX write: %d bytes processed\n", result); )
	return result;
}

// -----------------------------------------------------------------------------

static int __init cdmx_init (void)
{
	int i, p, err=0;
	struct attribute **ttyattr;

	_CDMX_DEBUG (printk(KERN_DEBUG "CDMX init\n"); )
	p = MAX(PORT_COUNT_MIN, cdmx_port_count);
	p = MIN(PORT_COUNT_MAX, cdmx_port_count);
	_CDMX_DEBUG (printk(KERN_DEBUG "CDMX init: port count = %d\n", p); )
	cdmx_port_count = p;

	ttyattr = (struct attribute **) \
			kzalloc( (cdmx_port_count+1) * sizeof(struct attribute *), GFP_KERNEL);
	if (ttyattr == NULL)
	{
		_CDMX_ERR (printk(KERN_ERR "CDMX init: %s: %d: out of memory\n", __FILE__, __LINE__); )
		err = -ENOMEM;
		goto failure1;
	}

	cdmx_ports = (struct cdmx_data *) \
			kzalloc( cdmx_port_count * sizeof(struct cdmx_data), GFP_KERNEL);
	if (cdmx_ports == NULL)
	{
		_CDMX_ERR (printk(KERN_ERR "CDMX init: %s: %d: out of memory\n", __FILE__, __LINE__); )
		err = -ENOMEM;
		goto failure2;
	}
	for (i=0; i< cdmx_port_count; i++)
	{
		mutex_init(&cdmx_ports[i].read_mutex);
		mutex_init(&cdmx_ports[i].write_mutex);
	}

	// Returns zero or a negative error code.
	err = alloc_chrdev_region(&cdmx_device_id, BASE_MINOR, cdmx_port_count, cdmx_devname);
	if (err)
	{
		_CDMX_ERR (printk(KERN_ERR "CDMX init: %s: %d: failed alloc_chrdev_region\n", __FILE__, __LINE__); )
		err = -ENOENT;
		goto failure3;
	}

	// Returns &struct class pointer on success, or ERR_PTR() on error.
	cdmx_devclass = class_create(THIS_MODULE, cdmx_devname);
	if (IS_ERR(cdmx_devclass))
	{
		_CDMX_ERR (printk(KERN_ERR "CDMX init: %s: %d: failed class_create\n", __FILE__, __LINE__); )
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
//		_CDMX_ERR (printk(KERN_ERR "CDMX init: %s: %d: failed cdev_add\n", __FILE__, __LINE__); )
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
			_CDMX_ERR (printk(KERN_ERR "CDMX init: %s: %d: failed cdev_add\n", __FILE__, __LINE__); )
			goto failure5;
		}
		cdmx_ports[i].cdev_valid = true;
	}

	for (i=0; i< cdmx_port_count; i++)
	{
		// Returns &struct device pointer on success, or ERR_PTR() on error.
		cdmx_ports[i].fsdev = \
			device_create(cdmx_devclass, NULL, \
			MKDEV(MAJOR(cdmx_device_id), i), NULL, cdmx_devtemplate, i);
		if (IS_ERR(cdmx_ports[i].fsdev))
		{
			_CDMX_ERR (printk(KERN_ERR "CDMX init: %s: %d: failed device_create\n", __FILE__, __LINE__); )
			err = -ENODEV;
			goto failure6;
		}

		cdmx_ports[i].port_id = i;
		snprintf(cdmx_ttyfile, sizeof(cdmx_ttytemplate), cdmx_ttytemplate, i);
		cdmx_ports[i].tty.attr.name = kstrdup(cdmx_ttyfile, GFP_KERNEL);
		cdmx_ports[i].tty.attr.mode = CHMOD_RW;
		cdmx_ports[i].tty.file = kstrdup("", GFP_KERNEL);
		ttyattr[i] = &(cdmx_ports[i].tty.attr);

		if ((cdmx_ports[i].tty.attr.name == NULL) ||
			(cdmx_ports[i].tty.file == NULL))
		{
			_CDMX_ERR (printk(KERN_ERR "CDMX init: %s: %d: out of memory\n", __FILE__, __LINE__); )
			err = -ENOMEM;
			goto failure6;
		}
	}

	for (i=0; i<cdmx_port_count; i++)
	{
		ttyattr[i] = &(cdmx_ports[i].tty.attr);
	}
	ttyattr[cdmx_port_count+1] = NULL;
	ttykobj.default_attrs = ttyattr;

	kobject_init(&kobj, &ttykobj);
	/*
	 * Return: If this function returns an error, kobject_put() must be
	 *         called to properly clean up the memory associated with the
	 *         object.  Under no instance should the kobject that is passed
	 *         to this function be directly freed with a call to kfree(),
	 *         that can leak memory.
	 *
	 *         If this function returns success, kobject_put() must also be called
	 *         in order to properly clean up the memory associated with the object.
	 */
	err = kobject_add(&kobj, NULL, "%s", "cdmx_p");
	if (err)
	{
		_CDMX_ERR (printk(KERN_ERR "CDMX init: %s: %d: failed kobject_add\n", __FILE__, __LINE__); )
		goto failure7;
	}

	_CDMX_DEBUG (printk(KERN_DEBUG "CDMX init done\n"); )
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
	_CDMX_DEBUG (printk(KERN_DEBUG "CDMX init failed\n"); )
		return err;
}

static void __exit cdmx_exit(void)
{
	int i;

	_CDMX_DEBUG (printk(KERN_DEBUG "CDMX exit\n"); )
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
	kfree(ttykobj.default_attrs);
	_CDMX_DEBUG (printk(KERN_DEBUG "CDMX exit done\n"); )
}

module_init(cdmx_init);
module_exit(cdmx_exit);

