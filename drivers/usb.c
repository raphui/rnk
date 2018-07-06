#include <kernel/printk.h>
#include <utils.h>
#include <drv/usb.h>
#include <mm/mm.h>
#include <errno.h>
#include <string.h>
#include <init.h>
#include <kernel/kmutex.h>

static int dev_count = 0;
static struct list_node usb_device_list;

int usb_read(struct device *dev, unsigned char *buff, unsigned int size)
{
	int ret;
	struct usb_device *usb = container_of(dev, struct usb_device, dev);

	ret = usb->usb_ops->read(usb, buff, size);

	return ret;
}

int usb_write(struct device *dev, unsigned char *buff, unsigned int size)
{
	int ret;
	struct usb_device *usb = container_of(dev, struct usb_device, dev);

	ret = usb->usb_ops->write(usb, buff, size);

	return ret;
}

struct usb_device *usb_new_device(void)
{
	struct usb_device *usbdev = NULL;

	usbdev = (struct usb_device *)kmalloc(sizeof(struct usb_device));
	if (!usbdev) {
		error_printk("cannot allocate usb device\n");
		return NULL;
	}

	memset(usbdev, 0, sizeof(struct usb_device));

	dev_count++;

	return usbdev;
}

int usb_remove_device(struct usb_device *usb)
{
	int ret = 0;
	struct usb_device *usbdev = NULL;

	ret = device_unregister(&usb->dev);
	if (ret < 0) {
		error_printk("failed to unregister usb device");
		return ret;
	}

	list_for_every_entry(&usb_device_list, usbdev, struct usb_device, node)
		if (usbdev == usb)
			break;

	if (usbdev) {
		list_delete(&usbdev->node);
		kfree(usbdev);
	}
	else
		ret = -ENOENT;


	dev_count--;

	return ret;
}

int usb_register_device(struct usb_device *usb)
{
	int ret = 0;

	snprintf(usb->dev.name, sizeof(usb->dev.name), "/dev/usb%d", dev_count);

	usb->dev.read = usb_read;
	usb->dev.write = usb_write;

	list_add_tail(&usb_device_list, &usb->node);

	ret = device_register(&usb->dev);
	if (ret < 0)
		error_printk("failed to register usb device\n");

	return ret;
}

int usb_init(void)
{
	int ret = 0;

	list_initialize(&usb_device_list);

	return ret;
}
postcore_initcall(usb_init);
