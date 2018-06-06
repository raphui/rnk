/*
 * Copyright (C) 2018  Raphaël Poggi <poggi.raph@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <printk.h>
#include <utils.h>
#include <usb.h>
#include <mm.h>
#include <errno.h>
#include <string.h>
#include <init.h>
#include <kmutex.h>

static int dev_count = 0;
static char dev_prefix[10] = "/dev/usb";
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
	char tmp[10] = {0};

	memcpy(tmp, dev_prefix, sizeof(dev_prefix));
	
	/* XXX: ascii 0 start at 0x30 */
	tmp[8] = 0x30 + dev_count;

	memcpy(usb->dev.name, tmp, sizeof(tmp));

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
