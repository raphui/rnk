/*
 * Copyright (C) 2014  Raphaël Poggi <poggi.raph@gmail.com>
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
#include <usart.h>
#include <mm.h>
#include <errno.h>
#include <string.h>
#include <init.h>
#include <console.h>

static int dev_count = 0;
static int master_count = 0;
static char dev_prefix[10] = "/dev/tty";
static struct list_node usart_device_list;
static struct list_node usart_master_list;


static int usart_read(struct device *dev, unsigned char *buff, unsigned int size)
{
	struct usart_master *usart = container_of(dev, struct usart_master, dev);

	verbose_printk("reading from usart !\n");

	return usart->usart_ops->read(usart, buff, size);
}

static int usart_write(struct device *dev, unsigned char *buff, unsigned int size)
{
	struct usart_master *usart = container_of(dev, struct usart_master, dev);

	verbose_printk("writing from usart !\n");

	return usart->usart_ops->write(usart, buff, size);
}

struct usart_device *usart_new_device(void)
{
	struct usart_device *usartdev = NULL;

	usartdev = (struct usart_device *)kmalloc(sizeof(struct usart_device));
	if (!usartdev) {
		error_printk("cannot allocate usart device\n");
		return NULL;
	}
	
	memset(usartdev, 0, sizeof(struct usart_device));

	dev_count++;

	return usartdev;
}

int usart_remove_device(struct usart_device *usart)
{
	int ret = 0;
	struct usart_device *usartdev = NULL;

	ret = device_unregister(&usart->dev);
	if (ret < 0) {
		error_printk("failed to unregister usart device");
		return ret;
	}

	list_for_every_entry(&usart_device_list, usartdev, struct usart_device, node)
		if (usartdev == usart)
			break;

	if (usartdev) {
		list_delete(&usartdev->node);
		kfree(usartdev);
	}
	else
		ret = -ENOENT;


	dev_count--;

	return ret;
}

int usart_register_device(struct usart_device *usart)
{
	int ret = 0;
	char tmp[10] = {0};

	memcpy(tmp, dev_prefix, sizeof(dev_prefix));
	
	/* XXX: ascii 0 start at 0x30 */
	tmp[8] = 0x30 + dev_count;

	memcpy(usart->dev.name, tmp, sizeof(tmp));

	list_add_tail(&usart_device_list, &usart->node);

	ret = device_register(&usart->dev);
	if (ret < 0)
		error_printk("failed to register usart device\n");

	return ret;
}

struct usart_master *usart_new_master(void)
{
	struct usart_master *usart = NULL;

	usart = (struct usart_master *)kmalloc(sizeof(struct usart_master));
	if (!usart) {
		error_printk("cannot allocate usart master\n");
		return NULL;
	}

	memset(usart, 0, sizeof(struct usart_master));

	master_count++;

	return usart;
}

int usart_remove_master(struct usart_master *usart)
{
	int ret = 0;
	struct usart_master *usartdev = NULL;

	ret = device_unregister(&usart->dev);
	if (ret < 0) {
		error_printk("failed to unregister usart master");
		return ret;
	}

	list_for_every_entry(&usart_master_list, usartdev, struct usart_master, node)
		if (usartdev == usart)
			break;

	if (usartdev) {
		list_delete(&usartdev->node);
		kfree(usartdev);
	}
	else
		ret = -ENOENT;



	master_count--;

	return ret;
}

int usart_register_master(struct usart_master *usart)
{
	int ret = 0;
	char tmp[10] = {0};

	memcpy(tmp, dev_prefix, sizeof(dev_prefix));

	/* XXX: ascii 0 start at 0x30 */
	tmp[8] = 0x30 + master_count;

	memcpy(usart->dev.name, tmp, sizeof(tmp));

	list_add_tail(&usart_master_list, &usart->node);

	ret = device_register(&usart->dev);
	if (ret < 0)
		error_printk("failed to register usart master\n");

	return ret;
}

int usart_init(void)
{
	int ret = 0;

	list_initialize(&usart_device_list);
	list_initialize(&usart_master_list);

	return ret;
}
postcore_initcall(usart_init);

#ifdef CONFIG_USART_DEBUG

struct io_operations io_op = {
	.write = usart_write,
};

#endif /* CONFIG_USART_DEBUG */
