#include <printk.h>
#include <utils.h>
#include <usart.h>
#include <mm.h>
#include <errno.h>
#include <string.h>
#include <init.h>
#include <console.h>
#include <kmutex.h>

static int dev_count = 0;
static int master_count = 0;
static struct list_node usart_device_list;
static struct list_node usart_master_list;


int usart_read(struct device *dev, unsigned char *buff, unsigned int size)
{
	int ret;
	struct usart_device *usart = container_of(dev, struct usart_device, dev);

	verbose_printk("reading from usart !\n");

	ret = usart->master->usart_ops->read(usart, buff, size);

	return ret;
}

int usart_write(struct device *dev, unsigned char *buff, unsigned int size)
{
	int ret;
	struct usart_device *usart = container_of(dev, struct usart_device, dev);

	verbose_printk("writing from usart !\n");

	ret = usart->master->usart_ops->write(usart, buff, size);

	return ret;
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

	snprintf(usart->dev.name, sizeof(usart->dev.name), "/dev/tty%d", dev_count);

	usart->dev.read = usart_read;
	usart->dev.write = usart_write;

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

	kmutex_init(&usart->usart_mutex);

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
