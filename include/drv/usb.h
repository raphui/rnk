#ifndef USB_H
#define USB_H

#include <drv/device.h>
#include <drv/clk.h>

struct usb_device;

struct usb_operations
{
	int (*write)(struct usb_device *usb, unsigned char *buff, unsigned int size);
	int (*read)(struct usb_device *usb, unsigned char *buff, unsigned int size);
	int (*ioctl)(struct usb_device *usb, int request, char *arg);
};

struct usb_device {
	unsigned int base_reg;
	struct clk clock;
	unsigned int irq;
	void *priv;
	struct device dev;
	struct list_node node;
	struct usb_operations *usb_ops;
};


struct usb_device *usb_new_device(void);
int usb_remove_device(struct usb_device *usb);
int usb_register_device(struct usb_device *usb);

#endif /* USB_H */
