#ifndef USB_CDC_H
#define USB_CDC_H

#include <stdint.h>
#include <drv/device.h>
#include <kernel/ksem.h>

#include <usb.h>
#include <usb_cdc.h>
#include <usb_std.h>
#include <usbd_core.h>

struct usb_cdc {
	struct device dev;
	struct device *usb_device;
	usbd_device *usb;
	unsigned char *user_buffer;
	struct semaphore read_sem;
	int len;
	int offset;
	char internal_buff[64];
	int internal_offset;
};

#endif /* USB_CDC_H */
