#ifndef CONSOLE_H
#define CONSOLE_H

#include <drv/device.h>

#ifdef CONFIG_USART_DEBUG
#include <drv/usart.h>
#elif defined(CONFIG_USB_DEBUG)
#include <drv/usb_cdc.h>
#endif

#if !defined(CONFIG_USART_DEBUG) && !defined(CONFIG_USB_DEBUG)
struct io_operations
{
	int (*write)(struct device *dev, unsigned char *buff, unsigned int len);
};

extern struct io_operations io_op;
#endif

struct console
{
#ifdef CONFIG_USART_DEBUG
	struct usart_device *usart;
#elif defined(CONFIG_USB_DEBUG)
	struct usb_cdc *usb;
	uint8_t buf[256];
	int idx;
#endif
	void *pdata;
#if !defined(CONFIG_USART_DEBUG) && !defined(CONFIG_USB_DEBUG)
	struct io_operations *io_ops;
#endif
};

int console_write(unsigned char *buff, unsigned int len);


#endif /* CONSOLE_H */
