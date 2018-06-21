#ifndef CONSOLE_H
#define CONSOLE_H

#include <device.h>

#ifdef CONFIG_USART_DEBUG
#include <usart.h>
#endif

#ifndef CONFIG_USART_DEBUG
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
#endif
	void *pdata;
#ifndef CONFIG_USART_DEBUG
	struct io_operations *io_ops;
#endif
};

int console_write(unsigned char *buff, unsigned int len);


#endif /* CONSOLE_H */
