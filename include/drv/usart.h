#ifndef USART_H
#define USART_H

#include <drv/device.h>
#include <kernel/printk.h>
#include <kernel/kmutex.h>

struct usart_master;

struct usart_device {
	unsigned int num;
	struct usart_master *master;
	struct device dev;
	struct list_node node;
};

struct usart_operations
{
	int (*read)(struct usart_device *usart, unsigned char *buff, unsigned int len);
	int (*write)(struct usart_device *usart, unsigned char *buff, unsigned int len);
	void (*print)(struct usart_master *usart, unsigned char byte);
	int (*printl)(struct usart_master *usart, const char *string);
};

struct usart_master {
	unsigned int num;
	unsigned int base_reg;
	unsigned int source_clk;
	unsigned int baud_rate;
	unsigned int mode;
	struct mutex usart_mutex;
	struct list_node node;
	struct device dev;
	struct usart_operations *usart_ops;
};

struct usart_bus {
	struct device dev;
};

int usart_write(struct device *dev, unsigned char *buff, unsigned int size);
int usart_read(struct device *dev, unsigned char *buff, unsigned int size);
struct usart_device *usart_new_device(void);
int usart_remove_device(struct usart_device *usart);
int usart_register_device(struct usart_device *usart);
struct usart_master *usart_new_master(void);
int usart_remove_master(struct usart_master *usart);
int usart_register_master(struct usart_master *usart);
int usart_init(void);

#endif /* USART_H */
