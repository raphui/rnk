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

#include <board.h>
#include <utils.h>
#include <usart.h>
#include <mm.h>
#include <errno.h>
#include <string.h>

static struct usart *usart;
static int dev_count = 0;
static char dev_prefix[10] = "/dev/tty";

int usart_init(unsigned int num, unsigned int base_reg, unsigned int baud_rate)
{
	int ret = 0;

	usart = (struct usart *)kmalloc(sizeof(*usart));
	if (usart < 0) {
		error_printk("cannot allocate usart\r\n");
		return -ENOMEM;
	}

	usart->num = num;
	usart->base_reg = base_reg;
	usart->baud_rate = baud_rate;

	dev_prefix[9] = '0';
	dev_prefix[10] = 0;

	memcpy(usart->dev.name, dev_prefix, 10);

	usart->dev.read = usart_read;
	usart->dev.write = usart_write;

	ret = device_register(&usart->dev);
	if (ret < 0) {
		error_printk("failed to register device\n");
		ret = -ENOMEM;
		goto failed_out;
	}

	return usart_ops.init(usart);

failed_out:
	kfree(usart);
	return ret;
}

int usart_read(struct device *dev, unsigned char *buff, unsigned int size)
{
	struct usart *usart = container_of(dev, struct usart, dev);

	printk("reading from usart !\n");

	return 0;
}

int usart_write(struct device *dev, unsigned char *buff, unsigned int size)
{
	struct usart *usart = container_of(dev, struct usart, dev);

	printk("writing from usart !\n");

	return 0;
}

void usart_print(unsigned char byte)
{
	usart_ops.print(usart, byte);
}


int usart_printl(const char *string)
{
	return usart_ops.printl(usart, string);
}


#ifdef CONFIG_USART_DEBUG

struct io_operations io_op = {
	.write = usart_print,
	.write_string = usart_printl,
};

#endif /* CONFIG_USART_DEBUG */
