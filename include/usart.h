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

#ifndef USART_H
#define USART_H

#include <device.h>
#include <printk.h>
#include <kmutex.h>

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
