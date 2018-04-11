/*
 * Copyright (C) 2016  Raphaël Poggi <poggi.raph@gmail.com>
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
