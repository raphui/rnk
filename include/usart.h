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
#include <stdio.h>

struct usart {
	unsigned int num;
	unsigned int base_reg;
	unsigned int baud_rate;
	struct device dev;
};

int usart_init(unsigned int num, unsigned int base_reg, unsigned int baud_rate);
int usart_read(struct device *dev, unsigned char *buff, unsigned int size);
int usart_write(struct device *dev, unsigned char *buff, unsigned int size);

void usart_print(unsigned char byte);
int usart_printl(const char *string);

#endif /* USART_H */
