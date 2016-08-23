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
 * along with this program; if not, write to the Frrestore * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef IRQ_H
#define IRQ_H

#include <device.h>
#include <list.h>

#define IRQF_RISING	1
#define IRQF_FALLING	2

struct irq {
	unsigned int num_line;
	struct device dev;
};


int irq_init(struct irq *irq);
int irq_request(unsigned int irq, void (*handler)(void), unsigned int flags, unsigned int extra);
int irq_action(unsigned int irq);

struct irq_operations
{
	int (*request)(unsigned int irq, unsigned int extra);
	int (*set_rising)(unsigned int line, unsigned int extra);
	int (*set_falling)(unsigned int line, unsigned int extra);
};

#endif /* IRQ_H */
