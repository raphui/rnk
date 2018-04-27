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

#ifndef PIO_H
#define PIO_H

#include <device.h>

#define IRQF_RISING	1
#define IRQF_FALLING	2

struct pio_operations
{
	void (*set_output)(unsigned int port, unsigned int mask, int pull_up);
	void (*set_input)(unsigned int port, unsigned int mask, int pull_up, int filter);
	void (*set_alternate)(unsigned int port, unsigned int mask, unsigned int num);
	void (*set_value)(unsigned int port, unsigned int mask);
	void (*clear_value)(unsigned int port, unsigned int mask);
	void (*toggle_value)(unsigned int port, unsigned int mask);
	int (*request_interrupt)(unsigned int port, unsigned int mask, void (*handler)(void), int flags, void *arg);
	void (*enable_interrupt)(unsigned int port, unsigned int mask);
	void (*disable_interrupt)(unsigned int port, unsigned int mask);
	int (*of_configure)(int fdt_offset);
	int (*of_configure_name)(int fdt_offset, char *name);
	int (*of_get)(int fdt_offset, char *name, unsigned int *port, unsigned int *pin);
	int (*export_pio)(unsigned int pin_num, unsigned int *port, unsigned int *pin);
};

struct pio {
	struct pio_operations *pio_ops;
	struct device dev;
};

int pio_init(struct pio *pio);
void pio_set_output(unsigned int port, unsigned int mask, int pull_up);
void pio_set_input(unsigned int port, unsigned int mask, int pull_up, int filter);
void pio_set_alternate(unsigned int port, unsigned int mask, unsigned int num);
void pio_set_value(unsigned int port, unsigned int mask);
void pio_clear_value(unsigned int port, unsigned int mask);
void pio_toggle_value(unsigned int port, unsigned int mask);
int pio_request_interrupt(unsigned int port, unsigned int mask, void (*handler)(void), int flags, void *arg);
void pio_enable_interrupt(unsigned int port, unsigned int mask);
void pio_disable_interrupt(unsigned int port, unsigned int mask);
int pio_of_configure(int fdt_offset);
int pio_of_configure_name(int fdt_offset, char *name);
int pio_of_get(int fdt_offset, char *name, unsigned int *port, unsigned int *pin);

#endif /* PIO_H */
