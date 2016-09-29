/*
 * Copyright (C) 2015  Raphaël Poggi <poggi.raph@gmail.com>
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

#ifndef TIMER_H
#define TIMER_H

#include <device.h>
#include <list.h>

struct timer_device
{
	struct device dev;
};

struct timer_callback {
	int delay;
	void (*handler)(void *);
	void *arg;
	struct list_node node;
};

struct timer
{
	unsigned int num;
	unsigned int base_reg;
	unsigned int one_shot;
	unsigned long rate;
	unsigned int prescaler;
	unsigned int rcc_base;
	unsigned char one_pulse:1;
	unsigned char count_up:1;
	unsigned int counter;
	struct timer_callback callback;
};

struct timer_operations
{
	int (*init)(struct timer *timer);
	void (*set_rate)(struct timer *timer, unsigned long rate);
	void (*set_counter)(struct timer *timer, unsigned short counter);
	void (*enable)(struct timer *timer);
	void (*disable)(struct timer *timer);
	void (*clear_it_flags)(struct timer *timer, unsigned int flags);
	int (*request_irq)(struct timer *timer, void (*handler)(void *), void *arg);
	int (*release_irq)(struct timer *timer);
};

int timer_init(void);
int timer_oneshot(unsigned int delay, void (*handler)(void *), void *arg);
int timer_oneshot_soft(unsigned int delay, void (*handler)(void *), void *arg);
void timer_set_rate(struct timer *timer, unsigned long rate);
void timer_set_counter(struct timer *timer, unsigned short counter);
void timer_enable(struct timer *timer);
void timer_disable(struct timer *timer);
void timer_clear_it_flags(struct timer *timer, unsigned int flags);
void timer_soft_decrease_delay(void);


#endif /* TIMER_H*/
