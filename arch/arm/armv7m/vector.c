/*
 * Copyright (C) 2016 Raphaël Poggi <poggi.raph@gmail.com>
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

#include <stdio.h>
#include <errno.h>
#include <stddef.h>
#include <armv7m/system.h>
#include <armv7m/vector.h>

static struct isr_entry sw_isr_table[CONFIG_NUM_IRQS];

static void dummy_wrapper(void)
{
	while (1)
		;
}

typedef void (*vect)(void);

vect __attribute__((__section__(".isr_vector_cmsis"))) isr_vector_table[CONFIG_NUM_IRQS] = {
	[0 ...(CONFIG_NUM_IRQS - 1)] = dummy_wrapper,
};

static inline int is_irq_valid(unsigned int irq)
{
	if ((irq > CONFIG_NUM_IRQS) || (irq < 0)) {
		error_printk("invalid irq num\n");
		return -EINVAL;
	}
	else
		return irq;
}

unsigned int vector_current_irq(void)
{
	return (unsigned int)IPSR();
}

void vector_set_isr_wrapper(void *wrapper)
{
	int i;

	for (i = 0; i < (CONFIG_NUM_IRQS - 1); i++)
		isr_vector_table[i] = wrapper;
}

int vector_set_isr_entry(struct isr_entry *entry, unsigned int irq)
{
	int ret = 0;

	ret = is_irq_valid(irq);
	if (ret < 0) {
		error_printk("irq num is not valid\n");
		return ret;
	}

	sw_isr_table[ret].isr = entry->isr;
	sw_isr_table[ret].arg = entry->arg;

	return ret;
}

struct isr_entry *vector_get_isr_entry(unsigned int irq)
{
	int ret;
	int idx = 0;
	struct isr_entry *entry = NULL;

	ret = is_irq_valid(irq);
	if (ret < 0) {
		error_printk("irq num is not valid\n");
		return NULL;
	}

	entry = &sw_isr_table[idx];

	return entry;
}
