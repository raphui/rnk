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

#include <board.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <mm.h>
#include <irq.h>
#include <armv7m/vector.h>
#include <init.h>

int irq_action(void)
{
	int ret = 0;
	unsigned char irq;
	struct isr_entry *entry = NULL;

	irq = vector_current_irq();

	entry = vector_get_isr_entry(irq);
	if (!entry) {
		error_printk("failed to retrieve isr entry in vector table\n");
		return -ENXIO;
	}

	if (entry->isr)
		entry->isr(entry->arg);

	return ret;	
}

int irq_request(unsigned int irq, void (*handler)(void *), void *arg)
{
	int ret = 0;
	struct isr_entry entry;

	entry.isr = handler;
	entry.arg = arg;

	ret = vector_set_isr_entry(&entry, irq);
	if (ret < 0)
		error_printk("failed to set isr entry, for irq: %d\n", irq);

	return ret;
}

int irq_init(void)
{
	int ret = 0;
	struct irq *irq = NULL;

	irq = (struct irq *)kmalloc(sizeof(struct irq));
	if (!irq) {
		error_printk("cannot allocate irq\n");
		return -ENOMEM;
	}

	irq->num_line = CONFIG_NUM_IRQS;

	ret = device_register(&irq->dev);
	if (ret < 0) {
		error_printk("failed to register device\n");
		ret = -ENOMEM;
		goto failed_out;
	}

	vector_set_isr_wrapper(&irq_action);

	return ret;

failed_out:
	kfree(irq);
	return ret;
}
coredevice_initcall(irq_init);
