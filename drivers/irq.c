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

struct action {
	void (*irq_action)(void);
	int irq;
	TAILQ_ENTRY(action) next;
};

static TAILQ_HEAD(, action) action_list;

int irq_action(unsigned int irq)
{
	int ret = 0;
	struct action *action = NULL;
	void (*hook)(void) = NULL;

	TAILQ_FOREACH(action, &action_list, next)
		if (action->irq == irq)
			break;

	if (!action) {
		error_printk("no action has been found for irq: %d\n", irq);
		return -ENOSYS;
	}

	hook = action->irq_action;

	hook();

	return ret;	
}

int irq_request(unsigned int irq, void (*handler)(void), unsigned int flags, unsigned int extra)
{
	int ret = 0;
	struct action *action = NULL;

	/* FIXME: hardcoded for exti stm32 driver, need to find a good interface instead of direct call */
	if (irq > 15) {
		error_printk("invalid line requested\n");
		ret = -EINVAL;
	}

	ret = irq_ops.request(irq, extra);
	if (ret < 0) {
		error_printk("failed to request irq: %d\n", irq);
		goto fail;
	}

	if (flags & IRQF_RISING)
		irq_ops.set_rising(irq, extra);
	else if (flags & IRQF_FALLING)
		irq_ops.set_falling(irq, extra);
	else {
		error_printk("wrong flags for irq: %d\n", irq);
		ret = -EINVAL;
		goto fail;
	}

	action = (struct action *)kmalloc(sizeof(struct action));
	if (!action) {
		error_printk("cannot allocate irq action\n");
		ret = -ENOMEM;
		goto fail;
	}

	action->irq_action = handler;
	action->irq = irq;

	if (TAILQ_EMPTY(&action_list))
		TAILQ_INSERT_HEAD(&action_list, action, next);
	else
		TAILQ_INSERT_TAIL(&action_list, action, next);

fail:
	return ret;
}

int irq_init(struct irq *irq)
{
	int ret = 0;
	struct irq *irqdev = NULL;

	irqdev = (struct irq *)kmalloc(sizeof(struct irq));
	if (!irqdev) {
		error_printk("cannot allocate irq\n");
		return -ENOMEM;
	}

	memcpy(irqdev, irq, sizeof(struct irq));

	ret = device_register(&irq->dev);
	if (ret < 0) {
		error_printk("failed to register device\n");
		ret = -ENOMEM;
		goto failed_out;
	}

	TAILQ_INIT(&action_list);

	return ret;

failed_out:
	kfree(irqdev);
	return ret;
}
