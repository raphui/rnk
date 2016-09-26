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

#include <board.h>
#include <timer.h>
#include <errno.h>
#include <mm.h>
#include <string.h>
#include <bitops.h>
#include <mutex.h>
#include <device.h>
#include <init.h>

static unsigned int timer_bitmap = 0;
static unsigned int timer_mask = 0;
static struct timer *timer_list[CONFIG_TIMER_NB];
static struct mutex timer_mutex;

void timer_set_rate(struct timer *timer, unsigned long rate)
{
	tim_ops.set_rate(timer, rate);
}

void timer_set_counter(struct timer *timer, unsigned short counter)
{
	tim_ops.set_counter(timer, counter);
}

void timer_enable(struct timer *timer)
{
	tim_ops.enable(timer);
}

void timer_disable(struct timer *timer)
{
	tim_ops.disable(timer);
}

void timer_clear_it_flags(struct timer *timer, unsigned int flags)
{
	tim_ops.clear_it_flags(timer, flags);
}

static int timer_request(void)
{
	int i;
	int ret;
	struct timer *timer = NULL;

	timer = (struct timer *)kmalloc(sizeof(struct timer));
	if (!timer) {
		error_printk("failed to allocate timer");
		return -ENOMEM;
	}

	i = ffz(timer_bitmap & timer_mask);
	if (i > CONFIG_TIMER_NB) {
		error_printk("all timers are allocate\n");
		ret = -EAGAIN;
		goto failed_out;
	}

	timer_list[i] = timer;

	timer_bitmap |= (1 << i);

	timer->num = i;

	ret = tim_ops.init(timer);
	if (ret < 0) {
		printk("failed to init timer via hardware IP\n");
		goto failed_out;
	}

	return i;

failed_out:
	kfree(timer);
	return ret;
}

static int timer_release(struct timer *timer)
{
	int ret = 0;

	mutex_lock(&timer_mutex);

	ret = tim_ops.release_irq(timer);
	if (ret < 0) {
		error_printk("failed to release timer via hardware IP\n");
		goto failed;
	}

	timer_bitmap &= ~(timer->num);
	
	mutex_unlock(&timer_mutex);

	kfree(timer_list[timer->num]);

failed:
	return 0;
}

static int timer_release_from_isr(struct timer *timer)
{
	int ret = 0;

	ret = tim_ops.release_irq(timer);
	if (ret < 0) {
		error_printk("failed to release timer via hardware IP\n");
		goto failed;
	}

	timer_bitmap &= ~(timer->num);
	
	kfree(timer_list[timer->num]);

failed:
	return 0;
}

static void timer_isr(void *arg)
{
	struct timer *timer = (struct timer *)arg;
	void (*callback)(void *) = timer->callback.handler;
	void *callback_arg = timer->callback.arg;

	callback(callback_arg);

	if (timer->one_shot) {
		timer_disable(timer);
		timer_release_from_isr(timer);
	}
}

int timer_oneshot(unsigned int delay, void (*handler)(void *), void *arg)
{
	int ret = 0;
	struct timer *timer = NULL;

	mutex_lock(&timer_mutex);

	ret = timer_request();
	if (ret < 0) {
		error_printk("failed to request timer\n");
		return ret;
	}

	timer = timer_list[ret];

	timer->one_shot = 1;
	timer->one_pulse = 1;
	timer->count_up = 0;
	timer->counter = delay;
	timer->callback.handler = handler;
	timer->callback.arg = arg;

	tim_ops.request_irq(timer, &timer_isr, timer);

	timer_enable(timer);

	mutex_unlock(&timer_mutex);

	return 0;
}

int timer_init(void)
{
	int ret = 0;
	struct timer_device *timer_dev = NULL;

	timer_dev = (struct timer_device *)kmalloc(sizeof(struct timer_device));
	if (!timer_dev) {
		error_printk("cannot allocate timer_device\n");
		return -ENOMEM;
	}

	ret = device_register(&timer_dev->dev);
	if (ret < 0) {
		error_printk("failed to register device\n");
		ret = -ENOMEM;
		goto failed_out;
	}

	timer_mask ^= (-1 ^ timer_mask) & (1 << CONFIG_TIMER_NB);

	mutex_init(&timer_mutex);

	return ret;

failed_out:
	kfree(timer_dev);
	return ret;
}
#ifdef CONFIG_INITCALL
device_initcall(timer_init);
#endif /* CONFIG_INITCALL */
