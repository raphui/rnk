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

#include <stdio.h>
#include <scheduler.h>
#include <thread.h>
#include <board.h>
#include <pio.h>
#include <queue.h>
#include <semaphore.h>

static struct queue queue;
static struct semaphore sem;

void main_thread(void)
{
	int b = 0;

	while (1) {
		queue_receive(&queue, &b, 10000);
		if (b == 1)
			printk("X");
		else if (b == 2)
			printk("Y");
		else
			printk(".");
	}
}

void wakeup_button(void)
{
	int a = 1;

	printk("wakeup !\n");
	svc_queue_post(&queue, &a);
}

void user_button(void)
{
	int a = 2;

	printk("user !\n");
	svc_queue_post(&queue, &a);
}

int test(void)
{
	printk("Starting app_button\n");

	init_queue(&queue, 1, sizeof(int));

	add_thread(&main_thread, 30);

	/* Configure wakeup button */
	pio_set_input(GPIOA_BASE, 0, 0, 0);
	pio_request_interrupt(GPIOA_BASE, 0, &wakeup_button, IRQF_FALLING, NULL);

	/* Configure user button */
//	pio_set_input(GPIOG_BASE, 15, 0, 0);
//	irq_request(15, &user_button, IRQF_FALLING, GPIOG_BASE);

	return 0;
}
