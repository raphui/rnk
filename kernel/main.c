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

#include <board.h>
#include <uart.h>
#include <stdio.h>
#include <scheduler.h>
#include <task.h>
#include <interrupt.h>
#include <pio.h>
#include <aic.h>
#include <utils.h>
#include <mm.h>
#include <mutex.h>
#include <arch/svc.h>

struct mutex mutex;

void first_task(void)
{

	printk("starting task A\r\n");
	while (1) {
		SVC_ARG(SVC_ACQUIRE_MUTEX, &mutex);
		printk("A");
		SVC_ARG(SVC_RELEASE_MUTEX, &mutex);
	}
}

static int count = 0;
void second_task(void)
{
	printk("starting task B\r\n");
	SVC_ARG(SVC_ACQUIRE_MUTEX, &mutex);
	while (1) {
		printk("B");
		if (count++ == 500)
			SVC_ARG(SVC_RELEASE_MUTEX, &mutex);
	}
}

void third_task(void)
{
	printk("starting task C\r\n");
	while (1) {
		printk("C");
	}
}

void fourth_task(void)
{
	unsigned int size = sizeof(unsigned int);
	unsigned char *p;
	unsigned int *array = (unsigned int *)kmalloc(size);
	int i = 0;

	printk("starting task D\r\n");
	printk("array (%x)\r\n", array);

	p = (unsigned char *)kmalloc(24);
	*p = 0xea;
	printk("p(%x): %x\r\n", p, *p);
	kfree(p);

	while (1) {
		for (i = 0; i < size; i++) {
			printk("D");
			array[i] = (1 << i);
			printk("%x ", array[i]);
		}

		for (i = 0; i < size; i++) {
			kfree(array);
		}
	}
}

void fifth_task(void)
{
	printk("starting task E\r\n");
	while (1) {
		printk("E");
	}
}

int main(void)
{
	uart_init();

	printk("Welcome to rnk\r\n");
	printk("- Initialise heap...\r\n");

	init_heap();

	printk("- Initialise scheduler...\r\n");
	schedule_init();

	init_mutex(&mutex);

	printk("- Add task to scheduler\r\n");

	add_task(&first_task, 1);
	add_task(&second_task, 6);
//	add_task(&third_task, 20);
//	add_task(&fourth_task, 20);
//	add_task(&fifth_task, 20);

	printk("- Start scheduling...\r\n");
	start_schedule();

	while(1)
		;

	return 0; //Never reach
}
