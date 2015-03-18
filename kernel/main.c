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

struct mutex mutex;

void first_task(void)
{

	printk("starting task A\r\n");
	while (1) {
		mutex_lock(&mutex);
		printk("A");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		mutex_unlock(&mutex);
	}
}

void second_task(void)
{
	printk("starting task B\r\n");
	while (1) {
		mutex_lock(&mutex);
		printk("B");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		printk(".");
		mutex_unlock(&mutex);
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

int main(void)
{
	uart_init();
	pio_set_output(AT91C_BASE_PIOA, (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3), 0);
	pio_clear_value(AT91C_BASE_PIOA, (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3));

	printk("Welcome to RNK ( Raphio new kernel )\r\n");
	printk("- Initialise heap...\r\n");

	init_heap();
	init_mutex(&mutex);

	printk("- Add task to scheduler\r\n");

	add_task(&first_task, 1);
	add_task(&second_task, 6);
//	add_task(&third_task, 20);
//	add_task(&fourth_task, 20);

	aic_disable_it(AT91C_ID_PIOA);
	aic_register_handler(AT91C_ID_PIOA, AT91C_AIC_PRIOR_LOWEST, pio_isr);
	aic_enable_it(AT91C_ID_PIOA);

	pio_set_input(AT91C_BASE_PIOA, (1 << 14) | (1 << 15) | (1 << 19) | (1 << 20), 1, 1);
	pio_enable_interrupt(AT91C_BASE_PIOA, (1 << 14) | (1 << 15) | (1 << 19) | (1 << 20));

	printk("- Start scheduling...\r\n");
	start_schedule();

	while(1)
		;

	return 0; //Never reach
}
