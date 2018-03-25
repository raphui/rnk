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
#include <thread.h>
#include <time.h>

void thread_a(void)
{
	int i = 0;

	printk("starting thread A\n");

	while (1) {
		if (i++ % 10)
			usleep(1000000);

		printk("A\n");
	}
}

void thread_b(void)
{
	int i = 0;

	printk("starting thread B\n");

	while (1) {
		if (i++ % 10)
			usleep(1000000);

		printk("B\n");
	}
}

void thread_c(void)
{
	int i = 0;

	printk("starting thread C\n");

	while (1) {
		if (i++ % 10)
			usleep(1000000);

		printk("C\n");
	}
}

int main(void)
{
	printk("Starting thread tests\n");

	printk("- adding thread A (%x)\n", &thread_a);
	add_thread(&thread_a, HIGHEST_PRIORITY);

	printk("- adding thread B(%x)\n", &thread_b);
	add_thread(&thread_b, HIGHEST_PRIORITY - 1);

	printk("- adding thread C(%x)\n", &thread_c);
	add_thread(&thread_c, HIGHEST_PRIORITY - 2);

	return 0;
}
