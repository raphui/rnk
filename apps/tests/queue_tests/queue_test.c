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
#include <queue.h>

static struct queue queue;

void thread_a(void)
{
	int a;

	printk("starting thread A\n");

	while (1) {
		printk("[A] receiving from queue: ");
		queue_receive(&queue, &a, 10000);
		printk("%d\n", a);
	}
}

void thread_b(void)
{
	int b = 1;

	printk("starting thread B\n");

	while (1) {
		printk("[B] posting from queue: ");
		queue_post(&queue, &b, 1000);
		printk("%d\n", b);
		b++;
	}
}

int main(void)
{
	int size = 4;
	int item_size = sizeof(int);

	printk("Starting queue tests\n");

	printk("- init queue with size of %d and item_size of %d\n", size, item_size);

	queue_init(&queue, size, item_size);

	printk("- adding thread A (%x)\n", &thread_a);
	add_thread(&thread_a, HIGHEST_PRIORITY);

	printk("- adding thread B(%x)\n", &thread_b);
	add_thread(&thread_b, HIGHEST_PRIORITY - 1);

	return 0;
}
