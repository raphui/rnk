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
#include <mutex.h>

static struct mutex mutex;

void thread_a(void)
{
	printk("starting thread A\n");

	while (1) {
		printk("[A] locking mutex\n");
		mutex_lock(&mutex);
		printk("[A] unlocking mutex\n");
		mutex_unlock(&mutex);
	}
}

void thread_b(void)
{
	printk("starting thread B\n");

	while (1) {
		printk("[B] locking mutex\n");
		mutex_lock(&mutex);
		printk("[B] unlocking mutex\n");
		mutex_unlock(&mutex);
	}
}

int main(void)
{
	printk("Starting mutex tests\n");

	init_mutex(&mutex);

	printk("- adding thread A (%x)\n", &thread_a);
	add_thread(&thread_a, 10);

	printk("- adding thread B(%x)\n", &thread_b);
	add_thread(&thread_b, 1);
}
