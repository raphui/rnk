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
#include <pthread.h>

static pthread_mutex_t mutex;

void thread_a(void *arg)
{
	printf("starting thread A\n");

	while (1) {
		printf("[A] locking mutex\n");
		pthread_mutex_lock(&mutex);
		printf("[A] unlocking mutex\n");
		pthread_mutex_unlock(&mutex);
	}
}

void thread_b(void *arg)
{
	printf("starting thread B\n");

	while (1) {
		printf("[B] locking mutex\n");
		pthread_mutex_lock(&mutex);
		printf("[B] unlocking mutex\n");
		pthread_mutex_unlock(&mutex);
	}
}

int main(void)
{
	printf("Starting mutex tests\n");

	pthread_mutex_init(&mutex);

	printf("- adding thread A (%x)\n", &thread_a);
	pthread_create(&thread_a, NULL, 10);

	printf("- adding thread B(%x)\n", &thread_b);
	pthread_create(&thread_b, NULL, 2);
}
