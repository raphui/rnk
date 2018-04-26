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
#include <mqueue.h>

static mqd_t mqueue;

void thread_a(void *arg)
{
	int a;

	printf("starting thread A\n");

	while (1) {
		printf("[A] receiving from queue: ");
		//queue_receive(&queue, &a, 10000);
		printf("%d\n", a);
	}
}

void thread_b(void *arg)
{
	int b = 1;

	printf("starting thread B\n");

	while (1) {
		printf("[B] posting from queue: ");
		//queue_post(&queue, &b, 1000);
		printf("%d\n", b);
		b++;
	}
}

int main(void)
{
	int ret;
	int size = 4;
	int item_size = sizeof(int);

	printf("Starting queue tests\n");

	printf("- init queue with size of %d and item_size of %d\n", size, item_size);

	ret = mq_open("test", 0, 0, 0);
	printf("mqueue: %d\n", ret);

	printf("- adding thread A (%x)\n", &thread_a);
	pthread_create(&thread_a, NULL, 4);

	printf("- adding thread B(%x)\n", &thread_b);
	pthread_create(&thread_b, NULL, 3);

	return 0;
}
