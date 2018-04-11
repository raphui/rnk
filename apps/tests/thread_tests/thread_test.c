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
#include <time.h>

void thread_a(void *arg)
{
	printf("starting thread A\n");

	while (1) {
		printf("A\n");
		time_usleep(30000);
	}
}

void thread_b(void *arg)
{
	printf("starting thread B\n");

	while (1) {
		printf("B\n");
		time_usleep(10000);
	}
}

void thread_c(void *arg)
{
	printf("starting thread C\n");

	while (1) {
		printf("C\n");
	}
}

int main(void)
{
	printf("Starting thread tests\n");

	printf("- adding thread A (%x)\n", &thread_a);
	pthread_create(&thread_a, NULL, 4);

	printf("- adding thread B(%x)\n", &thread_b);
	pthread_create(&thread_b, NULL, 3);

	printf("- adding thread C(%x)\n", &thread_c);
	pthread_create(&thread_c, NULL, 2);

	return 0;
}
