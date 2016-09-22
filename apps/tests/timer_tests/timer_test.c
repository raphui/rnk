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
#include <board.h>
#include <pio.h>
#include <time.h>

void thread_a(void)
{
	while (1) {
		printk("Sleeping...");
		pio_set_value(GPIOA_BASE, 3);
		usleep(10000);
		pio_clear_value(GPIOA_BASE, 3);
		printk("Waking up !\n");
	}
}

int main(void)
{
	printk("Starting timer tests\n");

	pio_set_output(GPIOA_BASE, 3, 0);

	add_thread(&thread_a, DEFAULT_PRIORITY);

	return 0;
}
