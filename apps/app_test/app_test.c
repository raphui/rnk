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

#include <stdio.h>
#include <scheduler.h>
#include <thread.h>
#include <unistd.h>

void app_thread(void)
{
	while (1) {
		printk("Z");
	}
}

int main(void)
{
	int fd;
	char s[] = "test from write function\n";

	printk("Hello world from app test !\n");

	fd = open("/dev/tty", O_WRONLY);
	if (fd < 0)
		error_printk("failed to open fd: /dev/tty0, error: %d\n", fd);
	else
		write(fd, s, sizeof(s));

	add_thread(&app_thread, 30);

	return 0;
}
