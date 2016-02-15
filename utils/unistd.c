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

#include <board.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#define MAX_FD	8

struct device_io {
	int (*read)(unsigned char *buff, unsigned int size);
	int (*write)(unsigned char *buff, unsigned int size);
	int perm;
};

static devs[MAX_FD];

static int fd = 0;

int open(const char *path, int flags)
{
	int ret = fd;

	if (fd < MAX_FD) {
		if (!strcmp(path, "dev/tty")) {

//			devs[fd].read = usart_ops.read;
//			devs[fd].write = usart_ops.write;

		} else if (!strcmp(path, "dev/spi")) {

//			devs[fd].read = spi_ops.read;
//			devs[fd].write = spi_ops.write;

		} else {
			debug_printk("invalid open path\n");
			ret = -ENOENT;
		}


	} else {
		debug_printk("not more file descriptor left\n");
		ret = -EMFILE;
	}

	return ret;
}

int close(int fd)
{
	int ret = 0;

	return ret;
}

int write(int fd, const void *buf, size_t size)
{
	int ret = 0;

	return ret;
}

int read(int fd, void *buf, size_t size)
{
	int ret = 0;

	return ret;
}
