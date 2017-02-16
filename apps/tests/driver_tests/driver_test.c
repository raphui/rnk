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
#include <spi.h>
#include <unistd.h>

#define SPI_DEVICE	"/dev/spi"

void thread_a(void)
{
	int fd;
	int ret;
	char buff[4] = {0x1, 0x2, 0x3, 0x4};

	while (1) {
		printk("Starting spi tests\n");
		printk("Validate using your logical analyser\n");

		fd = open(SPI_DEVICE, O_RDWR);
		if (fd < 0) {
			error_printk("failed to open: %s\n", SPI_DEVICE);
			break;
		}

		ret = write(fd, buff, 4);
		if (ret != 4) {
			error_printk("failed to write spi buff\n");
			break;
		}
	}
}

int main(void)
{
	printk("Starting driver tests\n");

	add_thread(&thread_a, DEFAULT_PRIORITY);

	return 0;

}
