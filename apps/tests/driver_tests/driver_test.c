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

#define SPI_DEVICE	"/dev/spi6"

#ifdef CONFIG_STM32F429

#define SDRAM_ADDRESS		(unsigned int *)0xC0000000
#define SDRAM_TEST_PATTERN	0xDEADBEEF

#endif /* CONFIG_STM32F429 */

void thread_a(void)
{
	int fd;
	int ret;
	char buff[16] = {0x1, 0x2, 0x3, 0x4,
			0x5, 0x6, 0x7, 0x8,
			0x9, 0xA, 0xB, 0xC,
			0xD, 0xE, 0xF, 0x10};

	while (1) {
#ifdef CONFIG_STM32F429
		printk("Starting SDRAM tests\n");

		printk("Writing: 0x%x at 0x%x\n", SDRAM_TEST_PATTERN, SDRAM_ADDRESS);

		*(SDRAM_ADDRESS) = SDRAM_TEST_PATTERN;

		printk("Reading content at 0x%x\n", SDRAM_ADDRESS);

		ret = *(SDRAM_ADDRESS);

		printk("SDRAM test: ");

		if (ret == SDRAM_TEST_PATTERN)
			printk("OK\n");
		else
			printk("NOK\n");
#endif /* CONFIG_STM32F429 */

		printk("Starting spi tests\n");
		printk("Validate using your logical analyser\n");

		fd = open(SPI_DEVICE, O_RDWR);
		if (fd < 0) {
			error_printk("failed to open: %s\n", SPI_DEVICE);
			break;
		}

		ret = write(fd, buff, 16);
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
