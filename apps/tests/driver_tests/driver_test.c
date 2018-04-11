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
#include <board.h>
#include <unistd.h>

#define SPI_DEVICE	"/dev/spi1"

#ifdef CONFIG_STM32F429

#define SDRAM_ADDRESS		(unsigned int *)0xC0000000
#define SDRAM_TEST_PATTERN	0xDEADBEEF

#endif /* CONFIG_STM32F429 */

static char spi_buff[16] = {0x1, 0x2, 0x3, 0x4,
			0x5, 0x6, 0x7, 0x8,
			0x9, 0xA, 0xB, 0xC,
			0xD, 0xE, 0xF, 0x10};


void thread_a(void *arg)
{
	int i;
	int fd;
	int ret;

	while (1) {
#ifdef CONFIG_STM32F429
		printf("Starting SDRAM tests\n");

		printf("Writing: 0x%x at 0x%x\n", SDRAM_TEST_PATTERN, SDRAM_ADDRESS);

		*(SDRAM_ADDRESS) = SDRAM_TEST_PATTERN;

		printf("Reading content at 0x%x\n", SDRAM_ADDRESS);

		ret = *(SDRAM_ADDRESS);

		printf("SDRAM test: ");

		if (ret == SDRAM_TEST_PATTERN)
			printf("OK\n");
		else
			printf("NOK\n");
#endif /* CONFIG_STM32F429 */

		printf("Starting spi tests\n");
		printf("Validate using your logical analyser\n");

		fd = open(SPI_DEVICE, O_RDWR);
		if (fd < 0) {
			printf("failed to open: %s\n", SPI_DEVICE);
			break;
		}

		ret = write(fd, spi_buff, 16);
		if (ret != 16) {
			printf("failed to write spi buff\n");
			break;
		}

		ret = read(fd, spi_buff, 16);
		if (ret != 16) {
			printf("failed to read spi buff\n");
			break;
		}

		for (i = 0; i < 16; i++)
			printf("spi_read[%d]: 0x%x\n", spi_buff[i]);
	}
}

int main(void)
{
	printf("Starting driver tests\n");

	pthread_create(&thread_a, NULL, 2);

	return 0;

}
