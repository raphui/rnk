/*
 * Copyright (C) 2015  Raphaël Poggi <poggi.raph@gmail.com>
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
 * along with this program; if not, write to the Frrestore * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef SPI_H
#define SPI_H

#include <device.h>
#include <dma.h>
#include <list.h>

#define SPI_TRANSFER_READ	1
#define SPI_TRANSFER_WRITE	2

struct spi {
	unsigned int num;
	unsigned int base_reg;
	unsigned int rate;	/* current rate */
	unsigned int speed;	/* wanted speed */
	unsigned short mode;
	struct dma dma;
	struct dma_transfer dma_trans;
	unsigned char only_tx;
	unsigned char only_rx;
	unsigned char use_dma;
	struct list_node node;
	struct device dev;
};

struct spi_bus {
	struct device dev;
};

struct spi_operations
{
	int (*init)(struct spi *spi);
	int (*write)(struct spi *spi, unsigned char *buff, unsigned int size);
	int (*read)(struct spi *spi, unsigned char *buff, unsigned int size);
};

int spi_transfer(struct spi *spi, unsigned char *buff, unsigned int size, int direction);
struct spi *spi_new_device(void);
int spi_remove_device(struct spi *spi);
int spi_register_device(struct spi *spi);
int spi_init(void);

#endif /* SPI_H */
