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
#include <mutex.h>

#define SPI_TRANSFER_READ	0
#define SPI_TRANSFER_WRITE	1

struct spi_device {
	struct spi_master *master;
	unsigned int cs;
	unsigned int speed;
	struct device dev;
	struct list_node node;
};

struct spi_operations
{
	int (*write)(struct spi_device *spi, unsigned char *buff, unsigned int size);
	int (*read)(struct spi_device *spi, unsigned char *buff, unsigned int size);
};

struct spi_master {
	unsigned int num;
	unsigned int base_reg;
	unsigned int source_clk;
	unsigned int rate;	/* current rate */
	unsigned int speed;	/* wanted speed */
	unsigned int irq;
	unsigned short mode;
	struct dma_transfer dma_trans;
	unsigned char only_tx;
	unsigned char only_rx;
	unsigned char use_dma;
	struct mutex spi_mutex;
	struct list_node node;
	struct device dev;
	struct dma_stream dma_stream[2];
	struct spi_operations *spi_ops;
};

int spi_transfer(struct spi_device *spi, unsigned char *buff, unsigned int size, int direction);
struct spi_device *spi_new_device_with_master(int fdt_offset);
struct spi_device *spi_new_device(void);
int spi_remove_device(struct spi_device *spi);
int spi_register_device(struct spi_device *spi);
struct spi_master *spi_new_master(void);
int spi_remove_master(struct spi_master *spi);
int spi_register_master(struct spi_master *spi);
int spi_init(void);

#endif /* SPI_H */
