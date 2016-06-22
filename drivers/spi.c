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

#include <board.h>
#include <spi.h>
#include <mm.h>
#include <errno.h>
#include <string.h>
#include <utils.h>

static int dev_count = 0;
static char dev_prefix[10] = "/dev/spi";

static int spi_write(struct device *dev, unsigned char *buff, unsigned int size)
{
	struct spi *spi = container_of(dev, struct spi, dev);

	debug_printk("writing from spi !\n");

	return spi_ops.write(spi, buff, size);
}

static int spi_read(struct device *dev, unsigned char *buff, unsigned int size)
{
	struct spi *spi = container_of(dev, struct spi, dev);

	debug_printk("reading from spi !\n");

	return spi_ops.read(spi, buff, size);
}

int spi_init(struct spi *spi)
{
	int ret = 0;
	struct spi *spidev = NULL;

	spidev = (struct spi *)kmalloc(sizeof(struct spi));
	if (!spidev) {
		error_printk("cannot allocate spi\n");
		return -ENOMEM;
	}

	memcpy(spidev, spi, sizeof(struct spi));

	spidev->dev.read = spi_read;
	spidev->dev.write = spi_write;

	ret = device_register(&spidev->dev);
	if (ret < 0) {
		error_printk("failed to register device\n");
		ret = -ENOMEM;
		goto failed_out;
	}

	return spi_ops.init(spidev);

failed_out:
	kfree(spidev);
	return ret;
}
