/*
 * Copyright (C) 2017  Raphaël Poggi <poggi.raph@gmail.com>
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

#include <stdio.h>
#include <spi.h>
#include <mm.h>
#include <errno.h>
#include <string.h>
#include <utils.h>
#include <init.h>
#include <fdtparse.h>
#include <spidev.h>
#include <pio.h>

static int spidev_of_init(struct spidev_device *spi)
{
	int offset;
	int ret = 0;
	const void *fdt_blob = fdtparse_get_blob();

	offset = fdt_path_offset(fdt_blob, spi->dev.of_path);
	if (offset < 0) {
		ret = -ENOENT;
		goto out;
	}

	ret = pio_of_configure(offset);

out:
	return ret;
}

int spidev_init(struct device *dev)
{
	int offset;
	int ret = 0;
	struct spidev_device *spidev = NULL;
	struct spi_device *spi = NULL;
	const void *fdt_blob = fdtparse_get_blob();

	spidev = (struct spidev_device *)kmalloc(sizeof(struct spidev_device));
	if (!spidev) {
		error_printk("cannot allocate spidev device\n");
		return -ENOMEM;
	}

	memcpy(&spidev->dev, dev, sizeof(struct device));

	offset = fdt_path_offset(fdt_blob, spidev->dev.of_path);
	if (offset < 0) {
		ret = -ENOENT;
		goto free_spidev;
	}

	spi = spi_new_device_with_master(offset);
	if (!spi) {
		error_printk("failed to retrive new spi device\n");
		ret = -EIO;
		goto free_spidev;
	}

	ret = spi_register_device(spi);
	if (ret < 0) {
		error_printk("failed to register spi device\n");
		goto free_spi;
	}

	spidev->spi = spi;

	return 0;

free_spi:
	kfree(spi);
free_spidev:
	kfree(spidev);
	return ret;
}

struct device spidev_driver = {
	.of_compat = "spi,spidev",
	.probe = spidev_init,
};

static int spidev_register(void)
{
	int ret = 0;

	ret = device_of_register(&spidev_driver);
	if (ret < 0)
		error_printk("failed to register spidev device\n");
	return ret;
}
coredevice_initcall(spidev_register);
