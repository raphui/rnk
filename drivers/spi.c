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

#include <stdio.h>
#include <spi.h>
#include <mm.h>
#include <errno.h>
#include <string.h>
#include <utils.h>
#include <init.h>
#include <fdtparse.h>
#include <pio.h>

static int dev_count = 0;
static int master_count = 0;
static char *dev_prefix = "/dev/spi";
static struct list_node spi_device_list;
static struct list_node spi_master_list;

static int spi_write(struct device *dev, unsigned char *buff, unsigned int size)
{
	int ret;
	struct spi_device *spi = container_of(dev, struct spi_device, dev);

	verbose_printk("writing from spi !\n");

	kmutex_lock(&spi->master->spi_mutex);

	pio_clear_value(spi->cs_port, spi->cs_pin);

	ret = spi->master->spi_ops->write(spi, buff, size);

	pio_set_value(spi->cs_port, spi->cs_pin);

	kmutex_unlock(&spi->master->spi_mutex);

	return ret;
}

static int spi_read(struct device *dev, unsigned char *buff, unsigned int size)
{
	int ret;
	struct spi_device *spi = container_of(dev, struct spi_device, dev);

	verbose_printk("reading from spi !\n");

	kmutex_lock(&spi->master->spi_mutex);

	pio_clear_value(spi->cs_port, spi->cs_pin);

	ret = spi->master->spi_ops->read(spi, buff, size);

	pio_set_value(spi->cs_port, spi->cs_pin);

	kmutex_unlock(&spi->master->spi_mutex);

	return ret;
}

int spi_transfer(struct spi_device *spi, unsigned char *buff, unsigned int size, int direction)
{
	int ret = 0;

	verbose_printk("spi %s transfer\n", (direction == SPI_TRANSFER_READ) ? "read" : "write");

	kmutex_lock(&spi->master->spi_mutex);

	pio_clear_value(spi->cs_port, spi->cs_pin);

	if (direction == SPI_TRANSFER_READ)
		ret = spi_read(&spi->dev, buff, size);
	else if (direction == SPI_TRANSFER_WRITE)
		ret = spi_write(&spi->dev, buff, size);
	else {
		error_printk("invalid spi transfer direction\n");
		ret = -EINVAL;
	}

	pio_set_value(spi->cs_port, spi->cs_pin);

	kmutex_unlock(&spi->master->spi_mutex);

	return ret;
}

struct spi_device *spi_new_device(void)
{
	struct spi_device *spidev = NULL;

	spidev = (struct spi_device *)kmalloc(sizeof(struct spi_device));
	if (!spidev) {
		error_printk("cannot allocate spi device\n");
		return NULL;
	}

	memset(spidev, 0, sizeof(struct spi_device));

	sem_init(&spidev->sem, 1);

	dev_count++;

	return spidev;
}

struct spi_device *spi_new_device_with_master(int fdt_offset)
{
	int ret;
	int parent_offset;
	char *fdt_path = NULL;
	struct device *dev;
	struct spi_master *spi;
	struct spi_device *spidev = NULL;
	const void *fdt_blob = fdtparse_get_blob();

	spidev = spi_new_device();
	if (!spidev) {
		error_printk("cannot allocate spi device\n");
		goto err;
	}

	parent_offset = fdt_parent_offset(fdt_blob, fdt_offset);
	if (parent_offset < 0) {
		error_printk("failed to retrive spi master for this spi device\n");
		goto err;
	}

	fdt_path = fdtparse_get_path(parent_offset);
	if (!fdt_path) {
		error_printk("failed to fdt path of spi device parent node\n");
		goto err;
	}

	dev = device_from_of_path((const char *)fdt_path);
	if (!dev) {
		error_printk("failed to find device with fdt path: %s\n", fdt_path);
		goto err;
	}

	spi = container_of(dev, struct spi_master, dev);

	spidev->master = spi;

	return spidev;

err:
	return NULL;
}

int spi_remove_device(struct spi_device *spi)
{
	int ret = 0;
	struct spi_device *spidev = NULL;

	ret = device_unregister(&spi->dev);
	if (ret < 0) {
		error_printk("failed to unregister spi device");
		return ret;
	}

	list_for_every_entry(&spi_device_list, spidev, struct spi_device, node)
		if (spidev == spi)
			break;

	if (spidev) {
		list_delete(&spidev->node);
		kfree(spidev);
	}
	else
		ret = -ENOENT;


	dev_count--;

	return ret;
}

int spi_register_device(struct spi_device *spi)
{
	int ret = 0;
	char *tmp = NULL;
	int size = strlen(dev_prefix) + 2;

	tmp = (char *)kmalloc(size);

	memset(tmp, 0, size);

	memcpy(tmp, dev_prefix, strlen(dev_prefix));
	
	/* XXX: ascii 0 start at 0x30 */
	tmp[strlen(dev_prefix)] = 0x30 + dev_count;
	tmp[strlen(dev_prefix) + 1] = 0;

	memcpy(spi->dev.name, tmp, strlen(tmp));

	spi->dev.read = spi_read;
	spi->dev.write = spi_write;

	list_add_tail(&spi_device_list, &spi->node);

	ret = device_register(&spi->dev);
	if (ret < 0)
		error_printk("failed to register spi device\n");

	kfree(tmp);

	return ret;
}

struct spi_master *spi_new_master(void)
{
	struct spi_master *spi = NULL;

	spi = (struct spi_master *)kmalloc(sizeof(struct spi_master));
	if (!spi) {
		error_printk("cannot allocate spi master\n");
		return NULL;
	}

	memset(spi, 0, sizeof(struct spi_master));

	master_count++;

	return spi;
}

int spi_remove_master(struct spi_master *spi)
{
	int ret = 0;
	struct spi_master *spidev = NULL;

	ret = device_unregister(&spi->dev);
	if (ret < 0) {
		error_printk("failed to unregister spi master");
		return ret;
	}

	list_for_every_entry(&spi_master_list, spidev, struct spi_master, node)
		if (spidev == spi)
			break;

	if (spidev) {
		list_delete(&spidev->node);
		kfree(spidev);
	}
	else
		ret = -ENOENT;


	master_count--;

	return ret;
}

int spi_register_master(struct spi_master *spi)
{
	int ret = 0;

	kmutex_init(&spi->spi_mutex);

	list_add_tail(&spi_master_list, &spi->node);

	ret = device_register(&spi->dev);
	if (ret < 0)
		error_printk("failed to register spi device\n");

	return ret;
}

int spi_init(void)
{
	int ret = 0;

	list_initialize(&spi_device_list);
	list_initialize(&spi_master_list);

	return ret;
}
postcore_initcall(spi_init);
