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

#ifdef CONFIG_INITCALL
#include <init.h>
#endif /* CONFIG_INITCALL */

static int dev_count = 0;
static char dev_prefix[10] = "/dev/spi";
static struct list_node spi_device_list;
static struct list_node spi_master_list;

static int spi_write(struct device *dev, unsigned char *buff, unsigned int size)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);

	verbose_printk("writing from spi !\n");

	return spi->master->spi_ops->write(spi, buff, size);
}

static int spi_read(struct device *dev, unsigned char *buff, unsigned int size)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);

	verbose_printk("reading from spi !\n");

	return spi->master->spi_ops->read(spi, buff, size);
}

int spi_transfer(struct spi_device *spi, unsigned char *buff, unsigned int size, int direction)
{
	int ret = 0;

	verbose_printk("spi %s transfer\n", (direction == SPI_TRANSFER_READ) ? "read" : "write");

	if (direction == SPI_TRANSFER_READ)
		ret = spi_read(&spi->dev, buff, size);
	else if (direction == SPI_TRANSFER_WRITE)
		ret = spi_write(&spi->dev, buff, size);
	else {
		error_printk("invalid spi transfer direction\n");
		ret = -EINVAL;
	}

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

	dev_count++;

	return spidev;
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
	char tmp[10] = {0};

	memcpy(tmp, dev_prefix, sizeof(dev_prefix));
	
	/* XXX: ascii 0 start at 0x30 */
	tmp[8] = 0x30 + dev_count;

	memcpy(spi->dev.name, tmp, sizeof(tmp));

	list_add_tail(&spi_device_list, &spi->node);

	ret = device_register(&spi->dev);
	if (ret < 0)
		error_printk("failed to register spi device\n");

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


	dev_count--;

	return ret;
}

int spi_register_master(struct spi_master *spi)
{
	int ret = 0;
	char tmp[10] = {0};

	memcpy(tmp, dev_prefix, sizeof(dev_prefix));

	/* XXX: ascii 0 start at 0x30 */
	tmp[8] = 0x30 + spi->num;

	memcpy(spi->dev.name, tmp, sizeof(tmp));

	list_add_tail(&spi_master_list, &spi->node);

	ret = device_register(&spi->dev);
	if (ret < 0)
		error_printk("failed to register spi device\n");

	return ret;
}

int spi_init(void)
{
	int ret = 0;
	struct spi_bus *bus = NULL;

	bus = (struct spi_bus *)kmalloc(sizeof(struct spi_bus));
	if (!bus) {
		error_printk("cannot allocate spi bus\n");
		return -ENOMEM;
	}

	bus->dev.read = spi_read;
	bus->dev.write = spi_write;

	ret = device_register(&bus->dev);
	if (ret < 0) {
		error_printk("failed to register bus\n");
		ret = -ENOMEM;
		goto failed_out;
	}

	list_initialize(&spi_device_list);
	list_initialize(&spi_master_list);

	return ret;

failed_out:
	kfree(bus);
	return ret;
}
#ifdef CONFIG_INITCALL
postcore_initcall(spi_init);
#endif /* CONFIG_INITCALL */
