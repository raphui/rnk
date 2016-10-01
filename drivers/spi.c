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
static struct list_node spi_device_list;

static int spi_write(struct device *dev, unsigned char *buff, unsigned int size)
{
	struct spi *spi = container_of(dev, struct spi, dev);

	verbose_printk("writing from spi !\n");

	return spi_ops.write(spi, buff, size);
}

static int spi_read(struct device *dev, unsigned char *buff, unsigned int size)
{
	struct spi *spi = container_of(dev, struct spi, dev);

	verbose_printk("reading from spi !\n");

	return spi_ops.read(spi, buff, size);
}

struct spi *spi_new_device(void)
{
	struct spi *spidev = NULL;

	spidev = (struct spi *)kmalloc(sizeof(struct spi));
	if (!spidev) {
		error_printk("cannot allocate spi device\n");
		return NULL;
	}

	dev_count++;

	return spidev;	
}

int spi_remove_device(struct spi *spi)
{
	int ret = 0;
	struct spi *spidev = NULL;

	list_for_every_entry(&spi_device_list, spidev, struct spi, node)
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

int spi_register_device(struct spi *spi)
{
	int ret = 0;
	char tmp[10] = {0};

	memcpy(tmp, dev_prefix, sizeof(dev_prefix));
	
	/* XXX: ascii 0 start at 0x30 */
	tmp[8] = 0x30 + spi->num;

	memcpy(spi->dev.name, tmp, sizeof(tmp));

	ret = device_register(&spi->dev);
	if (ret < 0) {
		error_printk("failed to register device\n");
		ret = -ENOMEM;
		goto failed_out;
	}

	list_add_tail(&spi_device_list, &spi->node);

	return spi_ops.init(spi);

failed_out:
	/* XXX: deallocate here ? */
	kfree(spi);
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

	return ret;

failed_out:
	kfree(bus);
	return ret;
}
#ifdef CONFIG_INITCALL
coredevice_initcall(spi_init);
#endif /* CONFIG_INITCALL */
