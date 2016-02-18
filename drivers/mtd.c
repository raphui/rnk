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
 * along with this program; if not, write to the Frrestore * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <board.h>
#include <utils.h>
#include <mtd.h>
#include <mm.h>
#include <errno.h>
#include <string.h>

static int dev_count = 0;
static char dev_prefix[10] = "/dev/mtd";

int mtd_init(struct mtd *mtd)
{
	int ret = 0;
	struct mtd *_mtd = NULL;

	_mtd = (struct mtd *)kmalloc(sizeof(struct mtd));
	if (_mtd < 0) {
		error_printk("cannot allocate mtd\n");
		return -ENOMEM;
	}

	memcpy(_mtd, mtd, sizeof(struct mtd));

	_mtd->dev.read = mtd_read;
	_mtd->dev.write = mtd_write;

	ret = device_register(&_mtd->dev);
	if (ret < 0) {
		error_printk("failed to register device\n");
		ret = -ENOMEM;
		goto failed_out;
	}

	return ret;

failed_out:
	kfree(mtd);
	return ret;
}

int mtd_read(struct device *dev, unsigned int addr)
{
	struct mtd *mtd = container_of(dev, struct mtd, dev);

	debug_printk("reading from mtd !\n");
}

int mtd_write(struct device *dev, unsigned int addr, unsigned data)
{
	struct mtd *mtd = container_of(dev, struct mtd, dev);

	debug_printk("writing from mtd !\n");

}
