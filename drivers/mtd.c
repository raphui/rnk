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
#include <stdio.h>

static int dev_count = 0;
static char dev_prefix[10] = "/dev/mtd";

static int mtd_check_addr(struct device *dev, unsigned int addr)
{
	struct mtd *mtd = container_of(dev, struct mtd, dev);
	int ret = 0;

	if ((addr < mtd->base_addr) || (addr > (mtd->base_addr + mtd->total_size))) {
		error_printk("addr is out of flash\n");
		ret = -EINVAL;
	}

	return ret;
}

static int mtd_read(struct device *dev, unsigned char *buff, unsigned int size)
{
	struct mtd *mtd = container_of(dev, struct mtd, dev);
	int ret = 0;

	debug_printk("reading from mtd !\n");

	ret = mtd_check_addr(dev, mtd->base_addr + mtd->curr_off);
	if (ret < 0)
		return ret;

	return mtd_ops.read(mtd, buff, size);
}

static int mtd_write(struct device *dev, unsigned char *buff, unsigned int size)
{
	struct mtd *mtd = container_of(dev, struct mtd, dev);
	int ret = 0;

	debug_printk("writing from mtd !\n");

	ret = mtd_check_addr(dev, mtd->base_addr + mtd->curr_off);
	if (ret < 0)
		return ret;

	return mtd_ops.write(mtd, buff, size);

}

static int mtd_lseek(struct device *dev, int offset, int whence)
{
	struct mtd *mtd = container_of(dev, struct mtd, dev);
	int ret = 0;

	switch (whence) {
	case SEEK_SET:
		ret = mtd_check_addr(dev, mtd->base_addr + offset);
		if (ret < 0)
			return ret;

		mtd->curr_off = mtd->base_addr + offset;
		break;
	case SEEK_CUR:
		ret = mtd_check_addr(dev, mtd->curr_off + offset);
		if (ret < 0)
			return ret;

		mtd->curr_off += offset;
		break;
	case SEEK_END:
		ret = mtd_check_addr(dev, mtd->base_addr + mtd->total_size + offset);
		if (ret < 0)
			return ret;

		mtd->curr_off = mtd->base_addr + mtd->total_size + offset;
		break;
	}

	return ret;
}

int mtd_init(struct mtd *mtd)
{
	int i;
	int ret = 0;
	int total_size = 0;
	struct mtd *_mtd = NULL;

	_mtd = (struct mtd *)kmalloc(sizeof(struct mtd));
	if (_mtd < 0) {
		error_printk("cannot allocate mtd\n");
		return -ENOMEM;
	}

	memcpy(_mtd, mtd, sizeof(struct mtd));

	_mtd->curr_off = 0;

	for (i = 0; i < mtd->num_sectors; i++) {
		total_size += _mtd->sector_size[i];
	}

	memcpy(mtd->dev.name, dev_prefix, 10);

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
