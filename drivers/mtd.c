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

#include <utils.h>
#include <mtd.h>
#include <mm.h>
#include <errno.h>
#include <string.h>
#include <init.h>
#include <printk.h>

static int dev_count = 0;
static char dev_prefix[10] = "/dev/mtd";
static struct list_node mtd_controller_list;

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

	verbose_printk("reading from mtd !\n");

	ret = mtd_check_addr(dev, mtd->base_addr + mtd->curr_off);
	if (ret < 0)
		return ret;

	ret = mtd->mtd_ops->read(mtd, buff, size);
	if (ret < 0)
		return ret;

	mtd->curr_off += size;

	return ret;
}

static int mtd_write(struct device *dev, unsigned char *buff, unsigned int size)
{
	struct mtd *mtd = container_of(dev, struct mtd, dev);
	int ret = 0;

	verbose_printk("writing from mtd !\n");

	ret = mtd_check_addr(dev, mtd->base_addr + mtd->curr_off);
	if (ret < 0)
		return ret;

	ret = mtd->mtd_ops->write(mtd, buff, size);
	if (ret < 0)
		return ret;

	mtd->curr_off += size;

	return ret;
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

struct mtd *mtd_new_controller(void)
{
	struct mtd *mtd = NULL;

	mtd = (struct mtd *)kmalloc(sizeof(struct mtd));
	if (!mtd) {
		error_printk("cannot allocate mtd controller");
		return NULL;
	}

	memset(mtd, 0, sizeof(struct mtd));

	dev_count++;

	return mtd;
}

int mtd_remove_controller(struct mtd *mtd)
{
	int ret = 0;

	struct mtd *mtddev = NULL;

	ret = device_unregister(&mtd->dev);
	if (ret < 0) {
		error_printk("failed to unregister mtd controller");
		return ret;
	}

	list_for_every_entry(&mtd_controller_list, mtddev, struct mtd, node)
		if (mtddev == mtd)
			break;

	if (mtddev) {
		list_delete(&mtddev->node);
		kfree(mtddev);
	}
	else
		ret = -ENOENT;



	dev_count--;

	return ret;
}

int mtd_register_controller(struct mtd *mtd)
{
	int i;
	int ret = 0;
	int total_size = 0;
	char tmp[10] = {0};

	mtd->curr_off = 0;

	for (i = 0; i < mtd->layout_size; i++) {
		total_size += mtd->mtd_map[i].pages_count * mtd->mtd_map[i].pages_size;
	}

	mtd->total_size = total_size;

	memcpy(tmp, dev_prefix, sizeof(dev_prefix));

	/* XXX: ascii 0 start at 0x30 */
	tmp[8] = 0x30 + dev_count;

	memcpy(mtd->dev.name, tmp, 10);

	mtd->dev.read = mtd_read;
	mtd->dev.write = mtd_write;

	list_add_tail(&mtd_controller_list, &mtd->node);

	ret = device_register(&mtd->dev);
	if (ret < 0)
		error_printk("failed to register mtd controller\n");

	return ret;
}

int mtd_init(void)
{
	int ret = 0;

	list_initialize(&mtd_controller_list);

	return ret;
}
postcore_initcall(mtd_init);
