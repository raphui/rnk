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
#include <lcd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <mm.h>
#include <init.h>

static int dev_count = 0;
static char dev_prefix[10] = "/dev/fb";
static struct list_node lcd_device_list;

struct lcd *lcd_get_device(void)
{
	struct lcd *lcddev = NULL;

	lcddev = list_peek_head_type(&lcd_device_list, struct lcd, node);

	return lcddev;
}

int lcd_configure_device(struct lcd *lcd)
{
	return lcd->lcd_ops->configure(lcd);
}

struct lcd *lcd_new_device(void)
{
	struct lcd *lcddev = NULL;

	lcddev = (struct lcd *)kmalloc(sizeof(struct lcd));
	if (!lcddev) {
		error_printk("cannot allocate lcd device\n");
		return NULL;
	}

	dev_count++;

	return lcddev;
}

int lcd_remove_device(struct lcd *lcd)
{
	int ret = 0;
	struct lcd *lcddev = NULL;

	list_for_every_entry(&lcd_device_list, lcddev, struct lcd, node)
		if (lcddev == lcd)
			break;

	if (lcddev) {
		list_delete(&lcddev->node);
		kfree(lcddev);
	}
	else
		ret = -ENOENT;


	dev_count--;

	return ret;
}

int lcd_register_device(struct lcd *lcd)
{
	int ret = 0;
	char tmp[10] = {0};

	memcpy(tmp, dev_prefix, sizeof(dev_prefix));

	/* XXX: ascii 0 start at 0x30
	 *	and (dev_count - 1) to start at 0
	 */
	tmp[8] = 0x30 + (dev_count - 1);

	memcpy(lcd->dev.name, tmp, sizeof(tmp));

	ret = device_register(&lcd->dev);
	if (ret < 0) {
		error_printk("failed to register device\n");
		ret = -ENOMEM;
		goto failed_out;
	}

	list_add_tail(&lcd_device_list, &lcd->node);

	return ret;

failed_out:
	/* XXX: deallocate here ? */
	kfree(lcd);
	return ret;
}

int lcd_init(void)
{
	int ret = 0;
	struct lcd_bus *bus = NULL;

	bus = (struct lcd_bus *)kmalloc(sizeof(struct lcd_bus));
	if (!bus) {
		error_printk("cannot allocate lcd bus\n");
		return -ENOMEM;
	}

	ret = device_register(&bus->dev);
	if (ret < 0) {
		error_printk("failed to register bus\n");
		ret = -ENOMEM;
		goto failed_out;
	}

	return ret;

failed_out:
	kfree(bus);
	return ret;
}
coredevice_initcall(lcd_init);
