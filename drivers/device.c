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
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <device.h>
#include <string.h>
#include <stdio.h>

static int device_count = 0;

LIST_HEAD(, device) device_list = LIST_HEAD_INITIALIZER(device_list);

static void insert_device(struct device *dev)
{
	struct device *device;

	device = LIST_FIRST(&device_list);

	if (!device_count) {
		verbose_printk("device list is empty\r\n");
		LIST_INSERT_HEAD(&device_list, dev, next);
	}
	else
		LIST_INSERT_BEFORE(device, dev, next);
}

int device_register(struct device *dev)
{
	int ret = 0;

	insert_device(dev);

	return ret;
}

int device_unregister(struct device *dev)
{
	int ret = 0;

	LIST_REMOVE(dev, next);

	return ret;
}

struct device *device_from_name(const char *name)
{
	struct device *dev = NULL;

	LIST_FOREACH(dev, &device_list, next) {
		if (!strcmp(name, dev->name))
			break;
	}

	return dev;
}
