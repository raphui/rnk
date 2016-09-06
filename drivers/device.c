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
#include <list.h>
#include <string.h>
#include <stdio.h>

static int device_count = 0;

static struct list_node device_list;

static void insert_device(struct device *dev)
{
	list_add_head(&device_list, &dev->next);

	device_count++;
}

int device_register(struct device *dev)
{
	int ret = 0;

	if (!device_count)
		list_initialize(&device_list);

	insert_device(dev);

	return ret;
}

int device_unregister(struct device *dev)
{
	int ret = 0;

	list_delete(&dev->next);

	return ret;
}

struct device *device_from_name(const char *name)
{
	struct device *dev = NULL;

	list_for_every_entry(&device_list, dev, struct device, next) {
		if (!strcmp(name, dev->name))
			break;
	}

	return dev;
}
