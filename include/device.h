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

#ifndef DEVICE_H
#define DEVICE_H

#include <list.h>
#include <stddef.h>

struct device {
	char name[32];
	const char *of_compat;
	struct list_node next;
	int (*probe)(struct device *device);
	int (*remove)(struct device *device);
	int (*read)(struct device *device, unsigned char *buff, unsigned int size);
	int (*write)(struct device *device, unsigned char *buff, unsigned int size);
	int (*lseek)(struct device *device, int offset, int whence);
};

int device_register(struct device *dev);
int device_unregister(struct device *dev);
struct device *device_from_name(const char *name);

#endif /* DEVICE_H */
