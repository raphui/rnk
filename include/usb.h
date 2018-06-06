/*
 * Copyright (C) 2018  Raphaël Poggi <poggi.raph@gmail.com>
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

#ifndef USB_H
#define USB_H

#include <device.h>

struct usb_device;

struct usb_operations
{
	int (*write)(struct usb_device *usb, unsigned char *buff, unsigned int size);
	int (*read)(struct usb_device *usb, unsigned char *buff, unsigned int size);
};

struct usb_device {
	unsigned int base_reg;
	unsigned int irq;
	void *priv;
	struct device dev;
	struct list_node node;
	struct usb_operations *usb_ops;
};


struct usb_device *usb_new_device(void);
int usb_remove_device(struct usb_device *usb);
int usb_register_device(struct usb_device *usb);

#endif /* USB_H */
