/*
 * Copyright (C) 2014  Raphaël Poggi <poggi.raph@gmail.com>
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

#ifndef LCD_H
#define LCD_H

#include <device.h>
#include <list.h>

struct lcd {
	unsigned int base_reg;
	unsigned short hsync;
	unsigned short vsync;
	unsigned short hbp;
	unsigned short hfp;
	unsigned short vbp;
	unsigned short vfp;
	unsigned int width;
	unsigned int height;
	unsigned char bpp;
	unsigned int fb_addr;
	struct lcd_operations *lcd_ops;
	struct list_node node;
	struct device dev;
};

struct lcd_bus {
	struct device dev;
};

struct lcd_operations
{
	int (*configure)(struct lcd *lcd);
	void (*init_gpio)(void);
};

struct lcd *lcd_get_device(void);
int lcd_configure_device(struct lcd *lcd);
struct lcd *lcd_new_device(void);
int lcd_remove_device(struct lcd *lcd);
int lcd_register_device(struct lcd *lcd);
void lcd_init_gpio(void);
int lcd_init(void);

#endif /* LCD_H */
