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

struct lcd {
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
};

struct lcd_operations
{
	int (*init)(struct ltdc *ltdc);
	void (*init_gpio)(void);
};

int lcd_init(struct ltdc *ltdc);
void lcd_init_gpio(void);

#endif /* LCD_H */
