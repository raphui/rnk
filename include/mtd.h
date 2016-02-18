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

#ifndef MTD_H
#define MTD_H

#include <device.h>

#define MAX_SECTORS	32

struct mtd {
	unsigned int base_addr;
	unsigned int sector_size[MAX_SECTORS];
	unsigned int num_sectors;
	unsigned int total_size;
	int curr_off;
	struct device dev;
};

struct mtd_operations
{
	int (*init)(struct mtd *mtd);
	int (*erase)(struct mtd *mtd, unsigned int sector);
	int (*write)(struct mtd *mtd, unsigned char *buff, unsigned int size);
	int (*read)(struct mtd *mtd, unsigned char *buff, unsigned int size);
};

int mtd_init(struct mtd *mtd);

#endif /* MTD_H */
