/*
 * Copyright (C) 2017  Raphaël Poggi <poggi.raph@gmail.com>
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

#include <stdio.h>
#include <string.h>
#include <errno.h>

#define PATH	"/home/raphio/test_semihosting"

int main(void)
{
	int ret = 0;
	char buff[1024];
	FILE *f;

	f = fopen(PATH, "rb");
	if (!f) {
		error_printk("failed to open: %s\n", PATH);
		ret = -ENOENT;
		goto out;
	}

	memset(buff, 0, 1024);

	ret = fread(buff, 1024, sizeof(char), f);
	if (ret < 0) {
		error_printk("failed to read from: %s\n", PATH);
		ret = -EIO;
		goto out;
	}

	printk("content from [%s]: %s\n", PATH, buff);

out:
	return ret;
}
