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

#include <console.h>
#include <mm.h>
#include <errno.h>
#include <init.h>
#include <fdtparse.h>
#include <stdio.h>

static struct console *cons;

int console_write(unsigned char *buff, unsigned int len)
{
	int ret = 0;

	if (!cons->io_ops) {
		error_printk("io operation not set\n");
		return -ENOTTY;
	}

	ret = cons->io_ops->write(cons->pdata, buff, len);

	return ret;
}

static int console_init(void)
{
	int offset;
	int ret = 0;
	char *path = NULL;
	struct device *dev = NULL;
	void *blob = fdtparse_get_blob();

	offset = fdtparse_alias_offset("console");
	if (offset < 0) {
		error_printk("failed to get offset for alias: console\n");
		return -ENOENT;
	}

	path = fdtparse_get_path(offset);
	if (!path) {
		error_printk("failed to retrieve console alias path\n");
		return -ENOENT;
	}

	dev = device_from_of_path(path);
	if (!dev) {
		error_printk("failed to retrieve console device struct\n");
		return -ENOENT;
	}

	cons = (struct console *)kmalloc(sizeof(struct console));
	if (!cons) {
		error_printk("failed to allocate console struct\n");
		return -ENOMEM;
	}

	cons->pdata = dev;
	cons->io_ops = &io_op;	

	return ret;
}
late_initcall(console_init);
