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

#include <board.h>
#include <unistd.h>
#include <errno.h>
#include <printk.h>
#include <string.h>
#include <device.h>
#include <export.h>
#include <syscall.h>

#define MAX_FD	8

struct device_io {
	struct device *dev;
	int (*read)(struct device *dev, unsigned char *buff, unsigned int size);
	int (*write)(struct device *dev, const unsigned char *buff, unsigned int size);
	int (*lseek)(struct device *dev, int offset, int whence);
	int perm;
};

static struct device_io devs[MAX_FD];

static int fd_num = 0;

int svc_open(const char *path, int flags)
{
	int ret = fd_num;
	struct device *dev;

	if (fd_num < MAX_FD) {
		dev = device_from_name(path);
		if (dev) {
			devs[fd_num].dev = dev;
			devs[fd_num].read = dev->read;
			devs[fd_num].write = dev->write;
			devs[fd_num].perm = flags;

			fd_num++;

		} else {
			error_printk("invalid open path\n");
			ret = -ENOENT;
		}


	} else {
		error_printk("no more file descriptor left\n");
		ret = -EMFILE;
	}

	return ret;
}

int open(const char *path, int flags)
{
	return syscall(SYSCALL_FD_OPEN, path, flags);
}
EXPORT_SYMBOL(open);

int svc_close(int fd)
{
	int ret = 0;

	if (fd < MAX_FD) {
		devs[fd].dev = NULL;
		devs[fd].read = NULL;
		devs[fd].write = NULL;
		devs[fd].perm = 0;

		fd_num--;
	} else {
		error_printk("invalid fd\n");
		ret = -EBADF;
	}

	return ret;
}

int close(int fd)
{
	return syscall(SYSCALL_FD_CLOSE, fd);
}
EXPORT_SYMBOL(close);

int svc_write(int fd, const void *buf, size_t size)
{
	int ret = 0;

	if (fd >= fd_num) {
		error_printk("invalid file descriptor\n");
		return -EBADF;
	}

	if (devs[fd].perm & (O_WRONLY | O_RDWR)) {
		if (devs[fd].write) {

			ret = devs[fd].write(devs[fd].dev, buf, size);

		} else {
			error_printk("cannot write to fd: %d\n", fd);
			ret = -ENOSYS;
		}
	} else {
		error_printk("forbidden writing to fd: %d\n", fd);
		ret = -EPERM;
	}


	return ret;
}

int write(int fd, const void *buf, size_t size)
{
	return syscall(SYSCALL_FD_WRITE, fd, buf, size);
}
EXPORT_SYMBOL(write);

int svc_read(int fd, void *buf, size_t size)
{
	int ret = 0;

	if (fd >= fd_num) {
		error_printk("invalid file descriptor\n");
		return -EBADF;
	}

	if (devs[fd].perm & (O_RDONLY | O_RDWR)) {
		if (devs[fd].read) {

			ret = devs[fd].read(devs[fd].dev, buf, size);

		} else {
			error_printk("cannot read to fd: %d\n", fd);
			ret = -ENOSYS;
		}
	} else {
		error_printk("forbidden reading to fd: %d\n", fd);
		ret = -EPERM;
	}

	return ret;
}

int read(int fd, void *buf, size_t size)
{
	return syscall(SYSCALL_FD_READ, fd, buf, size);
}
EXPORT_SYMBOL(read);


int svc_lseek(int fd, int offset, int whence)
{
	int ret = 0;

	if (fd >= fd_num) {
		error_printk("invalid file descriptor\n");
		return -EBADF;
	}

	if ((whence != SEEK_SET) || (whence != SEEK_CUR) || (whence != SEEK_END)) {
		error_printk("invalid whence\n");
		return -EINVAL;
	}

	if (devs[fd].lseek)
		ret = devs[fd].lseek(devs[fd].dev, offset, whence);
	else {
		error_printk("cannot lseek on fd: %d\n", fd);
		ret = -ENOSYS;
	}

	return ret;
}

int lseek(int fd, int offset, int whence)
{
	return syscall(SYSCALL_FD_LSEEK, fd, offset, whence);
}
EXPORT_SYMBOL(lseek);
