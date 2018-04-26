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

#define MAX_SYSCALL_ARGUMENT	3
#define MAX_FD	8

extern void svc_noarg(int number);
extern void svc_arg1(int number, void *arg);
extern void svc_arg2(int number, void *arg, void *arg2);
extern void *svc_arg3(int number, void *arg, void *arg2, void *arg3);

static int arch_system_call(unsigned int call, va_list va)
{
	int i;
	int ret = 0;
	void *args[MAX_SYSCALL_ARGUMENT];

	for (i = 0; i < MAX_SYSCALL_ARGUMENT; i++)
		args[i] = va_arg(va, void *);

	if (call < SYSCALL_END)
		ret = (int)svc_arg3(call, args[0], args[1], args[2]);

	return ret;
}

int open(const char *path, int flags)
{
	return syscall(SYSCALL_FD_OPEN, path, flags);
}
EXPORT_SYMBOL(open);

int close(int fd)
{
	return syscall(SYSCALL_FD_CLOSE, fd);
}
EXPORT_SYMBOL(close);

int write(int fd, const void *buf, size_t size)
{
	return syscall(SYSCALL_FD_WRITE, fd, buf, size);
}
EXPORT_SYMBOL(write);

int read(int fd, void *buf, size_t size)
{
	return syscall(SYSCALL_FD_READ, fd, buf, size);
}
EXPORT_SYMBOL(read);

int lseek(int fd, int offset, int whence)
{
	return syscall(SYSCALL_FD_LSEEK, fd, offset, whence);
}
EXPORT_SYMBOL(lseek);

int syscall(int number, ...)
{
	va_list va;
	int ret = 0;

	va_start(va, number);

	if (number >= SYSCALL_END)
		return -EINVAL;

	ret = arch_system_call(number, va);

	return ret;
}
