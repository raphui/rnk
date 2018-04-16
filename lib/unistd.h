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

#ifndef UNISTD_H
#define UNISTD_H

#include <stddef.h>
#include <printk.h>

#define O_RDONLY        0
#define O_WRONLY        1
#define O_RDWR          2

int open(const char *path, int flags);
int close(int fd);
int write(int fd, const void *buf, size_t size);
int read(int fd, void *buf, size_t size);
int lseek(int fd, int offset, int whence);

int svc_open(const char *path, int flags);
int svc_close(int fd);
int svc_write(int fd, const void *buf, size_t size);
int svc_read(int fd, void *buf, size_t size);
int svc_lseek(int fd, int offset, int whence);

int syscall(int number, ...);

#endif /* UNISTD_H */
