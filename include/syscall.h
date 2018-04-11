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

#ifndef SYSCALL_H
#define SYSCALL_H

enum service_calls {
	SYSCALL_THREAD_SWITCH,
	SYSCALL_THREAD_STOP,
	SYSCALL_ACQUIRE_MUTEX,
	SYSCALL_RELEASE_MUTEX,
	SYSCALL_WAIT_SEM,
	SYSCALL_POST_SEM,
	SYSCALL_USLEEP,
	SYSCALL_QUEUE_POST,
	SYSCALL_QUEUE_RECEIVE,
	SYSCALL_TIMER_ONESHOT,
	SYSCALL_ELF_LOAD,
	SYSCALL_FD_OPEN,
	SYSCALL_FD_CLOSE,
	SYSCALL_FD_WRITE,
	SYSCALL_FD_READ,
	SYSCALL_FD_LSEEK,
	SYSCALL_END,
};

struct syscall {
	int number;
	unsigned int *handler;
};

extern struct syscall syscall_table[];

int syscall(int number, ...);

#endif /* SYSCALL_H */
