/*
 * Copyright (C) 2018 Raphaël Poggi <poggi.raph@gmail.com>
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

#include <stdarg.h>
#include <errno.h>
#include <syscall.h>
#include <scheduler.h>
#include <kmutex.h>
#include <queue.h>
#include <ksem.h>
#include <ktime.h>
#include <unistd.h>
#include <elfloader.h>
#include <mm.h>

#include <arch/syscall.h>

struct syscall syscall_table[] = {
	{SYSCALL_THREAD_SWITCH, (unsigned int *)&schedule_thread},
	{SYSCALL_THREAD_CREATE, (unsigned int *)&add_thread},
	{SYSCALL_THREAD_STOP, (unsigned int *)&schedule_thread_stop},
	{SYSCALL_MUTEX_CREATE, (unsigned int *)&kmutex_init},
	{SYSCALL_MUTEX_ACQUIRE, (unsigned int *)&kmutex_lock},
	{SYSCALL_MUTEX_RELEASE, (unsigned int *)&kmutex_unlock},
	{SYSCALL_SEM_CREATE, (unsigned int *)&ksem_init},
	{SYSCALL_SEM_WAIT, (unsigned int *)&ksem_wait},
	{SYSCALL_SEM_POST, (unsigned int *)&ksem_post},
	{SYSCALL_TIME_USLEEP, (unsigned int *)&ktime_usleep},
	{SYSCALL_TIME_ONESHOT, (unsigned int *)&ktime_oneshot},
	{SYSCALL_QUEUE_POST, (unsigned int *)&svc_queue_post},
	{SYSCALL_QUEUE_RECEIVE, (unsigned int *)&svc_queue_receive},
	{SYSCALL_FD_OPEN, (unsigned int *)&svc_open},
	{SYSCALL_FD_CLOSE, (unsigned int *)&svc_close},
	{SYSCALL_FD_WRITE, (unsigned int *)&svc_write},
	{SYSCALL_FD_READ, (unsigned int *)&svc_read},
	{SYSCALL_FD_LSEEK, (unsigned int *)&svc_lseek},
	{SYSCALL_ALLOC, (unsigned int *)umalloc},
	{SYSCALL_FREE, (unsigned int *)ufree},
};

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
