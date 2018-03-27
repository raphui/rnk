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
#include <mutex.h>
#include <queue.h>
#include <semaphore.h>
#include <time.h>
#include <elfloader.h>

#include <arch/syscall.h>

struct syscall syscall_table[] = {
	{SYSCALL_THREAD_SWITCH, (unsigned int *)&schedule_thread},
	{SYSCALL_ACQUIRE_MUTEX, (unsigned int *)&svc_mutex_lock}, 
	{SYSCALL_RELEASE_MUTEX, (unsigned int *)&svc_mutex_unlock},
	{SYSCALL_WAIT_SEM, (unsigned int *)&svc_sem_wait},
	{SYSCALL_POST_SEM, (unsigned int *)&svc_sem_post},
	{SYSCALL_USLEEP, (unsigned int *)svc_usleep},
	{SYSCALL_QUEUE_POST, (unsigned int *)&svc_queue_post},
	{SYSCALL_QUEUE_RECEIVE, (unsigned int *)&svc_queue_receive},
	{SYSCALL_TIMER_ONESHOT, (unsigned int *)&svc_timer_soft_oneshot},
	{SYSCALL_ELF_LOAD, (unsigned int *)&svc_elf_exec}
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
