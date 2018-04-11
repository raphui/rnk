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

#include <pthread.h>
#include <syscall.h>
#include <export.h>

int pthread_create(void (*start_routine)(void *), void *arg, unsigned int priority)
{
	return syscall(SYSCALL_THREAD_CREATE, start_routine, arg, priority);
}
EXPORT_SYMBOL(pthread_create);

int pthread_mutex_init(pthread_mutex_t *mutex)
{
	return syscall(SYSCALL_MUTEX_CREATE, &mutex->kmutex);
}
EXPORT_SYMBOL(pthread_mutex_init);

int pthread_mutex_lock(pthread_mutex_t *mutex)
{
	return syscall(SYSCALL_MUTEX_ACQUIRE, &mutex->kmutex);
}
EXPORT_SYMBOL(pthread_mutex_lock);

int pthread_mutex_unlock(pthread_mutex_t *mutex)
{
	return syscall(SYSCALL_MUTEX_RELEASE, &mutex->kmutex);
}
EXPORT_SYMBOL(pthread_mutex_unlock);
