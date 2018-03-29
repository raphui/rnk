/*
 * Copyright (C) 2014  Raphaël Poggi <poggi.raph@gmail.com>
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
#include <stdarg.h>
#include <syscall.h>

#include <armv7m/svc.h>
#include <arch/system.h>

#define MAX_SYSCALL_ARGUMENT	4

int arch_system_call(unsigned int call, va_list va)
{
	int i;
	int ret = 0;
	void *args[MAX_SYSCALL_ARGUMENT];

	for (i = 0; i < MAX_SYSCALL_ARGUMENT; i++)
		args[i] = va_arg(va, void *);

	if (call == SYSCALL_THREAD_SWITCH)
		arch_request_sched();
	else if (call < SYSCALL_END)
		svc_arg3(call, args[0], args[1], args[2]);

	return ret;
}