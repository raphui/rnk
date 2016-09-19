/*
 * Copyright (C) 2015 Raphaël Poggi <poggi.raph@gmail.com>
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

#include <thread.h>
#include <armv7m/system.h>

void init_arch(void)
{
}

void arch_init_tick(void)
{
	init_systick();
}

void arch_request_sched(void)
{
	pendsv_request();
}

unsigned int arch_get_thread_stack(void)
{
	return PSP();
}

void arch_set_thread_stack(struct thread *t)
{
	SET_PSP((void *)t->regs->sp);
}
