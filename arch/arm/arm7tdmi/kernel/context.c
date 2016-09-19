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

#include <thread.h>
#include <scheduler.h>
#include <arch/svc.h>

void create_context(struct registers *_regs, struct thread *_thread)
{
	svc_create_context(_regs, _thread->start_stack, (unsigned int)_thread->func, (unsigned int)end_thread);
}

void activate_context(struct thread *_thread)
{
	svc_activate_context(_thread->regs);
}

void switch_context(struct registers *_current_regs, struct registers *_thread_regs)
{
	svc_switch_context(_current_regs, _thread_regs);
}

void save_user_context(void)
{
	svc_save_user_context();
}

void get_user_context(void)
{
	svc_get_user_context();
}
