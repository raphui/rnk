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

#include <task.h>
#include <scheduler.h>
#include <arch/svc.h>

void create_context(struct registers *_regs, struct task *_task)
{
	svc_create_context(_regs, _task->start_stack, (unsigned int)_task->func, (unsigned int)end_task);
}

void activate_context(struct task *_task)
{
	svc_activate_context(_task->regs);
}

void switch_context(struct registers *_current_regs, struct registers *_task_regs)
{
	svc_switch_context(_current_regs, _task_regs);
}

void save_user_context(void)
{
	svc_save_user_context();
}

void get_user_context(void)
{
	svc_get_user_context();
}
