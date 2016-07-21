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

#include <stdlib.h>
#include <thread.h>
#include <scheduler.h>
#include <string.h>

void create_context(struct registers *_regs, struct thread *_thread)
{

	unsigned int stack_top = _thread->start_stack + THREAD_STACK_OFFSET;

	stack_top = ROUNDDOWN(stack_top, 16);

	_regs = (struct registers *)stack_top;
	_regs--;

	memset(_regs, 0, sizeof(*_regs));

	_thread->regs->sp = _regs;
}
