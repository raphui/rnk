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

#include <scheduler.h>
#include <task.h>
#include <stdio.h>
#include <pit.h>
#include <board.h>
#include <arch/svc.h>

int task_switching = 0;

void start_schedule(void)
{
	int i;

	i = find_next_task();
	first_switch_task(i);
}


void schedule(void)
{
	int i;

	i = find_next_task();
	switch_task(i);
	task_switching = 1;
}

void schedule_task(struct task *task)
{
	int i;

	if (task)
		switch_task(task->pid);
	else {
		i = find_next_task();
		switch_task(i);
	}
	
	task_switching = 1;
}

/* Since tasks cannot end, if we jump into this functions it's mean that the context switch is buggy */
void end_task(void)
{
	void (*func)(void) = (void (*)())0xfffffd39;

	func();
}
