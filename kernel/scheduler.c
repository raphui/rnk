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
#include <armv7m/system.h>

int task_switching = 0;
unsigned int system_tick = 0;

void schedule_init(void)
{
	task_init();
}

void start_schedule(void)
{
	first_switch_task();
}


void schedule(void)
{
	struct task *t;

	t = find_next_task();
	switch_task(t);
	task_switching = 1;
}

void schedule_task(struct task *task)
{
	struct task *t;

	if (task)
		switch_task(task);
	else {
		t = find_next_task();
		switch_task(t);
	}
	
	task_switching = 1;
}

void schedule_isr(void)
{
	pendsv_request();
}

/* Since tasks cannot end, if we jump into this functions it's mean that the context switch is buggy */
void end_task(void)
{
	struct task *task = get_current_task();

	task->state = TASK_STOPPED;

	remove_runnable_task(task);
}
