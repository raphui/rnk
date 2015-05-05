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
#include <stdio.h>
#include <mm.h>
#include <utils.h>
#include <arch/svc.h>
#include <armv7m/system.h>

static int index_current_task = -1;
static int task_count = 0;

static void increment_task_priority(void)
{
	int i;
	int task_count = get_task_count();

	for (i = 0; i < task_count; i++) {
		if (i != index_current_task)
			task[i]->priority++;
	}
}

void task_init(void)
{
	list_init(&runnable_tasks);
}

void add_task(void (*func)(void), unsigned int priority)
{
	task[task_count] = (struct task *)kmalloc(sizeof(struct task));
	task[task_count]->state = TASK_RUNNABLE;
	task[task_count]->pid = task_count;
	task[task_count]->priority = priority;
	task[task_count]->start_stack = TASK_STACK_START + (task_count * TASK_STACK_OFFSET);
	task[task_count]->func = func;
	task[task_count]->regs = (struct registers *)kmalloc(sizeof(struct registers));
	task[task_count]->regs->sp = task[task_count]->start_stack;
	task[task_count]->regs->lr = (unsigned int)end_task;
	task[task_count]->regs->pc = (unsigned int)func;

	/* Creating task context */
	create_context(task[task_count]->regs, task[task_count]);

	task_count++;
}

void first_switch_task(int index_task)
{
	SVC_ARG(SVC_TASK_SWITCH, NULL);
}

void switch_task(int index_task)
{
	if (task[index_task]->state != TASK_BLOCKED) {
		task[index_task]->state = TASK_RUNNING;

		task[index_current_task]->regs->sp = PSP();
		SET_PSP(task[index_task]->regs->sp);

		index_current_task = index_task;

		increment_task_priority();
	}

}

int get_task_count(void)
{
	return task_count;
}

struct task *get_current_task(void)
{
	return task[index_current_task];
}

int get_current_task_index(void)
{
	return index_current_task;
}

int find_next_task(void)
{
	int i;
	int next = 0;
	int t = 0;
	int task_count = get_task_count();

	for (i = 0; i < task_count; i++)
		t = max(t, task[i]->priority);

	for (i = 0; i < task_count; i++) {
		if (task[i]->priority >= t && task[i]->state != TASK_BLOCKED) {
			t = task[i]->priority;
			next = i;
		}
	}

	return next;
}

