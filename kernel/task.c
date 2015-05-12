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
#include <queue.h>
#include <arch/svc.h>
#include <armv7m/system.h>

static struct task *current_task;
static int task_count = 0;
LIST_HEAD(, task) runnable_tasks = LIST_HEAD_INITIALIZER(runnable_tasks);

static void idle_task(void)
{
	while(1)
		;
}


static void increment_task_priority(void)
{
	struct task *task;

	LIST_FOREACH(task, &runnable_tasks, next) {
		debug_printk("task %d with priority %d\r\n", task->pid, task->priority);
		if ((task->func != &idle_task)) {
			debug_printk("increasing priority of task %d\r\n", task->pid);
			task->priority++;
		}
	}
}

static void insert_task(struct task *t)
{
	struct task *task;

	task = LIST_FIRST(&runnable_tasks);

	if (!task) {
		debug_printk("runnable list is empty\r\n");
		LIST_INSERT_HEAD(&runnable_tasks, task, next);
		return;
	}

	LIST_FOREACH(task, &runnable_tasks, next) {
//		debug_printk("t->priority: %d, task->priority; %d\r\n", t->priority, task->priority);
		if (t->priority > task->priority)
			LIST_INSERT_BEFORE(task, t, next);
	}
}

void task_init(void)
{
	LIST_INIT(&runnable_tasks);
	add_task(&idle_task, 0);
}

void add_task(void (*func)(void), unsigned int priority)
{
	struct task *task = (struct task *)kmalloc(sizeof(struct task));
	task->state = TASK_RUNNABLE;
	task->pid = task_count;
	task->priority = priority;
	task->start_stack = TASK_STACK_START + (task_count * TASK_STACK_OFFSET);
	task->func = func;
	task->regs = (struct registers *)kmalloc(sizeof(struct registers));
	task->regs->sp = task->start_stack;
	task->regs->lr = (unsigned int)end_task;
	task->regs->pc = (unsigned int)func;

	/* Creating task context */
	create_context(task->regs, task);

	if (task_count)
		insert_task(task);
	else
		LIST_INSERT_HEAD(&runnable_tasks, task, next);

	task_count++;
}

void first_switch_task(void)
{
	SVC_ARG(SVC_TASK_SWITCH, NULL);
}

void switch_task(struct task *task)
{
	if (current_task) {
		current_task->regs->sp = PSP();
		current_task->state = TASK_RUNNABLE;
		increment_task_priority();
		insert_task(current_task);
	}

	task->state = TASK_RUNNING;
	SET_PSP((void *)task->regs->sp);
	current_task = task;
}

struct task *get_current_task(void)
{
	return current_task;
}

struct task *find_next_task(void)
{
	struct task *task = NULL;

	task = LIST_FIRST(&runnable_tasks);
	LIST_REMOVE(task, next);

	printk("next task: %x\r\n", task);

	return task;
}

void insert_runnable_task(struct task *task)
{
	insert_task(task);
}

void remove_runnable_task(struct task *task)
{
	LIST_REMOVE(task, next);
}
