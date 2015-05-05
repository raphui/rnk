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

static int index_current_task = -1;
static int task_count = 0;
static struct task *current_task;

static void increment_task_priority(void)
{
	struct entry *e = &runnable_tasks.head;
	struct task *task;

	while (e->next) {
		task = (struct task *)container_of(e, struct task, list_entry);
		task->priority++;
		e = e->next;
	}
}

static void insert_task(struct task *t)
{
	struct entry *e = &runnable_tasks.head;
	struct task *task;

	while (e->next) {
		task = (struct task *)container_of(e, struct task, list_entry);

		if (t->priority > task->priority) {
			list_insert_before(task, t);
			break;
		}

		e = e->next;
	}
}

void task_init(void)
{
	list_init(&runnable_tasks);
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
	task->regs->sp = task[task_count]->start_stack;
	task->regs->lr = (unsigned int)end_task;
	task->regs->pc = (unsigned int)func;

	/* Creating task context */
	create_context(task[task_count]->regs, task[task_count]);

	insert_task(&runnable_tasks, &task->list_entry);

	task_count++;
}

void first_switch_task(int index_task)
{
	SVC_ARG(SVC_TASK_SWITCH, NULL);
}

void switch_task(int index_task)
{
	struct task *task;
	struct entry *e = list_get_head(&runnable_tasks);

	task = (struct task *)container_of(e, struct task, list_entry);

	task->state = TASK_RUNNING;

	current_task->regs->sp = PSP();
	SET_PSP(task->regs->sp);

	current_task->state = TASK_RUNNABLE;
	current_task = task;

	increment_task_priority();

	insert_task(&runnable_tasks, &current_task->list_entry);
}

int get_task_count(void)
{
	return task_count;
}

struct task *get_current_task(void)
{
	return current_task;
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

