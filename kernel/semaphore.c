/*
 * Copyright (C) 2015  Raphaël Poggi <poggi.raph@gmail.com>
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

#include <semaphore.h>
#include <task.h>
#include <stdio.h>


static void insert_waiting_task(struct semaphore *sem, struct task *t)
{
	struct entry *e = (struct entry *)&(sem->waiting_tasks.head);
	struct task *task;

	while (e->next) {
		task = (struct task *)container_of(e, struct task, list_entry);

		if (t->priority > task->priority) {
			list_insert_before(&sem->waiting_tasks, &task->list_entry, &t->list_entry);
			break;
		}

		e = e->next;
	}
}

void init_semaphore(struct semaphore *sem, unsigned int value)
{
	sem->value = value;
	sem->count = 0;
	sem->waiting = 0;
}

void sem_wait(struct semaphore *sem)
{
	struct task *current_task;

	if (sem->count < sem->value) {
		debug_printk("sem (%x) got\r\n", sem);
		sem->count--;
	} else {
		debug_printk("unable to got sem (%x)\r\n", sem);

		current_task = get_current_task();
		current_task->state = TASK_BLOCKED;

		remove_runnable_task(current_task);
		insert_waiting_task(sem, current_task);

		sem->waiting++;
	}
}

void sem_post(struct semaphore *sem)
{
	struct task *task;
	struct entry *e;

	if (sem->waiting) {
		debug_printk("tasks are waiting for sem (%x)\r\n", sem);

		e = list_get_head(&(sem->waiting_tasks));
		task = (struct task *)container_of(e, struct task, list_entry);
		task->state = TASK_RUNNABLE;
		sem->waiting--;
		sem->count++;

		insert_runnable_task(task);
		schedule_task(task);

	} else {
		if (sem->count == sem->value)
			debug_printk("all sem (%x) token has been post\r\n", sem);
		else
			sem->count++;

		debug_printk("sem (%x) post\r\n", sem);
	}

}
