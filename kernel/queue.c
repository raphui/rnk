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
 * along with this program; if not, write to the Frrestore * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <queue.h>
#include <task.h>
#include <scheduler.h>
#include <mm.h>
#include <arch/svc.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

static void insert_waiting_receive_task(struct queue *queue, struct task *t)
{
	struct task *task;

	if (queue->waiting_receive) {
		LIST_FOREACH(task, &queue->waiting_receive_tasks, next) {
			if (t->priority > task->priority)
				LIST_INSERT_BEFORE(task, t, next);
		}

	} else {
		LIST_INSERT_HEAD(&queue->waiting_receive_tasks, t, next);
	}


}

static void insert_waiting_post_task(struct queue *queue, struct task *t)
{
	struct task *task;

	if (queue->waiting_post) {
		LIST_FOREACH(task, &queue->waiting_post_tasks, next) {
			if (t->priority > task->priority)
				LIST_INSERT_BEFORE(task, t, next);
		}

	} else {
		LIST_INSERT_HEAD(&queue->waiting_post_tasks, t, next);
	}


}

void init_queue(struct queue *queue, unsigned int size, unsigned int item_size)
{
	queue->item_queued = 0;
	queue->size = size;
	queue->item_size = item_size;

	queue->head = (unsigned int *)kmalloc(size * item_size);
	queue->curr = queue->head;
	queue->wr = queue->head;
	queue->tail = queue->head + (size * item_size);

	queue->waiting_receive = 0;
	queue->waiting_post = 0;
	LIST_INIT(&queue->waiting_receive_tasks);
	LIST_INIT(&queue->waiting_post_tasks);
}

void svc_queue_post(struct queue *queue, void *item)
{
	if (queue->item_queued < queue->item_size) {
		if ((queue->wr + queue->item_size) <= queue->tail) {
			memcpy(queue->wr, item, queue->item_size);
			printk("wr: %x, v: %d\r\n", queue->wr, *(int *)item);
			queue->wr += queue->item_size;
			queue->item_queued++;
		}
	}
}

void queue_post(struct queue *queue, void *item, unsigned int timeout)
{
	int back_from_sleep = 0;
	for (;;) {
		if (queue->item_queued < queue->item_size) {
			SVC_ARG2(SVC_QUEUE_POST, queue, item);
			break;
		} else if (timeout) {
			usleep(timeout);
			back_from_sleep = 1;
		} else if (back_from_sleep || !timeout) {
			break;
		}
	}
	
}

void svc_queue_receive(struct queue *queue, void *item)
{
	if (queue->item_queued) {
		if ((queue->curr + queue->item_size) <= queue->wr) {
			memcpy(item, queue->curr, queue->item_size);
			queue->curr += queue->item_size;
			queue->item_queued--;
		}
	}
}

void queue_receive(struct queue *queue, void *item, unsigned int timeout)
{
	int back_from_sleep = 0;
	for (;;) {
		if (queue->item_queued) {
			SVC_ARG2(SVC_QUEUE_RECEIVE, queue, item);
			break;
		} else if (timeout) {
			usleep(timeout);
			back_from_sleep = 1;
		} else if (back_from_sleep || !timeout) {
			break;
		}
	}
}
