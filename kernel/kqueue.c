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

#include <kqueue.h>
#include <thread.h>
#include <scheduler.h>
#include <mm.h>
#include <syscall.h>
#include <printk.h>
#include <string.h>
#include <ktime.h>
#include <spinlock.h>
#include <export.h>

static void insert_waiting_receive_thread(struct queue *queue, struct thread *t)
{
	struct thread *thread;

#if defined(CONFIG_SCHEDULE_ROUND_ROBIN) || defined(CONFIG_SCHEDULE_RR_PRIO)
		list_add_tail(&queue->waiting_receive_threads, &t->event_node);
#elif defined(CONFIG_SCHEDULE_PRIORITY)
		list_for_every_entry(&queue->waiting_receive_threads, thread, struct thread, event_node)
			if (t->priority > thread->priority)
				list_add_before(&thread->event_node, &t->event_node);

#endif

}

static void remove_waiting_receive_thread(struct queue *queue, struct thread *t)
{
	list_delete(&t->event_node);
}

static void insert_waiting_post_thread(struct queue *queue, struct thread *t)
{
	struct thread *thread;

#if defined(CONFIG_SCHEDULE_ROUND_ROBIN) || defined(CONFIG_SCHEDULE_RR_PRIO)
		list_add_tail(&queue->waiting_post_threads, &t->event_node);
#elif defined(CONFIG_SCHEDULE_PRIORITY)
		list_for_every_entry(&queue->waiting_post_threads, thread, struct thread, event_node)
			if (t->priority > thread->priority)
				list_add_before(&thread->event_node, &t->event_node);

#endif

}

static void remove_waiting_post_thread(struct queue *queue, struct thread *t)
{
	list_delete(&t->event_node);
}

void kqueue_init(struct queue *queue, unsigned int size, unsigned int item_size)
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
	list_initialize(&queue->waiting_receive_threads);
	list_initialize(&queue->waiting_post_threads);
}

void kqueue_clear(struct queue *queue)
{
	queue->item_queued = 0;
	queue->wr = queue->head;
	queue->curr = queue->head;
}

static void _kqueue_post(struct queue *queue, void *item)
{
	struct thread *t = NULL;

	if (queue->item_queued < queue->size) {
		if ((queue->wr + queue->item_size) <= queue->tail) {
			memcpy(queue->wr, item, queue->item_size);
			debug_printk("wr: %x, v: %d\r\n", queue->wr, *(int *)item);
			queue->wr += queue->item_size;
			queue->item_queued++;

			if (!list_is_empty(&queue->waiting_receive_threads)) {
				t = list_peek_head_type(&queue->waiting_receive_threads, struct thread, event_node);

				t->state = THREAD_RUNNABLE;
				remove_waiting_receive_thread(queue, t);
				schedule_yield();
			}
		}
	}
}

void kqueue_post(struct queue *queue, void *item, unsigned int timeout)
{
	int back_from_sleep = 0;
	for (;;) {
		if (back_from_sleep) {
			break;
		} else if (queue->item_queued < queue->item_size) {
			_kqueue_post(queue, item);
			break;
		} else if (timeout) {
			insert_waiting_post_thread(queue, get_current_thread());
			ktime_usleep(timeout);
			back_from_sleep = 1;
		} else if (!timeout) {
			break;
		}
	}

}

static void _kqueue_receive(struct queue *queue, void *item)
{
	struct thread *t = NULL;


	if (queue->item_queued) {
		if ((queue->curr + queue->item_size) <= queue->wr) {
			memcpy(item, queue->curr, queue->item_size);
			queue->curr += queue->item_size;
			queue->item_queued--;

			if (!list_is_empty(&queue->waiting_post_threads)) {
				t = list_peek_head_type(&queue->waiting_post_threads, struct thread, event_node);

				t->state = THREAD_RUNNABLE;
				remove_waiting_post_thread(queue, t);
				schedule_yield();
			}

		}

		if (queue->curr == queue->wr)
			kqueue_clear(queue);
	}
}

void kqueue_receive(struct queue *queue, void *item, unsigned int timeout)
{
	int back_from_sleep = 0;
	for (;;) {
		if (queue->item_queued) {
			_kqueue_receive(queue, item);
			break;
		} else if (timeout) {
			insert_waiting_receive_thread(queue, get_current_thread());
			ktime_usleep(timeout);
			back_from_sleep = 1;
		} else if (back_from_sleep || !timeout) {
			break;
		}
	}
}
