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

#ifndef QUEUE_H
#define QUEUE_H

#include <list.h>

struct queue {
	unsigned int *head;
	unsigned int *tail;
	unsigned int *curr;
	unsigned int *wr;
	unsigned int item_queued;
	unsigned int size;
	unsigned int item_size;
	unsigned int waiting_post;
	unsigned int waiting_receive;
	struct list_node waiting_receive_threads;
	struct list_node waiting_post_threads;
};

int kqueue_init(struct queue *queue, unsigned int size, unsigned int item_size);
int kqueue_clear(struct queue *queue);
int kqueue_post(struct queue *queue, void *item, unsigned int timeout);
int kqueue_receive(struct queue *queue, void *item, unsigned int timeout);

#endif /* QUEUE_H */
