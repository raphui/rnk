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

void init_queue(struct queue *queue, unsigned int size, unsigned int item_size)
{
	queue->size = size;
	queue->item_size = item_size;
}

void queue_post(struct queue *queue, void *item)
{
}

void queue_receive(struct queue *queue, void *item)
{
}

void svc_queue_post(struct queue *queue, void *item)
{
}

void svc_queue_receive(struct queue *queue, void *item)
{
}
