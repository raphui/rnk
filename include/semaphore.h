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

#ifndef SEMAPHORE_H
#define SEMAPHORE_H

#include <list.h>

struct semaphore {
	unsigned int value;
	unsigned int count;
	unsigned int waiting;
	LIST_HEAD(, task) waiting_tasks;
};

void init_semaphore(struct semaphore *sem, unsigned int value);
void svc_sem_wait(struct semaphore *sem);
void svc_sem_post(struct semaphore *sem);
void sem_wait(struct semaphore *sem);
void sem_post(struct semaphore *sem);

#endif /* SEMAPHORE_H */
