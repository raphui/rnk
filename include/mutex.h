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

#ifndef MUTEX_H
#define MUTEX_H

#include <thread.h>
#include <list.h>

struct mutex {
	unsigned char lock;
	struct thread *owner;
	unsigned int waiting;
	struct list_node waiting_threads;
};

void kmutex_init(struct mutex *mutex);
void kmutex_lock(struct mutex *mutex);
void kmutex_unlock(struct mutex *mutex);

#endif /* MUTEX_H */
