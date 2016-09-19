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

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <thread.h>

extern int thread_switching;
extern unsigned int system_tick;

void create_context(struct registers *_regs, struct thread *_thread);
void activate_context(struct thread *_thread);
void switch_context(struct registers *_current_regs, struct registers *_thread_regs);
void save_user_context(void);
void get_user_context(void);
void start_schedule(void);
int schedule_init(void);
void schedule(void);
void schedule_thread(struct thread *thread);
void schedule_isr(void);
void end_thread(void);

#endif /* SCHEDULER_H */
