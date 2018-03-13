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

#include <thread.h>
#include <scheduler.h>
#include <armv7m/svc.h>

void create_context(struct registers *_regs, struct thread *_thread)
{
	_thread->regs->lr = 0xFFFFFFFD;

	asm volatile(
                 "stmdb   %[stack]!, {%[psr]}   /* xPSR */                      \n\
                  stmdb   %[stack]!, {%[pc]}    /* PC */                        \n\
                  stmdb   %[stack]!, {%[lr]}    /* LR */                        \n\
                  stmdb   %[stack]!, {%[zero]}  /* R12 */                       \n\
                  stmdb   %[stack]!, {%[zero]}  /* R3 */                        \n\
                  stmdb   %[stack]!, {%[zero]}  /* R2 */                        \n\
                  stmdb   %[stack]!, {%[zero]}  /* R1 */                        \n\
                  stmdb   %[stack]!, {%[zero]}  /* R0 */                        \n"
                  /* Output */
                  :[stack] "+r" (_thread->regs->sp)
                  /* Input */
                  :[pc] "r" (_thread->func), [lr] "r" (_thread->regs->lr),
                   [zero] "r" (0), [psr] "r" (0x01000000) /* Set the Thumb bit */
                  /* Clobber */
                  :);
}

void save_user_context(void)
{
	save_context();
}

void get_user_context(void)
{
	restore_context();
}
