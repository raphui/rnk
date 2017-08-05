/*
 * Copyright (C) 2017  Raphaël Poggi <poggi.raph@gmail.com>
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

#include <armv7a/system.h>
#include <thread.h>
#include <scheduler.h>

void create_context(struct registers *_regs, struct thread *_thread)
{

	unsigned int cpsr = ARM_MODE_USR;

	asm volatile(
                 "stmdb   %[stack]!, {%[psr]}   /* xPSR */                      \n\
                  stmdb   %[stack]!, {%[pc]}    /* PC */                        \n\
                  stmdb   %[stack]!, {%[lr]}    /* LR */                        \n\
                  stmdb   %[stack]!, {%[zero]}  /* R12 */                       \n\
                  stmdb   %[stack]!, {%[zero]}  /* R3 */                        \n\
                  stmdb   %[stack]!, {%[zero]}  /* R2 */                        \n\
                  stmdb   %[stack]!, {%[zero]}  /* R1 */                        \n\
                  stmdb   %[stack]!, {%[zero]}  /* R0 */                        \n\
                  stmdb   %[stack]!, {%[zero]}  /* R11 */                       \n\
                  stmdb   %[stack]!, {%[zero]}  /* R10 */                       \n\
                  stmdb   %[stack]!, {%[zero]}  /* R9 */                        \n\
                  stmdb   %[stack]!, {%[zero]}  /* R8 */                        \n\
                  stmdb   %[stack]!, {%[frame]} /* R7 - Frame Pointer*/         \n\
                  stmdb   %[stack]!, {%[zero]}  /* R6 */                        \n\
                  stmdb   %[stack]!, {%[zero]}  /* R5 */                        \n\
                  stmdb   %[stack]!, {%[zero]}  /* R4 */                        \n"
                  /* Output */
                  :[stack] "+r" (_thread->regs->sp)
                  /* Input */
                  :[pc] "r" (_thread->func), [lr] "r" (_thread->regs->lr), [frame] "r" (_thread->start_stack + 2000),
                   [zero] "r" (0), [psr] "r" (cpsr) /* Set the Thumb bit */
                  /* Clobber */
                  :);
}
