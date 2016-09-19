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

#ifndef SVC_H
#define SVC_H

#include <thread.h>

enum service_calls {
	SVC_THREAD_SWITCH,
	SVC_ACQUIRE_MUTEX,
	SVC_RELEASE_MUTEX,
	SVC_WAIT_SEM,
	SVC_POST_SEM,
	SVC_USLEEP,
	SVC_QUEUE_POST,
	SVC_QUEUE_RECEIVE,
};

/*
 * We need to make sure that we get the return value
 * without screwing up r0, since GCC doesn't understand that
 * SVC has a return value.
 *
 * Save the LR to prevent clobbering it with reentrant SVC calls.
 */
#define SVC(call)  ({ \
    unsigned int ret = 0;   \
    asm volatile ("push {lr}     \n"    \
                  "svc  %[code]  \n"    \
                  "pop  {lr}     \n"    \
                  "mov  %[ret], r0  \n" \
                  :[ret] "+r" (ret)     \
                  :[code] "I" (call)    \
                  :"r0");               \
    ret;    \
})

#define SVC_ARG(call, arg)  ({ \
    unsigned int ret = 0;   \
    asm volatile ("mov  r1, %[ar]  \n"  \
                  "push {lr}     \n"    \
                  "svc  %[code]  \n"    \
                  "pop  {lr}     \n"    \
                  "mov  %[ret], r0  \n" \
                  :[ret] "+r" (ret)     \
                  :[code] "I" (call), [ar] "r" (arg)     \
                  :"r0");               \
    ret;    \
})

#define SVC_ARG2(call, arg1, arg2)  ({ \
    unsigned int ret = 0;   \
    asm volatile ("mov  r1, %[ar1]  \n"  \
                  "mov  r2, %[ar2]  \n"  \
                  "push {lr}     \n"    \
                  "svc  %[code]  \n"    \
                  "pop  {lr}     \n"    \
                  "mov  %[ret], r0  \n" \
                  :[ret] "+r" (ret)     \
                  :[code] "I" (call), [ar1] "r" (arg1), [ar2] "r" (arg2)     \
                  :"r0", "r1");               \
    ret;    \
})

void __activate_context(struct registers *_reg);
void __switch_context(struct registers *_curr_reg, struct registers *_reg);
void save_context(void);
void restore_context(void);

#endif /* SVC_H */
