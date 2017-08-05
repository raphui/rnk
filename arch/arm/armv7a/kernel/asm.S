/*
 * Copyright (C) 2017 Raphaël Poggi <poggi.raph@gmail.com>
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
#include <asm/linkage.h>

.arm
ENTRY(arm_context_switch)
	push	{r4-r11, lr}
	str	sp, [r0]
	clrex	/* clear exclusive lock */
	mov	sp, r1
	pop	{r4-r11, lr}
	mov	pc, lr

ENTRY(arm_get_stack)
	mov	r0, sp
	mov	pc, lr

ENTRY(arm_set_stack)
	mov	sp, r0
	mov	pc, lr

ENTRY(arm_set_irq_stack)
	mrs	r3, cpsr	/* save current mode */
	bic	r4, r3, #ARM_MODE
	orr	r4, r4, #ARM_MODE_IRQ
	msr	cpsr, r4
	mov	sp, r0
	msr	cpsr, r3	/* restore arm mode */
	mov	pc, lr

ENTRY(arm_get_user_stack)
	mrs	r3, cpsr	/* save current mode */
	bic	r4, r3, #ARM_MODE
	orr	r4, r4, #ARM_MODE_SYS
	msr	cpsr, r4
	mov	r0, sp
	msr	cpsr, r3	/* restore arm mode */
	mov	pc, lr

ENTRY(arm_set_user_stack)
	mrs	r3, cpsr	/* save current mode */
	bic	r4, r3, #ARM_MODE
	orr	r4, r4, #ARM_MODE_SYS
	msr	cpsr, r4
	mov	sp, r0
	msr	cpsr, r3	/* restore arm mode */
	mov	pc, lr
