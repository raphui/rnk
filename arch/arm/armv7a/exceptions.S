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

#define S_FRAME_SIZE	72

#define S_OLD_R0	68
#define S_PSR		64
#define S_PC		60
#define S_LR		56
#define S_SP		52

#define S_IP		48
#define S_FP		44
#define S_R10		40
#define S_R9		36
#define S_R8		32
#define S_R7		28
#define S_R6		24
#define S_R5		20
#define S_R4		16
#define S_R3		12
#define S_R2		8
#define S_R1		4
#define S_R0		0

	.macro	bad_save_user_regs
	sub	sp, sp, #S_FRAME_SIZE
	stmia	sp, {r0 - r12}			@ Calling r0-r12
	ldr	r2, =abort_stack
	ldmia	r2, {r2 - r3}			@ get pc, cpsr
	add	r0, sp, #S_FRAME_SIZE		@ restore sp_SVC

	add	r5, sp, #S_SP
	mov	r1, lr
	stmia	r5, {r0 - r3}			@ save sp_SVC, lr_SVC, pc, cpsr
	mov	r0, sp
	.endm

	.macro	irq_save_user_regs
	sub	sp, sp, #S_FRAME_SIZE
	stmia	sp, {r0 - r12}			@ Calling r0-r12
	add     r8, sp, #S_PC
	stmdb   r8, {sp, lr}^                   @ Calling SP, LR
	str     lr, [r8, #0]                    @ Save calling PC
	mrs     r6, spsr
	str     r6, [r8, #4]                    @ Save CPSR
	str     r0, [r8, #8]                    @ Save OLD_R0
	mov	r0, sp
	.endm

	.macro	irq_restore_user_regs
	ldmia	sp, {r0 - lr}^			@ Calling r0 - lr
	mov	r0, r0
	ldr	lr, [sp, #S_PC]			@ Get PC
	add	sp, sp, #S_FRAME_SIZE
	subs	pc, lr, #4			@ return & move spsr_svc into cpsr
	.endm

	.macro get_bad_stack
	ldr	r13, =abort_stack
	str	lr, [r13]			@ save caller lr / spsr
	mrs	lr, spsr
	str     lr, [r13, #4]

	mov	r13, #ARM_MODE_SVC			@ prepare SVC-Mode
	@ msr	spsr_c, r13
	msr	spsr, r13
	mov	lr, pc
	movs	pc, lr
	.endm

#if 0
	.macro try_data_abort
	ldr	r13, =arm_ignore_data_abort	@ check try mode
	ldr	r13, [r13]
	cmp	r13, #0
	beq	do_abort_\@
	ldr	r13, =arm_data_abort_occurred
	str	r13, [r13]
	mrs	r13, spsr			@ read saved CPSR
	tst	r13, #1<<5			@ check Thumb mode
	subeq	lr, #4				@ next ARM instr
	subne	lr, #6				@ next Thumb instr
	movs	pc, lr
do_abort_\@:
	.endm
#endif

	.macro get_irq_stack			@ setup IRQ stack
	ldr	sp, IRQ_STACK_START
	.endm

	.macro get_fiq_stack			@ setup FIQ stack
	ldr	sp, FIQ_STACK_START
	.endm

.arm

ENTRY(undef_vector)
	get_bad_stack
	bad_save_user_regs
	bl	undefined_instruction

ENTRY(swi_vector)
	irq_save_user_regs
	ldr	r0,[lr,#-4]
	bic	r0, r0, #0xFF000000
	mov	r1, sp
	bl	software_interrupt
	irq_restore_user_regs

ENTRY(pabt_vector)
	get_bad_stack
	bad_save_user_regs
	bl	prefetch_abort

ENTRY(dabt_vector)
	get_bad_stack
	bad_save_user_regs
	bl	data_abort

ENTRY(rsvd_vector)
	b	rsvd_vector

ENTRY(irq_vector)
	irq_save_user_regs
	bl	irq_interrupt
	irq_restore_user_regs

ENTRY(fiq_vector)
	get_bad_stack
	bad_save_user_regs
	bl	fiq_interrupt

ENTRY(reset_vector)
	/*
	 * ICIALLU: Invalidate all instruction caches to PoU,
	 * includes flushing of branch predictors.
	 * Even if the i-cache is off it might contain stale entries
	 * that are better discarded before enabling the cache.
	 * Architectually this is even possible after a cold reset.
	 */
	mcr	p15, 0, r12, c7, c5, 0
	/* DSB, ensure completion of the invalidation */
	mcr	p15, 0, r12, c7, c10, 4
	/*
	 * ISB, ensure instruction fetch path is in sync.
	 * Note that the ARM Architecture Reference Manual, ARMv7-A and ARMv7-R
	 * edition (ARM DDI 0406C.c) doesn't define this instruction in the
	 * ARMv6 part (D12.7.10). It only has: "Support of additional
	 * operations is IMPLEMENTATION DEFINED".
	 * But an earlier version of the ARMARM (ARM DDI 0100I) does define it
	 * as "Flush prefetch buffer (PrefetchFlush)".
	 */
	mcr	p15, 0, r12, c7, c5, 4

	/* disable MMU stuff and data/unified caches */
	mrc	p15, 0, r12, c1, c0, 0		/* SCTLR */
	bic	r12, r12, #(CR_M | CR_C | CR_B)
	bic	r12, r12, #(CR_S | CR_R | CR_V)

	/* enable instruction cache */
	orr	r12, r12, #CR_I

	orr	r12, r12, #CR_U
	bic	r12, r12, #CR_A

	mcr	p15, 0, r12, c1, c0, 0		/* SCTLR */

	/* set up the stack for irq, fiq, abort, undefined, system/user, and lastly supervisor mode */
	mov     r12, #0

	cpsid   i,#0x11       /* fiq */
	mov     sp, r12

	cpsid   i,#0x17       /* abort */
	mov     sp, r12

	cpsid   i,#0x1b       /* undefined */
	mov     sp, r12

	cpsid   i,#0x1f       /* system */
	mov     sp, r12

	/* set the cpu to SVC32 mode, mask irq and fiq */
	mrs	r12, cpsr
	orr	r12, r12, #I_BIT
	orr	r12, r12, #F_BIT
	bic	r12, r12, #ARM_MODE
	orr	r12, r12, #ARM_MODE_SVC
	msr	cpsr, r12

	bl	arch_init

abort_stack:
.space 8
