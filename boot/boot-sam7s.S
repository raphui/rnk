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


/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

//#include <board.h>
#include <sam7s-reg.h>

//------------------------------------------------------------------------------
//         Definitions
//------------------------------------------------------------------------------

#define IRQ_STACK_SIZE   8*3*4

#define ARM_MODE_ABT     0x17
#define ARM_MODE_FIQ     0x11
#define ARM_MODE_IRQ     0x12
#define ARM_MODE_SVC     0x13
#define ARM_MODE_SYS	 0x1F
#define ARM_MODE_USR	 0x10
#define ARM_MODE	 0x1F

#define I_BIT            0x80
#define F_BIT            0x40

//------------------------------------------------------------------------------
//         Startup routine
//------------------------------------------------------------------------------

            .align      4
            .arm
 
/* Exception vectors
 *******************/
            .section    .vectors, "a"

resetVector:
        ldr     pc, =resetHandler       /* Reset */
undefVector:
        b       undefVector             /* Undefined instruction */

swiVector:
	b swiHandler			/* Software interrupt */

prefetchAbortVector:
        b       prefetchAbortVector     /* Prefetch abort */
dataAbortVector:
        b       dataAbortVector         /* Data abort */
reservedVector:
        b       reservedVector          /* Reserved for future use */
irqVector:
        b       irqHandler              /* Interrupt */
fiqVector:
                                        /* Fast interrupt */
//------------------------------------------------------------------------------
/// Handles a fast interrupt request by branching to the address defined in the
/// AIC.
//------------------------------------------------------------------------------
fiqHandler:
        b       fiqHandler

swiHandler:
	stmfd   sp!, {r0-r12, lr}
	mrs	r8, SPSR
	stmfd	sp!, {r8}
	mov	r0, lr		/* Put the SVC instruction (lr - 4) in r0 */

        bl       __svc_handler               /* Software interrupt */

	ldmia	sp!, {r8}
	msr	SPSR, r8
        ldmia   sp!, {r0-r12, lr}
	subs	pc, lr, #0

//------------------------------------------------------------------------------
/// Handles incoming interrupt requests by branching to the corresponding
/// handler, as defined in the AIC. Supports interrupt nesting.
//------------------------------------------------------------------------------
irqHandler:

/* Save interrupt context on the stack to allow nesting */
        sub     lr, lr, #4
        stmfd   sp!, {lr}
        mrs     r8, SPSR
        stmfd   sp!, {r0, r8}

	mov	r1, r8
	and	r1, r1, #ARM_MODE
	teq	r1, #ARM_MODE_USR
	mov	r10, lr
	bleq	svc_save_user_context

/* Write in the IVR to support Protect Mode */
        ldr     lr, =AT91C_BASE_AIC
        ldr     r0, [r14, #AIC_IVR]
        str     lr, [r14, #AIC_IVR]

/* Branch to interrupt handler in Supervisor mode */
        msr     CPSR_c, #ARM_MODE_SVC | I_BIT | F_BIT
        stmfd   sp!, {r1-r3, r4, r12, lr}
        mov     lr, pc
        bx      r0
        ldmia   sp!, {r1-r3, r4, r12, lr}
        msr     CPSR_c, #ARM_MODE_IRQ | I_BIT | F_BIT

/* Acknowledge interrupt */
        ldr     lr, =AT91C_BASE_AIC
        str     lr, [r14, #AIC_EOICR]

/* Clear interruption */
	ldr	lr, =AT91C_BASE_AIC
	ldr	r0, [r14, #AIC_IPR]
	str	r0, [r14, #AIC_ICCR]

	and	r1, r1, #ARM_MODE
	teq	r1, #ARM_MODE_USR
	bleq	svc_get_user_context

/* Restore interrupt context and branch back to calling code */
        ldmia   sp!, {r0, lr}
        msr     SPSR_cxsf, lr
        ldmia   sp!, {pc}^

//------------------------------------------------------------------------------
/// Initializes the chip and branches to the main() function.
//------------------------------------------------------------------------------
            .section    .text
            .global     entry

entry:
resetHandler:

/* Dummy access to the .vectors section so it does not get optimized */
        ldr     r0, =resetVector

/* Set pc to actual code location (i.e. not in remap zone) */
	ldr     pc, =1f

/* Perform low-level initialization of the chip using LowLevelInit() */
1:
        ldr     r4, =_sstack
	ldr	r5, =_ustack
        mov     sp, r4
	ldr     r0, =low_level_init
        mov     lr, pc
        bx      r0

/* Initialize the relocate segment */

        ldr     r0, =_efixed
        ldr     r1, =_srelocate
        ldr     r2, =_erelocate
1:
        cmp     r1, r2
        ldrcc   r3, [r0], #4
        strcc   r3, [r1], #4
        bcc     1b

/* Clear the zero segment */
	ldr     r0, =_szero
        ldr     r1, =_ezero
        mov     r2, #0
1:
        cmp     r0, r1
        strcc   r2, [r0], #4
        bcc     1b

/* Setup stacks
 **************/
/* IRQ mode */
        msr     CPSR_c, #ARM_MODE_IRQ | I_BIT | F_BIT
        mov     sp, r4
        sub     r4, r4, #IRQ_STACK_SIZE

/* System mode*/
	msr	CPSR_c, #ARM_MODE_SYS | I_BIT | F_BIT
	mov	sp, r5

/* Supervisor mode (interrupts enabled) */
        msr     CPSR_c, #ARM_MODE_SVC | F_BIT
        mov     sp, r4

/* Branch to main()
 ******************/
        ldr     r0, =main
        mov     lr, pc
        bx      r0

/* Loop indefinitely when program is finished */
1:
        b       1b

