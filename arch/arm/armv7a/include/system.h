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

#ifndef ARMV7A_SYSTEM_H
#define ARMV7A_SYSTEM_H

#define ARCH_STACK_SIZE	0x10000
#define CACHE_LINE 32

#define ARM_MODE_USR	0x10
#define ARM_MODE_FIQ	0x11
#define ARM_MODE_IRQ    0x12
#define ARM_MODE_SVC    0x13
#define ARM_MODE_ABT    0x17
#define ARM_MODE_SYS	0x1F
#define ARM_MODE	0x1F

#define I_BIT            0x80
#define F_BIT            0x40

#define isb() __asm__ __volatile__ ("isb" : : : "memory")
#define dsb() __asm__ __volatile__ ("dsb" : : : "memory")
#define dmb() __asm__ __volatile__ ("dmb" : : : "memory")

/*
 *  * CR1 bits (CP#15 CR1)
 *   */
#define CR_M    (1 << 0)	/*  MMU enable				*/
#define CR_A    (1 << 1)	/*  Alignment abort enable		*/
#define CR_C    (1 << 2)	/*  Dcache enable			*/
#define CR_W    (1 << 3)	/*  Write buffer enable			*/
#define CR_P    (1 << 4)	/*  32-bit exception handler		*/
#define CR_D    (1 << 5)	/*  32-bit data address range		*/
#define CR_L    (1 << 6)	/*  Implementation defined		*/
#define CR_B    (1 << 7)	/*  Big endian				*/
#define CR_S    (1 << 8)	/*  System MMU protection		*/
#define CR_R    (1 << 9)	/*  ROM MMU protection			*/
#define CR_F    (1 << 10)	/*  Implementation defined		*/
#define CR_Z    (1 << 11)	/*  Implementation defined		*/
#define CR_I    (1 << 12)	/*  Icache enable			*/
#define CR_V    (1 << 13)	/*  Vectors relocated to 0xffff0000	*/
#define CR_RR   (1 << 14)	/*  Round Robin cache replacement	*/
#define CR_L4   (1 << 15)	/*  LDR pc can set T bit		*/
#define CR_DT   (1 << 16)
#define CR_IT   (1 << 18)
#define CR_ST   (1 << 19)
#define CR_FI   (1 << 21)	/*  Fast interrupt (lower latency mode)	*/
#define CR_U    (1 << 22)	/*  Unaligned access operation		*/
#define CR_XP   (1 << 23)	/*  Extended page tables		*/
#define CR_VE   (1 << 24)	/*  Vectored interrupts			*/
#define CR_EE   (1 << 25)	/*  Exception (Big) Endian		*/
#define CR_TRE  (1 << 28)	/*  TEX remap enable			*/
#define CR_AFE  (1 << 29)	/*  Access flag enable			*/
#define CR_TE   (1 << 30)	/*  Thumb exception enable		*/

#ifndef __ASSEMBLY__

static inline void __enable_it(void)
{
	asm volatile("cpsie i");
}

static inline void __disable_it(void)
{
	asm volatile("cpsid i");
}

static inline int arch_it_disabled(void)
{
	unsigned int state;

	asm volatile("mrs %0, cpsr" : "=r"(state));
	state &= 0x1;

	return !!state;
}

static inline void wait_for_interrupt(void)
{
	asm volatile("wfi");
}

struct pt_regs {
	long uregs[18];
};

static inline unsigned int get_cr(void)
{
		unsigned int val;
		asm volatile("mrc p15, 0, %0, c1, c0, 0  @ get CR" : "=r" (val) : : "cc");
		return val;
}

static inline void set_cr(unsigned int val)
{
		asm volatile("mcr p15, 0, %0, c1, c0, 0 @ set CR"
				  : : "r" (val) : "cc");
		isb();
}

static inline unsigned int get_vbar(void)
{
		unsigned int vbar;
		asm volatile("mrc p15, 0, %0, c12, c0, 0 @ get VBAR"
			     : "=r" (vbar) : : "cc");
		return vbar;
}

static inline void set_vbar(unsigned int vbar)
{
		asm volatile("mcr p15, 0, %0, c12, c0, 0 @ set VBAR"
			     : : "r" (vbar) : "cc");
		isb();
}

extern unsigned int arm_get_stack(void);
extern void arm_set_stack(void *stack);

extern void arm_set_irq_stack(void *stack);

extern void arm_set_user_stack(void *stack);
extern unsigned int arm_get_user_stack(void);

extern void *__bss_start;
extern void *__bss_stop;

extern void arm_sys_mode(void);

#endif

#endif /* ARMV7A_SYSTEM_H */
