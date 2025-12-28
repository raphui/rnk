#ifndef ARMV7M_SYSTEM_H
#define ARMV7M_SYSTEM_H

#include <utils.h>

extern int systick_init(void);
extern int systick_disable_it(void);
extern int systick_enable_it(void);

static inline void READ_AND_DISCARD(unsigned int*reg) {
    asm volatile ("" : "=m" (*reg) : "r" (*reg));
}

static inline unsigned char FAULTMASK(void) {
    unsigned char val;

    asm volatile ("mrs    %[val], faultmask"
        :[val] "=r" (val)
        ::);

    return val;
}

static inline unsigned char IPSR(void) {
    unsigned char val;

    asm volatile ("mrs    %[val], ipsr"
        :[val] "=r" (val)
        ::);

    return val;
}

static inline unsigned int PSP(void) {
    unsigned int val;

    asm volatile ("mrs    %[val], psp"
        :[val] "=r" (val)
        ::);

    return val;
}

static inline void SET_PSP(void *addr) {
    asm volatile ("msr    psp, %[addr]"
        ::[addr] "r" (addr)
        :);
}

static inline unsigned int *MSP(void) {
    unsigned int *val;

    asm volatile ("mrs    %[val], msp"
        :[val] "=r" (val)
        ::);

    return val;
}

static inline void __isb(void)
{
	asm volatile ("isb" ::: "memory");
}

static inline void __dsb(void)
{
	asm volatile ("dsb" ::: "memory");
}

static inline void __enable_it(void)
{
	asm volatile("" ::: "memory");
	asm volatile ("cpsie i":::);
}

static inline void __disable_it(void)
{
	asm volatile ("cpsid i":::);
	asm volatile("" ::: "memory");
}

static inline int arch_it_disabled(void)
{
	unsigned int state;

	asm volatile ("mrs %0, primask" : "=r"(state));
	state &= 0x1;

	return !!state;
}

static inline void wait_for_interrupt(void)
{
	asm volatile ("wfi");
}

/* Cortex M4 General Registers */

#define CORTEX_M_PERIPH_BASE		(volatile unsigned int) (0xE0000000)

/* System Control Map */
#define SCS_BASE                        (volatile unsigned int) (0xE000E000)                                 /* System Control Space Base Address */
#define SYSTICK_BASE                    (SCS_BASE + 0x0010)                                     /* Systick Registers Base Address */
#define NVIC_BASE                       (SCS_BASE + 0x0100)                                     /* Nested Vector Interrupt Control */
#define SCB_BASE                        (SCS_BASE + 0x0D00)                                     /* System Control Block Base Address */
#define MPU_BASE                        (SCB_BASE + 0x0090)                                     /* MPU Block Base Address */
#define FPU_BASE                        (SCB_BASE + 0x0230)                                     /* FPU Block Base Address */
#define DWT_BASE			(SCB_BASE + 0x1000)					/* DWT Block Base Address */

/* SysTick Timer */
#define SYSTICK_CTL                     (volatile unsigned int) (SYSTICK_BASE)                    /* Control register for SysTick timer peripheral */
#define SYSTICK_RELOAD                  (volatile unsigned int) (SYSTICK_BASE + 0x04)             /* Value assumed by timer upon reload */
#define SYSTICK_VAL                     (volatile unsigned int) (SYSTICK_BASE + 0x08)             /* Current value of timer */
#define SYSTICK_CAL                     (volatile unsigned int) (SYSTICK_BASE + 0x0C)             /* Calibration settings/value register */

/* Nested Vector Interrupt Controller */
#define NVIC_ISER0                      (volatile unsigned int) (NVIC_BASE + 0x000)               /* Interrupt set-enable register 0 */
#define NVIC_ISER1                      (volatile unsigned int) (NVIC_BASE + 0x004)               /* Interrupt set-enable register 1 */
#define NVIC_ISER2                      (volatile unsigned int) (NVIC_BASE + 0x008)               /* Interrupt set-enable register 2 */
#define NVIC_ISER3                      (volatile unsigned int) (NVIC_BASE + 0x00C)               /* Interrupt set-enable register 3 */
#define NVIC_ICER0                      (volatile unsigned int) (NVIC_BASE + 0x080)               /* Interrupt clear-enable register 0 */
#define NVIC_ICER1                      (volatile unsigned int) (NVIC_BASE + 0x084)               /* Interrupt clear-enable register 1 */
#define NVIC_ICER2                      (volatile unsigned int) (NVIC_BASE + 0x088)               /* Interrupt clear-enable register 2 */
#define NVIC_ICER3                      (volatile unsigned int) (NVIC_BASE + 0x08C)               /* Interrupt clear-enable register 3 */
#define NVIC_ISPR0                      (volatile unsigned int) (NVIC_BASE + 0x100)               /* Interrupt set-pending register 0 */
#define NVIC_ISPR1                      (volatile unsigned int) (NVIC_BASE + 0x104)               /* Interrupt set-pending register 1 */
#define NVIC_ISPR2                      (volatile unsigned int) (NVIC_BASE + 0x108)               /* Interrupt set-pending register 2 */
#define NVIC_ISPR3                      (volatile unsigned int) (NVIC_BASE + 0x10C)               /* Interrupt set-pending register 3 */
#define NVIC_ICPR0                      (volatile unsigned int) (NVIC_BASE + 0x180)               /* Interrupt clear-pending register 0 */
#define NVIC_ICPR1                      (volatile unsigned int) (NVIC_BASE + 0x184)               /* Interrupt clear-pending register 1 */
#define NVIC_ICPR2                      (volatile unsigned int) (NVIC_BASE + 0x188)               /* Interrupt clear-pending register 2 */
#define NVIC_ICPR3                      (volatile unsigned int) (NVIC_BASE + 0x18C)               /* Interrupt clear-pending register 3 */
#define NVIC_IPR(n)                     (volatile unsigned int)  (NVIC_BASE + 0x300 + n)           /* Interrupt n priority register */

#define SCS_ICTR			(volatile unsigned int) (SCS_BASE + 0x004)
#define SCS_DEMCR			(volatile unsigned int) (SCS_BASE + 0xDFC)

/* System Control Block (SCB) */
#define SCB_ICSR                        (volatile unsigned int) (SCB_BASE + 0x004)                /* Interrupt Control and State Register */
#define SCB_VTOR                        (volatile unsigned int) (SCB_BASE + 0x008)                /* Vector Table Offset Register */
#define SCB_AIRCR                       (volatile unsigned int) (SCB_BASE + 0x00C)                /* Application Interrupt and Reset Control Register */
#define SCB_SCR                         (volatile unsigned int) (SCB_BASE + 0x010)                /* System Control Register */
#define SCB_SHPR(n)			(volatile unsigned int) (SCB_BASE + 0x18 + (n - 4))	 /* System Handler Priority Register */
#define SCB_SHPR1			(volatile unsigned int) (SCB_BASE + 0x18)		  /* System Handler Priority Register 1 */
#define SCB_SHPR2			(volatile unsigned int) (SCB_BASE + 0x1C)		  /* System Handler Priority Register 2 */
#define SCB_SHPR3			(volatile unsigned int) (SCB_BASE + 0x20)		  /* System Handler Priority Register 3 */
#define SCB_SHCSR                       (volatile unsigned int) (SCB_BASE + 0x024)                /* System Handler Control and State Register */
#define SCB_CFSR                        (volatile unsigned int) (SCB_BASE + 0x028)                /* Configurable fault status register - Describes Usage, Bus, and Memory faults */
#define SCB_HFSR                        (volatile unsigned int) (SCB_BASE + 0x02C)                /* Hard fault status register - Describes hard fault */
#define SCB_MMFAR                       (volatile unsigned int) (SCB_BASE + 0x034)                /* Memory management fault address register - Address that caused fault */
#define SCB_BFAR                        (volatile unsigned int) (SCB_BASE + 0x038)                /* Bus fault address register - Address that caused fault */
#define SCB_CPACR                       (volatile unsigned int) (SCB_BASE + 0x088)                /* Coprocessor (FPU) Access Control Register */

/* Memory Protection Unit (MPU)
 * ST PM0214 (Cortex M4 Programming Manual) pg. 195 */
#define MPU_TYPER                       (volatile unsigned int) (MPU_BASE + 0x00)                 /* MPU Type Register - Describes HW MPU */
#define MPU_CTRL                        (volatile unsigned int) (MPU_BASE + 0x04)                 /* MPU Control Register */
#define MPU_RNR                         (volatile unsigned int) (MPU_BASE + 0x08)                 /* MPU Region Number Register */
#define MPU_RBAR                        (volatile unsigned int) (MPU_BASE + 0x0C)                 /* MPU Region Base Address Register */
#define MPU_RASR                        (volatile unsigned int) (MPU_BASE + 0x10)                 /* MPU Region Attribute and Size Register */

/* Floating Point Unit (FPU)
 * ST PM0214 (Cortex M4 Programming Manual) pg. 236 */
#define FPU_CCR                         (volatile unsigned int) (FPU_BASE + 0x04)                 /* FPU Context Control Register */
#define FPU_CAR                         (volatile unsigned int) (FPU_BASE + 0x08)                 /* FPU Context Address Register */

/**********************************************************************************************************************************************/

/* Special purpose CONTROL register */
#define CONTROL_NPRIV                   (1 << 0)                                                /* Execution privilege in thread mode */
#define CONTROL_SPSEL                   (1 << 1)                                                /* SP selection */
#define CONTROL_FPCA                    (1 << 2)                                                /* FP extension enable */

/* System Control Block */
#define SCB_ICSR_PENDSVCLR              (unsigned int) (1 << 27)                                    /* Clear PendSV interrupt */
#define SCB_ICSR_PENDSVSET              (unsigned int) (1 << 28)                                    /* Set PendSV interrupt */

#define SCB_SCR_SLEEPONEXIT             (unsigned int) (1 << 1)                                     /* Sleep on return from interrupt routine */
#define SCB_SCR_SLEEPDEEP               (unsigned int) (1 << 2)                                     /* Use deep sleep as low power mode */
#define SCB_SCR_SEVONPEND               (unsigned int) (1 << 4)                                     /* Send event on pending exception */

#define SCB_SHCSR_MEMFAULTENA           (unsigned int) (1 << 16)                                    /* Enables Memory Management Fault */
#define SCB_SHCSR_BUSFAULTENA           (unsigned int) (1 << 17)                                    /* Enables Bus Fault */
#define SCB_SHCSR_USEFAULTENA           (unsigned int) (1 << 18)                                    /* Enables Usage Fault */

#define SCB_AIRCR_VECTKEY_SHIFT		(unsigned int) (16)					    /* Register key */
#define SCB_AIRCR_PRIGROUP_MASK		(unsigned int) (0x7)					    /* Interrupt priority grouping */
#define SCB_AIRCR_PRIGROUP_SHIFT	(unsigned int) (8)					    /* Interrupt priority grouping */
#define SCB_AIRCR_SYSRESETREQ		(unsigned int) (1 << 2)					    /* System reset request */

/* Hard Fault Status Register */
#define SCB_HFSR_VECTTBL                (unsigned int) (1 << 1)                                     /* Vector table hard fault.  Bus fault on vector table read during exception handling. */
#define SCB_HFSR_FORCED                 (unsigned int) (1 << 30)                                    /* Forced hard fault.  Escalation of another fault. */

/* Memory Management Fault Status Register */
#define SCB_MMFSR_IACCVIOL              (unsigned char)  (1 << 0)                                     /* Instruction access violation.  No address in MMFAR */
#define SCB_MMFSR_DACCVIOL              (unsigned char)  (1 << 1)                                     /* Data access violation.  Address in MMFAR */
#define SCB_MMFSR_MUNSTKERR             (unsigned char)  (1 << 3)                                     /* Fault on unstacking from exception.  No address in MMAR */
#define SCB_MMFSR_MSTKERR               (unsigned char)  (1 << 4)                                     /* Fault on stacking for exception.  No address in MMFAR */
#define SCB_MMFSR_MLSPERR               (unsigned char)  (1 << 5)                                     /* Fault during FP lazy state preservation. */
#define SCB_MMFSR_MMARVALID             (unsigned char)  (1 << 7)                                     /* MMFAR holds valid address */

/* Bus Fault Status Register */
#define SCB_BFSR_IBUSERR                (unsigned char)  (1 << 0)                                     /* Instruction bus error.  No address in BFAR */
#define SCB_BFSR_PRECISERR              (unsigned char)  (1 << 1)                                     /* Precise data bus error.  Address in BFAR */
#define SCB_BFSR_IMPRECISERR            (unsigned char)  (1 << 2)                                     /* Imprecise data bus error.  No address in BFAR */
#define SCB_BFSR_UNSTKERR               (unsigned char)  (1 << 3)                                     /* Fault on unstacking from exception.  No address in BFAR */
#define SCB_BFSR_STKERR                 (unsigned char)  (1 << 4)                                     /* Fault on stacking for exception.  No address in BFAR */
#define SCB_BFSR_LSPERR                 (unsigned char)  (1 << 5)                                     /* Fault on FP lazy state preservation. */
#define SCB_BFSR_BFARVALID              (unsigned char)  (1 << 7)                                     /* BFAR holds valid address */

/* Usage Fault Status Register */
#define SCB_UFSR_UNDEFINSTR             (unsigned short) (1 << 0)                                     /* Undefined instruction */
#define SCB_UFSR_INVSTATE               (unsigned short) (1 << 1)                                     /* Invalid state - PC stacked for exception return attempts illegal use of epsr */
#define SCB_UFSR_INVPC                  (unsigned short) (1 << 2)                                     /* Invalid PC load */
#define SCB_UFSR_NOCP                   (unsigned short) (1 << 3)                                     /* No coprocessor */
#define SCB_UFSR_UNALIGNED              (unsigned short) (1 << 8)                                     /* Unaligned access */
#define SCB_UFSR_DIVBYZERO              (unsigned short) (1 << 9)                                     /* Divide by zero */

#define SCB_CPACR_CP10_FULL             (unsigned int) (0x3 << 20)                                  /* Access privileges for coprocessor 10 (FPU) */
#define SCB_CPACR_CP11_FULL             (unsigned int) (0x3 << 22)                                  /* Access privileges for coprocessor 11 (FPU) */

/* Memory Protection Unit */
/* See pg. 183 in STM32F4 Prog Ref (PM0214) */
#define MPU_CTRL_ENABLE                 (unsigned int) (1 << 0)                                     /* Enables MPU */
#define MPU_CTRL_HFNMIENA               (unsigned int) (1 << 1)                                     /* Enables MPU during Hardfault, NMI, and Faultmask handlers */
#define MPU_CTRL_PRIVDEFENA             (unsigned int) (1 << 2)                                     /* Enable privileged software access to default memory map */

#define MPU_RASR_ENABLE                 (unsigned int) (1 << 0)                                     /* Enable region */
#define MPU_RASR_SIZE(x)                (unsigned int) (x << 1)                                     /* Region size (2^(x+1) bytes) */
#define MPU_RASR_SHARE_CACHE_WBACK      (unsigned int) (1 << 16) | (1 << 17) | (1 << 18)            /* Sharable, Cachable, Write-Back */
#define MPU_RASR_SHARE_NOCACHE_WBACK    (unsigned int) (1 << 16) | (0 << 17) | (1 << 18)            /* Sharable, Not Cachable, Write-Back */
#define MPU_RASR_NORMAL_CACHE		(unsigned int) (1 << 17)				    /* Normal memory, Cachable, Write-through*/
#define MPU_RASR_SHARE_CACHE		(unsigned int) (1 << 17) | (1 << 18)			    /* Sharable, Cachable, Write-through*/
#define MPU_RASR_DEVICE_SHARE		(unsigned int) (1 << 16) | (1 << 18)			    /* Device memory, Sharable, Write-through*/
#define MPU_RASR_AP_PRIV_NO_UN_NO       (unsigned int) (0 << 24)                                    /* No access for any */
#define MPU_RASR_AP_PRIV_RW_UN_NO       (unsigned int) (1 << 24)                                    /* No access for any */
#define MPU_RASR_AP_PRIV_RW_UN_RO       (unsigned int) (2 << 24)                                    /* Unprivileged Read Only Permissions */
#define MPU_RASR_AP_PRIV_RW_UN_RW       (unsigned int) (3 << 24)                                    /* All RW Permissions */
#define MPU_RASR_AP_PRIV_RO_UN_NO       (unsigned int) (5 << 24)                                    /* Privileged RO Permissions, Unpriv no access */
#define MPU_RASR_AP_PRIV_RO_UN_RO       (unsigned int) (6 << 24)                                    /* All RO Permissions */
#define MPU_RASR_XN                     (unsigned int) (1 << 28)                                    /* MPU Region Execute Never */

/* Floating Point Unit (FPU)
 * ST PM0214 (Cortex M4 Programming Manual) pg. 236 */
#define FPU_CCR_ASPEN                   (unsigned int) (1 << 31)                                    /* FPU Automatic State Preservation */

#define DWT_CTRL			(volatile unsigned int) (DWT_BASE + 0x00)

#define DWT_CYCCNT			(volatile unsigned int) (DWT_BASE + 0x04)
#define DWT_CPICNT			(volatile unsigned int) (DWT_BASE + 0x08)
#define DWT_EXCCNT			(volatile unsigned int) (DWT_BASE + 0x0C)
#define DWT_SLEEPCNT			(volatile unsigned int) (DWT_BASE + 0x10)
#define DWT_LSUCNT			(volatile unsigned int) (DWT_BASE + 0x14)
#define DWT_FOLDCNT			(volatile unsigned int) (DWT_BASE + 0x18)

#define DWT_PCSR			(volatile unsigned int) (DWT_BASE + 0x1C)
#define DWT_COMP(n)			(volatile unsigned int) (DWT_BASE + 0x20 + (n) * 16)
#define DWT_MASK(n)			(volatile unsigned int) (DWT_BASE + 0x24 + (n) * 16)
#define DWT_FUNCTION(n)			(volatile unsigned int) (DWT_BASE + 0x28 + (n) * 16)

#define DWT_CTRL_NUMCOMP_SHIFT		28
#define DWT_CTRL_NUMCOMP		(0x0F << DWT_CTRL_NUMCOMP_SHIFT)
#define DWT_CTRL_NOTRCPKT		(1 << 27)
#define DWT_CTRL_NOEXTTRIG		(1 << 26)
#define DWT_CTRL_NOCYCCNT		(1 << 25)
#define DWT_CTRL_NOPRFCCNT		(1 << 24)
#define DWT_CTRL_CYCEVTENA		(1 << 22)
#define DWT_CTRL_FOLDEVTENA		(1 << 21)
#define DWT_CTRL_LSUEVTENA		(1 << 20)
#define DWT_CTRL_SLEEPEVTENA		(1 << 19)
#define DWT_CTRL_EXCEVTENA		(1 << 18)
#define DWT_CTRL_CPIEVTENA		(1 << 17)
#define DWT_CTRL_EXCTRCENA		(1 << 16)
#define DWT_CTRL_PCSAMPLENA		(1 << 12)
#define DWT_CTRL_SYNCTAP_SHIFT		10
#define DWT_CTRL_SYNCTAP		(3 << DWT_CTRL_SYNCTAP_SHIFT)
#define DWT_CTRL_SYNCTAP_DISABLED	(0 << DWT_CTRL_SYNCTAP_SHIFT)
#define DWT_CTRL_SYNCTAP_BIT24		(1 << DWT_CTRL_SYNCTAP_SHIFT)
#define DWT_CTRL_SYNCTAP_BIT26		(2 << DWT_CTRL_SYNCTAP_SHIFT)
#define DWT_CTRL_SYNCTAP_BIT28		(3 << DWT_CTRL_SYNCTAP_SHIFT)
#define DWT_CTRL_CYCTAP			(1 << 9)
#define DWT_CTRL_POSTCNT_SHIFT		5
#define DWT_CTRL_POSTCNT		(0x0F << DWT_CTRL_POSTCNT_SHIFT)
#define DWT_CTRL_POSTPRESET_SHIFT	1
#define DWT_CTRL_POSTPRESET		(0x0F << DWT_CTRL_POSTPRESET_SHIFT)
#define DWT_CTRL_CYCCNTENA		(1 << 0)

#define DWT_MASKx_MASK			0x0F

#define DWT_FUNCTIONx_MATCHED		(1 << 24)
#define DWT_FUNCTIONx_DATAVADDR1_SHIFT	16
#define DWT_FUNCTIONx_DATAVADDR1	(15 << DWT_FUNCTIONx_DATAVADDR1_SHIFT)
#define DWT_FUNCTIONx_DATAVADDR0_SHIFT	12
#define DWT_FUNCTIONx_DATAVADDR0	(15 << DWT_FUNCTIONx_DATAVADDR0_SHIFT)
#define DWT_FUNCTIONx_DATAVSIZE_SHIFT	10
#define DWT_FUNCTIONx_DATAVSIZE		(3 << DWT_FUNCTIONx_DATAVSIZE_SHIFT)
#define DWT_FUNCTIONx_DATAVSIZE_BYTE	(0 << DWT_FUNCTIONx_DATAVSIZE_SHIFT)
#define DWT_FUNCTIONx_DATAVSIZE_HALF	(1 << DWT_FUNCTIONx_DATAVSIZE_SHIFT)
#define DWT_FUNCTIONx_DATAVSIZE_WORD	(2 << DWT_FUNCTIONx_DATAVSIZE_SHIFT)
#define DWT_FUNCTIONx_DATAVMATCH	(1 << 8)
#define DWT_FUNCTIONx_CYCMATCH		(1 << 7)
#define DWT_FUNCTIONx_EMITRANGE		(1 << 5)
#define DWT_FUNCTIONx_FUNCTION				15
#define DWT_FUNCTIONx_FUNCTION_DISABLED			0

#define SCS_DEMCR_TRCENA	(1 << 24)

static inline void pendsv_request(void)
{
	unsigned int val = readl(SCB_ICSR);

	val |= SCB_ICSR_PENDSVSET;
	writel(SCB_ICSR, val);
}

static inline void enable_deepsleep(void)
{
	unsigned int val = readl(SCB_SCR);

	val |= SCB_SCR_SLEEPDEEP;
	writel(SCB_SCR, val);
}

static inline void disable_deepsleep(void)
{
	unsigned int val = readl(SCB_SCR);

	val &= ~SCB_SCR_SLEEPDEEP;
	writel(SCB_SCR, val);
}

static inline void arch_sys_reboot(void)
{
	unsigned val = readl(SCB_AIRCR);

	val = ((0x5FA << SCB_AIRCR_VECTKEY_SHIFT)
			| (val & SCB_AIRCR_PRIGROUP_MASK)
			| SCB_AIRCR_SYSRESETREQ); /*  Keep priority group unchanged */

	writel(SCB_AIRCR, val);

	__dsb();

	while(1)
		;
}

#endif /* ARMV7M_SYSTEM_H */
