#include <board.h>
#include <kernel/printk.h>
#include <utils.h>
#include <kernel/ktime.h>
#include <kernel/scheduler.h>
#include <kernel/syscall.h>
#include <armv7m/system.h>
#include <armv7m/thread.h>
#include <arch/system.h>
#include <symbols.h>
#include <backtrace.h>

static void __attribute__((optimize("O0"))) dump_stack(unsigned int *stack)
{
	volatile unsigned int lr;
	volatile unsigned int pc;

	lr = stack[5];
	pc = stack[6];

#ifdef CONFIG_UNWIND
	/* fp = 0, because we don't care about it */
	unwind_backtrace(0, (unsigned int)stack, lr, pc);
#endif /* CONFIG_UNWIND */

	while (1)
		;
}


void __attribute__((optimize("O0"))) hardfault_handler(void)
{
	asm volatile (
		"tst lr, #4			\n"
		"ite eq				\n"
		"mrseq r0, msp			\n"
		"mrsne r0, psp			\n"
//		"ldr r1, [r0, #24]		\n"
//		"ldr r2, _dump_stack		\n"
		"b dump_stack			\n"
//		"_dump_stack: .word dump_stack	\n"
	);	

	while (1)
		;
}

void memmanage_handler(void)
{
	while (1)
		;
}

void busfault_handler(void)
{
	while (1)
		;
}

void usagefault_handler(void)
{
	while (1)
		;
}

void systick_handler(void)
{
	system_tick++;

#if !defined(CONFIG_HR_TIMER) && !defined(CONFIG_BW_DELAY) && !defined(CONFIG_TICKLESS)
	decrease_thread_delay();
#endif

	decrease_timer_delay();
}

void pendsv_handler(void)
{
	schedule_thread(NULL);
}

int svc_handler(unsigned int svc_number, void *arg1, void *arg2, void *arg3)
{
	void *ret;
	void * (*handler)(void *, void *, void *);
	unsigned char type;

	debug_printk("svc_handler: got call %d\r\n", svc_number);

	handler = (void (*))syscall_table[svc_number].handler;
	type = syscall_table[svc_number].type;

	ret = (*handler)(arg1, arg2, arg3);

	if (type == SYSCALL_PRIVILEGE_ELEVATION)
		arch_thread_switch_unpriv();
	else
		arch_thread_set_return(ret);


	return (int)ret;
}
