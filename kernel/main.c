#include <kernel/printk.h>
#include <kernel/scheduler.h>
#include <kernel/thread.h>
#include <mm/mm.h>
#include <kernel/ktime.h>
#include <elf/elfloader.h>
#include <arch/system.h>
#include <arch/spinlock.h>

#ifdef CONFIG_UNWIND
#include <backtrace.h>
#endif /* CONFIG_UNWIND */

#include <init.h>
#include <trace.h>

extern initcall_t __rnk_initcalls_start[], __rnk_initcalls_end[];
extern exitcall_t __rnk_exitcalls_start[], __rnk_exitcalls_end[];

int main(void)
{
	int ret;
	unsigned long irqstate;
	initcall_t *initcall;

	arch_interrupt_save(&irqstate, SPIN_LOCK_FLAG_IRQ);

	printk("Welcome to rnk\n");

	printk("- Initialise architecture...\n");

#ifdef CONFIG_TRACE
	trace_enable();
#endif /* CONFIG_TRACE */

	arch_init();

	for (initcall = __rnk_initcalls_start; initcall < __rnk_initcalls_end; initcall++) {
		debug_printk("initcall-> %pS\n", *initcall);
		ret = (*initcall)();
		if (ret < 0)
			error_printk("initcall %pS failed: %d\n", *initcall, ret);
	}

	printk("- Initialise scheduler...\n");

#ifdef CONFIG_UNWIND
	unwind_init();
#endif /* CONFIG_UNWIND */

	printk("- Loading app\n");

	ret = elf_exec((char *)0x08020000);
	if (ret <= 0) {
		printk("failed to exec elf\n");
		goto fail;
	}
	else
		printk("efl execution done\n");

	printk("- Add app thread to scheduler\n");

#ifdef CONFIG_USER
	add_thread((void *)ret, NULL, HIGHEST_PRIORITY, USER_THREAD);
#else
	add_thread((void *)ret, NULL, HIGHEST_PRIORITY, PRIVILEGED_THREAD);
#endif

	printk("- Start scheduling...\n");
	start_schedule();

	arch_interrupt_restore(irqstate, SPIN_LOCK_FLAG_IRQ);

fail:
	while(1)
		;

	return 0; //Never reach
}
