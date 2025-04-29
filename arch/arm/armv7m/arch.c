#include <armv7m/mpu.h>
#include <armv7m/system.h>
#include <armv7m/vector.h>
#include <armv7m/nvic.h>
#include <drv/clk.h>
#include <board.h>
#include <stddef.h>

extern void __svc(void);
extern void __pendsv(void);
extern void systick_handler(void);
extern void lowlevel_delay(int delay);

void arch_init(void)
{
	struct isr_entry entry;

#ifdef CONFIG_USER
	mpu_init();
#endif

	entry.isr = (void *)__svc;
	entry.arg = NULL;

	vector_set_isr_entry(&entry, svcall_irq);

	entry.isr = (void *)__pendsv;
	vector_set_isr_entry(&entry, pendsv_irq);

	entry.isr = (void *)systick_handler;
	vector_set_isr_entry(&entry, systick_irq);
}

void arch_init_tick(void)
{
#ifndef CONFIG_TICKLESS
	systick_init();
#endif
}

void arch_idle(int low_power)
{
	__disable_it();
#ifdef CONFIG_LOW_POWER
	if (low_power) {
		systick_disable_it();
		board_enter_low_power();
		enable_deepsleep();
	}
#endif
	__dsb();
	wait_for_interrupt();
	__isb();
#ifdef CONFIG_LOW_POWER
	if (low_power) {
		disable_deepsleep();
		board_exit_low_power();
		systick_enable_it();
	}
#endif
	__enable_it();
	__dsb();
	__isb();
}

void arch_lowlevel_delay(int delay)
{
        int cycles = delay * (clk_get_sysfreq() / 1000000);
        if (cycles > 60)
                lowlevel_delay((cycles - 60) / 4);
}
