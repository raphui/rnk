#include <armv7m/mpu.h>
#include <armv7m/system.h>
#include <armv7m/vector.h>
#include <armv7m/nvic.h>
#include <stddef.h>

extern void __svc(void);
extern void __pendsv(void);
extern void systick_handler(void);

void arch_init(void)
{
	int i;
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
	systick_init();
}

void arch_idle(void)
{
	__disable_it();

	__dsb();
	wait_for_interrupt();
	__isb();

	__enable_it();
	__dsb();
	__isb();
}
