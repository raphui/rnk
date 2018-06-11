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

	mpu_init();

	entry.isr = (void *)__svc;
	entry.arg = NULL;

	vector_set_isr_entry(&entry, svcall_irq);

	entry.isr = (void *)__pendsv;
	vector_set_isr_entry(&entry, pendsv_irq);

	entry.isr = (void *)systick_handler;
	vector_set_isr_entry(&entry, systick_irq);


	/* Set PendSV and SVC to lowest priority.
	* This means that both will be deferred
	* until all other exceptions have executed.
	* Additionally, PendSV will not interrupt
	* an SVC. */

	nvic_set_priority_interrupt(systick_irq, 0x3);
	nvic_set_priority_interrupt(svcall_irq, 0x7);
	nvic_set_priority_interrupt(pendsv_irq, 0x7);

	for (i = 0; i < CONFIG_NUM_IRQS; i++)
		nvic_set_priority_interrupt(i, 0x3);
}

void arch_init_tick(void)
{
	systick_init();
}
