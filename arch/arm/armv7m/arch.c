#include <armv7m/mpu.h>
#include <armv7m/system.h>
#include <armv7m/vector.h>
#include <stddef.h>

extern void __svc(void);
extern void __pendsv(void);
extern void systick_handler(void);

void arch_init(void)
{
	struct isr_entry entry;

	mpu_init();

	entry.isr = (void *)__svc;
	entry.arg = NULL;

	vector_set_isr_entry(&entry, -5);

	entry.isr = (void *)__pendsv;
	vector_set_isr_entry(&entry, -2);

	entry.isr = (void *)systick_handler;
	vector_set_isr_entry(&entry, -1);
}

void arch_init_tick(void)
{
	systick_init();
}
