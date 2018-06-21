#include <armv7m/system.h>
#include <armv7m/nvic.h>
#include <utils.h>

#ifdef CONFIG_STM32F429
#define SYS_CLOCK	180000000
#define SYSTICK_FREQ	1000//4000
#else
#define SYS_CLOCK	84000000
#define SYSTICK_FREQ	1000//4000
#endif /* CONFIG_STM32F429 */

int systick_init(void)
{
	int ret = 0;

	writel(SYSTICK_RELOAD, SYS_CLOCK / SYSTICK_FREQ);
	writel(SYSTICK_VAL, 0);
	writel(SYSTICK_CTL, 0x00000007);

	/* Set PendSV and SVC to lowest priority.
	* This means that both will be deferred
	* until all other exceptions have executed.
	* Additionally, PendSV will not interrupt
	* an SVC. */

	nvic_set_priority_interrupt(systick_irq, 0xFE);
	nvic_set_priority_interrupt(svcall_irq, 0xFF);
	nvic_set_priority_interrupt(pendsv_irq, 0xFF);

	return ret;
}
