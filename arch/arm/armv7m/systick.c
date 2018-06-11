#include <armv7m/system.h>
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

	return ret;
}
