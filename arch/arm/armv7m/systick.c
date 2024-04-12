#include <armv7m/system.h>
#include <drv/clk.h>
#include <utils.h>

int systick_init(void)
{
	int ret = 0;
	int sysfreq = clk_get_sysfreq();
	int systick_freq = 1000;

	writel(SYSTICK_RELOAD, sysfreq / systick_freq);
	writel(SYSTICK_VAL, 0);
	writel(SYSTICK_CTL, 0x00000007);

	return ret;
}

int systick_disable_it(void)
{
	int ret = 0;
	int val = readl(SYSTICK_CTL);

	val &= ~(1 << 1);

	writel(SYSTICK_CTL, val);

	return ret;
}

int systick_enable_it(void)
{
	int ret = 0;
	int val = readl(SYSTICK_CTL);

	val &= ~(1 << 1);

	writel(SYSTICK_CTL, val);

	return ret;
}
