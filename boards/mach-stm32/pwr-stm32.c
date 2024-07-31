#include <board.h>
#include <utils.h>
#include <mm/mm.h>
#include <kernel/printk.h>
#include <drv/device.h>
#include <mach/rcc-stm32.h>
#include <armv7m/nvic.h>
#include <fdtparse.h>
#include <init.h>
#include <errno.h>
#include <ioctl.h>
#include <drv/clk.h>

struct pwr {
	struct device dev;
	unsigned int enabled_clock_ahb1;
	unsigned int enabled_clock_ahb2;
	unsigned int enabled_clock_ahb3;
	unsigned int enabled_clock_apb1enr1;
	unsigned int enabled_clock_apb1enr2;
	unsigned int enabled_clock_apb2;
};

static struct pwr *pwr = NULL;

void stm32_pwr_set_regulator(int freq)
{
	int reg = PWR->CR1;

	reg &= ~PWR_CR1_VOS_Msk;

	if (freq > 26000000) {
		reg |= PWR_CR1_VOS_0;
	} else {
		reg |= PWR_CR1_VOS_1;
	
	}

	PWR->CR1 = reg;

	while (PWR->SR2 & PWR_SR2_VOSF)
		;

	if (freq < 2000000) {
		PWR->CR1 |= PWR_CR1_LPR;
	} else {
		PWR->CR1 &= ~PWR_CR1_LPR;

		while (PWR->SR2 & PWR_SR2_REGLPF)
			;
	}
}

void stm32_pwr_enter_lpsleep(int mode)
{
	unsigned int discard;

	PWR->CR1 |= mode;

	/* XXX: save enabled APB1 and APB2 periph clock so we will restore them when waking up */
	pwr->enabled_clock_ahb1 = RCC->AHB1ENR;
	pwr->enabled_clock_ahb2 = RCC->AHB2ENR;
	pwr->enabled_clock_ahb3 = RCC->AHB3ENR;
	pwr->enabled_clock_apb1enr1 = RCC->APB1ENR1;
	pwr->enabled_clock_apb1enr2 = RCC->APB1ENR2;
	pwr->enabled_clock_apb2 = RCC->APB2ENR;

#if 1
	/* XXX: disable these clocks but keep ones we really need
	 * FIXME: (make it clever than hardcoded value)
	 */
	discard = pwr->enabled_clock_apb1enr1;
	discard &= ~(RCC_APB1ENR1_LPTIM1EN | RCC_APB1ENR1_PWREN);
	RCC->APB1ENR1 &= ~discard;

	discard = pwr->enabled_clock_apb1enr2;
	RCC->APB1ENR2 &= ~discard;

	discard = pwr->enabled_clock_apb2;
	discard &= ~(RCC_APB2ENR_SYSCFGEN);
	RCC->APB2ENR &= ~discard;

	discard = pwr->enabled_clock_ahb1;
	discard &= ~(RCC_AHB1ENR_FLASHEN);
	RCC->AHB1ENR &= ~discard;

	discard = pwr->enabled_clock_ahb2;
	discard &= ~(RCC_AHB2ENR_GPIOAEN);
	RCC->AHB2ENR &= ~discard;

	discard = pwr->enabled_clock_ahb3;
	RCC->AHB3ENR &= ~discard;

#else
	RCC->APB1SMENR1 = RCC_APB1SMENR1_LPTIM1SMEN | RCC_APB1SMENR1_PWRSMEN;
	RCC->APB1SMENR2 = 0;
	RCC->APB2SMENR = RCC_APB2SMENR_SYSCFGSMEN;
#endif

}

void stm32_pwr_exit_lpsleep(int mode)
{
	PWR->CR1 &= ~(mode | PWR_CR1_LPR);

	RCC->AHB1ENR = pwr->enabled_clock_ahb1;
	RCC->AHB2ENR = pwr->enabled_clock_ahb2;
	RCC->AHB3ENR = pwr->enabled_clock_ahb3;
	RCC->APB1ENR1 = pwr->enabled_clock_apb1enr1;
	RCC->APB1ENR2 = pwr->enabled_clock_apb1enr2;
	RCC->APB2ENR = pwr->enabled_clock_apb2;
}

static int stm32_pwr_of_init(struct pwr *pwr)
{
	int offset;
	int ret = 0;
	struct clk clock;
	const void *fdt_blob = fdtparse_get_blob();

	offset = fdt_path_offset(fdt_blob, pwr->dev.of_path);
	if (offset < 0) {
		ret = -ENOENT;
		goto out;
	}

	ret = fdt_node_check_compatible(fdt_blob, offset, pwr->dev.of_compat);
	if (ret < 0)
		goto out;

	ret = stm32_rcc_of_enable_clk(offset, &clock);
	if (ret < 0) {
		error_printk("failed to retrieve pwr clock\n");
		ret = -EIO;
	}

out:
	return ret;
}

static int stm32_pwr_init(struct device *dev)
{
	int ret = 0;

	pwr = kmalloc(sizeof(*pwr));
	if (!pwr) {
		error_printk("failed to allocate pwr structure\n");
		ret = ENOMEM;
		goto err;
	}

	memcpy(&pwr->dev, dev, sizeof(struct device));

	ret = stm32_pwr_of_init(pwr);
	if (ret < 0) {
		error_printk("failed to init pwr with fdt data\n");
		goto err_alloc;
	}

	return 0;

err_alloc:
	kfree(pwr);
err:
	return ret;
}

struct device stm32_pwr_driver = {
	.of_compat = "st,stm32-pwr",
	.probe = stm32_pwr_init,
};

static int stm32_pwr_register(void)
{
	int ret = 0;

	ret = device_of_register(&stm32_pwr_driver);
	if (ret < 0)
		error_printk("failed to register stm32_pwr device\n");
	return ret;
}
postarch_initcall(stm32_pwr_register);
