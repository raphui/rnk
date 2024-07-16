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
#include <drv/timer.h>
#include <drv/irq.h>

struct irq_callback {
	void (*callback)(void *arg);
	void *arg;
};

static struct irq_callback irq_callback_infos;

extern unsigned int system_tick;

static void stm32_lptim_isr(void *arg)
{
	struct timer *timer = (struct timer *)arg;
	LPTIM_TypeDef *lptim = (LPTIM_TypeDef *)timer->base_reg;

	lptim->ICR = 0x7F;

	system_tick += timer->counter;// * timer->rate);
 
	irq_callback_infos.callback(irq_callback_infos.arg);
}

static short stm32_lptim_find_best_pres(unsigned long parent_rate, unsigned long rate, unsigned int *prescaler)
{
	unsigned int i;
	unsigned short pres[] = {1, 2, 4, 8, 16, 32, 64, 128};
	unsigned int diff;
	unsigned int best_diff;
	unsigned long curr_rate;

	best_diff = parent_rate - rate;

	for (i = 0; i < sizeof(pres) / sizeof(unsigned short); i++) {
		curr_rate = parent_rate / pres[i];

		if (curr_rate < rate)
			diff = rate - curr_rate;
		else
			diff = curr_rate - rate;

		if (diff < best_diff) {
			best_diff = diff;
		}

		if (!best_diff || curr_rate < rate)
			break;
	}

	*prescaler = pres[i];

	return i;
}


static void stm32_lptim_set_rate(struct timer *timer, unsigned long rate)
{
	LPTIM_TypeDef *lptim = (LPTIM_TypeDef *)timer->base_reg;
	unsigned short pres;

	pres = stm32_lptim_find_best_pres(timer->rate, rate, &timer->prescaler);

	timer->rate = 1000000 * timer->prescaler / timer->clock.source_clk;

	lptim->CFGR &= ~LPTIM_CFGR_PRESC_Msk;
	lptim->CFGR |= (pres << LPTIM_CFGR_PRESC_Pos);
}

static void stm32_lptim_set_counter(struct timer *timer, unsigned int counter)
{
	LPTIM_TypeDef *lptim = (LPTIM_TypeDef *)timer->base_reg;

	counter /= timer->rate;

	/* XXX: if counter is 0, that means we are under the lowest granularity from the timer
	 * so let's put the counter to 1.
	 */
	if (!counter)
		counter = 1;

	lptim->ARR = counter;

	while (!(lptim->ISR & LPTIM_ISR_ARROK))
		;

	lptim->CMP = counter - 1;

	while (!(lptim->ISR & LPTIM_ISR_CMPOK))
		;
}

static void stm32_lptim_enable(struct timer *timer)
{
	LPTIM_TypeDef *lptim = (LPTIM_TypeDef *)timer->base_reg;
	int nvic = timer->irq;

	nvic_enable_interrupt(nvic);

	lptim->CR = LPTIM_CR_ENABLE | LPTIM_CR_SNGSTRT | LPTIM_CR_CNTSTRT;
}

static void stm32_lptim_disable(struct timer *timer)
{
	LPTIM_TypeDef *lptim = (LPTIM_TypeDef *)timer->base_reg;
	int nvic = timer->irq;

	nvic_disable_interrupt(nvic);

	lptim->CR = (1 << 3) | LPTIM_CR_CNTSTRT;
}

static int stm32_lptim_request_irq(struct timer *timer, void (*handler)(void *), void *arg)
{
	LPTIM_TypeDef *lptim = (LPTIM_TypeDef *)timer->base_reg;

	irq_callback_infos.callback = handler;
	irq_callback_infos.arg = arg;

	lptim->CR = LPTIM_CR_ENABLE;

	return 0;
}

static int stm32_lptim_release_irq(struct timer *timer)
{
	memset(&irq_callback_infos, 0, sizeof(struct irq_callback));
	return 0;
}

struct timer_operations lptim_ops = {
	.set_counter = stm32_lptim_set_counter,
	.enable = stm32_lptim_enable,
	.disable = stm32_lptim_disable,
	.request_irq = stm32_lptim_request_irq,
	.release_irq = stm32_lptim_release_irq,
};


static int stm32_lptim_of_init(struct timer *timer)
{
	int offset;
	int ret = 0;
	const void *fdt_blob = fdtparse_get_blob();

	offset = fdt_path_offset(fdt_blob, timer->dev.of_path);
	if (offset < 0) {
		ret = -ENOENT;
		goto out;
	}

	ret = fdt_node_check_compatible(fdt_blob, offset, timer->dev.of_compat);
	if (ret < 0)
		goto out;

	timer->base_reg = (unsigned int)fdtparse_get_addr32(offset, "reg");
	if (!timer->base_reg) {
		error_printk("failed to retrieve rtc base reg from fdt\n");
		ret = -ENOENT;
		goto out;
	}

	ret = fdtparse_get_int(offset, "interrupts", (int *)&timer->irq);
	if (ret < 0) {
		error_printk("failed to retrieve timer irq\n");
		ret = -EIO;
		goto out;
	}

	ret = stm32_rcc_of_enable_clk(offset, &timer->clock);
	if (ret < 0) {
		error_printk("failed to retrieve lptim clock\n");
		ret = -EIO;
	}

out:
	return ret;
}

static int stm32_lptim_init(struct device *dev)
{
	int ret = 0;
	struct timer *timer = NULL;
	LPTIM_TypeDef *lptim = NULL;

	timer = timer_new();
	if (!timer) {
		error_printk("failed to retrieve new timer\n");
		ret = -EIO;
		goto err;
	}

	memcpy(&timer->dev, dev, sizeof(struct device));
	timer->tim_ops = &lptim_ops;

	ret = stm32_lptim_of_init(timer);
	if (ret < 0) {
		error_printk("failed to init lptim with fdt data\n");
		goto err_alloc;
	}

	lptim = (LPTIM_TypeDef *)timer->base_reg;

	timer->rate = timer->clock.source_clk;

	stm32_lptim_set_rate(timer, 125000);//1000000);//32000);

	lptim->IER |= LPTIM_IER_CMPMIE;

	lptim->CR = LPTIM_CR_ENABLE;

	lptim->ICR = 0x7F;

	ret = irq_request(timer->irq, &stm32_lptim_isr, timer);
	if (ret < 0) {
		error_printk("cannot request isr for irq line: %d\n", timer->irq);
		goto clk_disable;
	}
	
#ifdef CONFIG_TICKLESS
	ret = timer_lp_register(timer);
#else
	ret = timer_register(timer);
#endif
	if (ret < 0) {
		error_printk("failed to register lptim timer\n");
		goto clk_disable;
	}

	return 0;

clk_disable:
	stm32_rcc_disable_clk(timer->clock.gated, timer->clock.id);
err_alloc:
	kfree(timer);
err:
	return ret;
}

struct device stm32_lptim_driver = {
	.of_compat = "st,stm32-lptim",
	.probe = stm32_lptim_init,
};

static int stm32_lptim_register(void)
{
	int ret = 0;

	ret = device_of_register(&stm32_lptim_driver);
	if (ret < 0)
		error_printk("failed to register stm32_lptim device\n");
	return ret;
}
postarch_initcall(stm32_lptim_register);
