#include <board.h>
#include <errno.h>
#include <drv/timer.h>
#include <stddef.h>
#include <armv7m/nvic.h>
#include <armv7m/vector.h>
#include <drv/irq.h>
#include <mm/mm.h>
#include <fdtparse.h>
#include <init.h>
#include <kernel/printk.h>
#include <drv/device.h>
#include <mach/rcc-stm32.h>
#include <mach/exti-stm32.h>

#define RTC_WKUP_FREQ	16000

struct irq_callback {
	void (*callback)(void *arg);
	void *arg;
};

static struct irq_callback irq_callback_infos;

extern unsigned int system_tick;

static void stm32_rtc_isr(void *arg)
{
	struct timer *timer = (struct timer *)arg;
	RTC_TypeDef *rtc = (RTC_TypeDef *)timer->base_reg;

	rtc->ISR &= ~RTC_ISR_WUTF;
	stm32_exti_clear_line(22);

	system_tick += timer->counter;

	irq_callback_infos.callback(irq_callback_infos.arg);
}

static short stm32_rtc_find_best_pres(unsigned long parent_rate, unsigned long rate)
{
	unsigned int i;
	unsigned short pres[] = {2, 4, 8, 16};
	unsigned int diff;
	unsigned int best_diff;
	unsigned long curr_rate;

	best_diff = parent_rate - rate;

	for (i = 0; i < 4; i++) {
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

	return 4 - i - 1;
}


static void stm32_rtc_set_rate(struct timer *timer, unsigned long rate)
{
	RTC_TypeDef *rtc = (RTC_TypeDef *)timer->base_reg;
	unsigned short pres;

	pres = stm32_rtc_find_best_pres(timer->rate, rate);

	timer->rate = rate;

	rtc->CR &= ~RTC_CR_WUCKSEL;
	rtc->CR |= pres;

}

static void stm32_rtc_set_counter(struct timer *timer, unsigned short counter)
{
	int n;
	RTC_TypeDef *rtc = (RTC_TypeDef *)timer->base_reg;

	stm32_rtc_set_rate(timer, RTC_WKUP_FREQ);

	n = counter / (1000 / (timer->rate / 1000));
	n--;

	rtc->WUTR = n;
}

static void stm32_rtc_enable(struct timer *timer)
{
	RTC_TypeDef *rtc = (RTC_TypeDef *)timer->base_reg;
	int nvic = timer->irq;

	nvic_enable_interrupt(nvic);

	rtc->CR |= RTC_CR_WUTE;
}

static void stm32_rtc_disable(struct timer *timer)
{
	RTC_TypeDef *rtc = (RTC_TypeDef *)timer->base_reg;
	int nvic = timer->irq;

	nvic_disable_interrupt(nvic);

	rtc->CR &= ~RTC_CR_WUTE;
}

static int stm32_rtc_request_irq(struct timer *timer, void (*handler)(void *), void *arg)
{
	irq_callback_infos.callback = handler;
	irq_callback_infos.arg = arg;

	return 0;
}

static int stm32_rtc_release_irq(struct timer *timer)
{
	memset(&irq_callback_infos, 0, sizeof(struct irq_callback));
	return 0;
}

struct timer_operations rtc_ops = {
	.set_counter = stm32_rtc_set_counter,
	.enable = stm32_rtc_enable,
	.disable = stm32_rtc_disable,
	.request_irq = stm32_rtc_request_irq,
	.release_irq = stm32_rtc_release_irq,
};

static int stm32_rtc_of_init(struct timer *timer)
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

out:
	return ret;
}

static int stm32_rtc_init(struct device *dev)
{
	int ret = 0;
	struct timer *timer = NULL;
	RTC_TypeDef *rtc = NULL;

	timer = timer_new();
	if (!timer) {
		error_printk("failed to retrieve new timer\n");
		ret = -EIO;
		goto err;
	}

	memcpy(&timer->dev, dev, sizeof(struct device));
	timer->tim_ops = &rtc_ops;

	ret = stm32_rtc_of_init(timer);
	if (ret < 0) {
		error_printk("failed to init timer with fdt data\n");
		goto free_timer;
	}

	RCC->BDCR |= (2 << 8);

	rtc = (RTC_TypeDef *)timer->base_reg;

	rtc->WPR = 0xCA;
	rtc->WPR = 0x53;

	rtc->CR &= ~RTC_CR_WUTE;

	while (!(rtc->ISR & RTC_ISR_WUTWF))
		;

	rtc->ISR &= ~RTC_ISR_WUTF;

	rtc->CR |= RTC_CR_WUTIE;

	timer->rate = timer->clock.source_clk;

	stm32_exti_configure(22, IRQF_RISING);

	ret = irq_request(timer->irq, &stm32_rtc_isr, timer);
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
		error_printk("failed to register stm32 timer\n");
		goto clk_disable;
	}

	return ret;

clk_disable:
	stm32_rcc_disable_clk(timer->clock.gated, timer->clock.id);
free_timer:
	kfree(timer);
err:
	return ret;
}

struct device stm32_rtc_driver = {
	.of_compat = "st,stm32f4xx-rtc",
	.probe = stm32_rtc_init,
};

static int stm32_rtc_register(void)
{
	int ret = 0;

	ret = device_of_register(&stm32_rtc_driver);
	if (ret < 0)
		error_printk("failed to register stm32_rtc device\n");
	return ret;
}
postarch_initcall(stm32_rtc_register);
