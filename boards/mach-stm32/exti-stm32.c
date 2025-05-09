#include <board.h>
#include <utils.h>
#include <kernel/printk.h>
#include <armv7m/nvic.h>
#include <armv7m/vector.h>
#include <errno.h>
#include <drv/irq.h>
#include <mm/mm.h>
#include <init.h>
#include <string.h>

struct action {
	void (*irq_action)(void *);
	void *arg;
};

static struct action exti_callbacks[CONFIG_EXTI_LINES];

static int nvic_array[16] = {
	EXTI0_IRQn,
	EXTI1_IRQn,
	EXTI2_IRQn,
	EXTI3_IRQn,
	EXTI4_IRQn,
	EXTI9_5_IRQn,
	EXTI9_5_IRQn,
	EXTI9_5_IRQn,
	EXTI9_5_IRQn,
	EXTI9_5_IRQn,
	EXTI15_10_IRQn,
	EXTI15_10_IRQn,
	EXTI15_10_IRQn,
	EXTI15_10_IRQn,
	EXTI15_10_IRQn,
	EXTI15_10_IRQn,
};

static unsigned char stm32_exti_base_mask(unsigned int gpio_base)
{
	unsigned char mask = 0;

	switch (gpio_base) {
		case GPIOA_BASE:
			mask = 0x0;
			break;
		case GPIOB_BASE:
			mask = 0x1;
			break;
		case GPIOC_BASE:
			mask = 0x2;
			break;
#ifndef CONFIG_STM32L442
		case GPIOD_BASE:
			mask = 0x3;
			break;
		case GPIOE_BASE:
			mask = 0x4;
			break;
#endif
#ifdef CONFIG_STM32F4XX
		case GPIOF_BASE:
			mask = 0x5;
			break;
		case GPIOG_BASE:
			mask = 0x6;
			break;
#endif
		case GPIOH_BASE:
			mask = 0x7;
			break;
#ifdef CONFIG_STM32F4XX
		case GPIOI_BASE:
			mask = 0x8;
			break;
#endif
#ifdef CONFIG_STM32F429
		case GPIOJ_BASE:
			mask = 0x9;
			break;
		case GPIOK_BASE:
			mask = 0xA;
			break;
#endif /* CONFIG_STM32F429 */
		default:
			error_printk("invalid gpio base reg\r\n");
			mask = -EINVAL;
			break;
	}

	return mask;
}


static int stm32_exti_get_nvic_number(unsigned int gpio_num)
{
	int nvic = 0;

	if (gpio_num > 15) {
		error_printk("line %d is not configurable\r\n", gpio_num);
		return -EINVAL;
	} else {
		nvic = nvic_array[gpio_num];
	}

	return nvic;
}

static void stm32_exti_isr(void *arg)
{
	int i;
	void (*hook)(void *) = NULL;
#ifdef CONFIG_STM32F4XX
	int line = EXTI->PR & 0x7FFFFF;

	EXTI->PR = line;
#else
	int line = EXTI->PR1 & 0x7FFFFF;

	EXTI->PR1 = line;
#endif

	for (i = 0; i < 32; i++) {
		if (line & (1 << i)) {
			if (exti_callbacks[i].irq_action) {
				hook = exti_callbacks[i].irq_action;
				hook(exti_callbacks[i].arg);
			}
		}
	}
}

void stm32_exti_clear_line(unsigned int line)
{
#ifdef CONFIG_STM32F4XX
	EXTI->PR |= line;
#else
	EXTI->PR1 |= line;
#endif
}

int stm32_exti_configure(unsigned int line, unsigned int edge)
{
	int ret = 0;

#ifdef CONFIG_STM32F4XX
	/* clean any previous interrupt flags */
	if (edge & IRQF_RISING)
		EXTI->RTSR |= (1 << line);

	if (edge & IRQF_FALLING)
		EXTI->FTSR |= (1 << line);


	EXTI->IMR |= (1 << line);
#else
	/* clean any previous interrupt flags */
	if (edge & IRQF_RISING)
		EXTI->RTSR1 |= (1 << line);

	if (edge & IRQF_FALLING)
		EXTI->FTSR1 |= (1 << line);


	EXTI->IMR1 |= (1 << line);
#endif

	return ret;
}

int stm32_exti_configure_line(unsigned int gpio_base, unsigned int gpio_num)
{
	unsigned char mask = stm32_exti_base_mask(gpio_base);
	int ret = 0;

#ifdef CONFIG_STM32F429
	if ((gpio_base == GPIOK_BASE) && (gpio_num >= 8)) {
		error_printk("GPIOK %d is not configurable above pin 8\r\n", gpio_num);
		return -EINVAL;
	}
#endif /* CONFIG_STM32F429 */

	if (mask < 0) {
		error_printk("cannot retrieve exti base mask\r\n");
		return mask;
	}

	if (gpio_num <= 15) {
		SYSCFG->EXTICR[gpio_num >> 2] = (mask << ((gpio_num % 4) *  4));

#ifdef CONFIG_STM32F4XX
		/* clean any previous interrupt flags */
		EXTI->RTSR &= ~(1 << gpio_num);
		EXTI->FTSR &= ~(1 << gpio_num);
		EXTI->IMR &= ~(1 << gpio_num);
#else
		/* clean any previous interrupt flags */
		EXTI->RTSR1 &= ~(1 << gpio_num);
		EXTI->FTSR1 &= ~(1 << gpio_num);
		EXTI->IMR1 &= ~(1 << gpio_num);
#endif

	} else {
		error_printk("pin %d is not configurable\r\n", gpio_num);
		return -EINVAL;
	}

	return ret;
}

int stm32_exti_enable_falling(unsigned int gpio_base, unsigned int gpio_num)
{
	int nvic = stm32_exti_get_nvic_number(gpio_num);
	int ret = 0;

	if (nvic < 0) {
		error_printk("cannot enable it on line %d\r\n", gpio_num);
		return -EINVAL;
	}

#ifdef CONFIG_STM32F4XX
	EXTI->FTSR |= (1 << gpio_num);
	EXTI->IMR |= (1 << gpio_num);
#else
	EXTI->FTSR1 |= (1 << gpio_num);
	EXTI->IMR1 |= (1 << gpio_num);
#endif

	nvic_enable_interrupt(nvic);

	return ret;
}

int stm32_exti_enable_rising(unsigned int gpio_base, unsigned int gpio_num)
{
	int nvic = stm32_exti_get_nvic_number(gpio_num);
	int ret = 0;

	if (nvic < 0) {
		error_printk("cannot enable it on line %d\r\n", gpio_num);
		return -EINVAL;
	}

#ifdef CONFIG_STM32F4XX
	EXTI->RTSR |= (1 << gpio_num);
	EXTI->IMR |= (1 << gpio_num);
#else
	EXTI->RTSR1 |= (1 << gpio_num);
	EXTI->IMR1 |= (1 << gpio_num);
#endif

	nvic_enable_interrupt(nvic);

	return ret;
}

int stm32_exti_disable_falling(unsigned int gpio_base, unsigned int gpio_num)
{
	int nvic = stm32_exti_get_nvic_number(gpio_num);
	int ret = 0;

	if (nvic < 0) {
		error_printk("cannot disable it on line %d\r\n", gpio_num);
		return -EINVAL;
	}

#ifdef CONFIG_STM32F4XX
	EXTI->FTSR &= ~(1 << gpio_num);
	EXTI->IMR &= ~(1 << gpio_num);
#else
	EXTI->FTSR1 &= ~(1 << gpio_num);
	EXTI->IMR1 &= ~(1 << gpio_num);
#endif

	nvic_clear_interrupt(nvic);
	nvic_disable_interrupt(nvic);

	return ret;
}

int stm32_exti_disable_rising(unsigned int gpio_base, unsigned int gpio_num)
{
	int nvic = stm32_exti_get_nvic_number(gpio_num);
	int ret = 0;

	if (nvic < 0) {
		error_printk("cannot disable it on line %d\r\n", gpio_num);
		return -EINVAL;
	}

#ifdef CONFIG_STM32F4XX
	EXTI->RTSR &= ~(1 << gpio_num);
	EXTI->IMR &= ~(1 << gpio_num);
#else
	EXTI->RTSR1 &= ~(1 << gpio_num);
	EXTI->IMR1 &= ~(1 << gpio_num);
#endif

	nvic_clear_interrupt(nvic);
	nvic_disable_interrupt(nvic);

	return ret;
}

void stm32_exti_disable_interrupt(unsigned int gpio_base, unsigned int gpio_num)
{
	EXTI->IMR1 &= ~(1 << gpio_num);
}

void stm32_exti_enable_interrupt(unsigned int gpio_base, unsigned int gpio_num)
{
	EXTI->IMR1 |= (1 << gpio_num);
}

int stm32_exti_request_irq(unsigned int gpio_base, unsigned int gpio_num, void (*handler)(void *), int flags, void *arg)
{
	int ret = 0;

	ret = stm32_exti_configure_line(gpio_base, gpio_num);
	if (ret < 0) {
		error_printk("failed to init exti for gpio_base: %x gpio_num: %d\n", gpio_base, gpio_num);
		goto fail;
	}

	if (!(flags & (IRQF_RISING | IRQF_FALLING))) {
		error_printk("wrong flags for gpio_base: %x gpio_num: %d\n", gpio_base, gpio_num);
		ret = -EINVAL;
		goto fail;
	}

	if (flags & IRQF_RISING) {
		ret = stm32_exti_enable_rising(gpio_base, gpio_num);
		if (ret < 0) {
			error_printk("failed to configure irq flags\n");
			goto fail;
		}
	}

	if (flags & IRQF_FALLING) {
		ret = stm32_exti_enable_falling(gpio_base, gpio_num);
		if (ret < 0) {
			error_printk("failed to configure irq flags\n");
			goto fail;
		}
	}

	exti_callbacks[gpio_num].irq_action = handler;
	exti_callbacks[gpio_num].arg = arg;

fail:
	return ret;
}

int stm32_exti_init(void)
{
	int ret = 0;

	ret = irq_request(EXTI0_IRQn, &stm32_exti_isr, NULL);
	if (ret < 0)
		goto fail;

	ret = irq_request(EXTI1_IRQn, &stm32_exti_isr, NULL);
	if (ret < 0)
		goto fail;

	ret = irq_request(EXTI2_IRQn, &stm32_exti_isr, NULL);
	if (ret < 0)
		goto fail;

	ret = irq_request(EXTI3_IRQn, &stm32_exti_isr, NULL);
	if (ret < 0)
		goto fail;

	ret = irq_request(EXTI4_IRQn, &stm32_exti_isr, NULL);
	if (ret < 0)
		goto fail;

	ret = irq_request(EXTI9_5_IRQn, &stm32_exti_isr, NULL);
	if (ret < 0)
		goto fail;

	ret = irq_request(EXTI15_10_IRQn, &stm32_exti_isr, NULL);
	if (ret < 0)
		goto fail;

	memset(exti_callbacks, 0, sizeof(exti_callbacks));
fail:
	return ret;
}
postarch_initcall(stm32_exti_init);
