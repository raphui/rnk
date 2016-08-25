/*
 * Copyright (C) 2015  Raphaël Poggi <poggi.raph@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <board.h>
#include <utils.h>
#include <stdio.h>
#include <arch/nvic.h>
#include <armv7m/vector.h>
#include <errno.h>
#include <list.h>
#include <irq.h>
#include <mm.h>

struct action {
	void (*irq_action)(void);
	int irq;
	TAILQ_ENTRY(action) next;
};

static TAILQ_HEAD(, action) action_list;

static int nvic_array[15] = {
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
		case GPIOD_BASE:
			mask = 0x3;
			break;
		case GPIOE_BASE:
			mask = 0x4;
			break;
		case GPIOF_BASE:
			mask = 0x5;
			break;
		case GPIOG_BASE:
			mask = 0x6;
			break;
		case GPIOH_BASE:
			mask = 0x7;
			break;
		case GPIOI_BASE:
			mask = 0x8;
			break;
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
	unsigned int irq = vector_current_irq();
	int line = EXTI->PR & 0x7FFFFF;

	line >>= 1;

	EXTI->PR |= (0x7FFFFF);
	nvic_clear_interrupt(irq);

	stm32_exti_action(line);
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

	TAILQ_INIT(&action_list);

fail:
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

	EXTI->RTSR |= (1 << gpio_num);
	EXTI->IMR |= (1 << gpio_num);

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

	EXTI->FTSR |= (1 << gpio_num);
	EXTI->IMR |= (1 << gpio_num);

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

	EXTI->RTSR &= ~(1 << gpio_num);
	EXTI->IMR &= ~(1 << gpio_num);

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

	EXTI->FTSR &= ~(1 << gpio_num);
	EXTI->IMR &= ~(1 << gpio_num);

	nvic_clear_interrupt(nvic);
	nvic_disable_interrupt(nvic);

	return ret;
}

int stm32_exti_action(unsigned int line)
{
	int ret = 0;
	struct action *action = NULL;
	void (*hook)(void) = NULL;

	TAILQ_FOREACH(action, &action_list, next)
		if (action->irq == line)
			break;

	if (!action) {
		error_printk("no action has been found for exti: %d\n", line);
		return -ENOSYS;
	}

	hook = action->irq_action;

	hook();

	return ret;
}

int stm32_exti_request_irq(unsigned int gpio_base, unsigned int gpio_num, void (*handler)(void), int flags, void *arg)
{
	int ret = 0;
	struct action *action = NULL;

	ret = stm32_exti_configure_line(gpio_base, gpio_num);
	if (ret < 0) {
		error_printk("failed to init exti for gpio_base: %x gpio_num: %d\n", gpio_base, gpio_num);
		goto fail;
	}

	if (flags & IRQF_RISING)
		stm32_exti_enable_rising(gpio_base, gpio_num);
	else if (flags & IRQF_FALLING)
		stm32_exti_enable_falling(gpio_base, gpio_num);
	else {
		error_printk("wrong flags for gpio_base: %x gpio_num: %d\n", gpio_base, gpio_num);
		ret = -EINVAL;
		goto fail;
	}

	action = (struct action *)kmalloc(sizeof(struct action));
	if (!action) {
		error_printk("cannot allocate exti irq action\n");
		ret = -ENOMEM;
		goto fail;
	}

	action->irq_action = handler;
	action->irq = gpio_num;

	if (TAILQ_EMPTY(&action_list))
		TAILQ_INSERT_HEAD(&action_list, action, next);
	else
		TAILQ_INSERT_TAIL(&action_list, action, next);

fail:
	return ret;
}
