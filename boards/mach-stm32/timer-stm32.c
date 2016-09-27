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
 * along with this program; if not, write to the Frrestore * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <board.h>
#include <errno.h>
#include <timer.h>
#include <stddef.h>
#include <arch/nvic.h>
#include <armv7m/vector.h>
#include <irq.h>
#include <mm.h>

#include <mach/rcc-stm32.h>

struct action {
	void (*irq_action)(void *arg);
	void *arg;
	int irq;
	struct list_node node;
};

static struct list_node action_list;

static int stm32_timer_get_nvic_number(struct timer *timer)
{
	int nvic = 0;

	switch (timer->num + 2) {
		case 2:
			nvic = TIM2_IRQn;
			break;
		case 3:
			nvic = TIM3_IRQn;
			break;
		case 4:
			nvic = TIM4_IRQn;
			break;
		case 5:
			nvic = TIM5_IRQn;
			break;

	};

	return nvic;
}

static void stm32_timer_clear_it_flags(struct timer *timer, unsigned int flags)
{
	TIM_TypeDef *tim = NULL;

	tim = (TIM_TypeDef *)timer->base_reg;

	tim->SR &= ~flags;
}

static int stm32_timer_action(struct timer *timer)
{
	int ret = 0;
	void *arg = NULL;
	struct action *action = NULL;
	void (*hook)(void *) = NULL;

	list_for_every_entry(&action_list, action, struct action, node)
		if (action->irq == (timer->num + 2))
			break;

	if (!action) {
		error_printk("no action has been found for exti: %d\n", timer->num + 2);
		return -ENOSYS;
	}

	hook = action->irq_action;
	arg = action->arg;

	hook(arg);

	return ret;
}

static void stm32_timer_isr(void *arg)
{
	struct timer *timer = (struct timer *)arg;
	unsigned int irq = vector_current_irq();

	stm32_timer_clear_it_flags(timer, TIM_SR_UIF);

	nvic_clear_interrupt(irq);

	stm32_timer_action(timer);
}

static int stm32_timer_init(struct timer *timer)
{
	int ret = 0;
	int irq_line = 0;
	unsigned int base_reg = 0;

	/* XXX: timer generic driver start from 0 to CONFIG_TIMER_NB
	 *	but stm32 driver start from 2 to 5, so we made the conversion in this way
	 */
	if ((timer->num + 2) < 2 || (timer->num + 2) > 5)
		return -EINVAL;

	switch (timer->num + 2) {
		case 2:
			base_reg = TIM2_BASE;
			break;
		case 3:
			base_reg = TIM3_BASE;
			break;
		case 4:
			base_reg = TIM4_BASE;
			break;
		case 5:
			base_reg = TIM5_BASE;
			break;
	}

	timer->base_reg = base_reg;

	ret = stm32_rcc_enable_clk(timer->base_reg);
	if (ret < 0) {
		error_printk("cannot enable TIM%d clock\r\n", timer->num + 2);
		return ret;
	}

	timer->rate = (APB1_PRES > 1) ? APB1_CLK * 2 : APB1_CLK;
	timer->prescaler = 0;

	irq_line = stm32_timer_get_nvic_number(timer);
	if (irq_line < 0) {
		error_printk("invalid irq line\n");
		goto clk_disable;
	}

	ret = irq_request(irq_line, &stm32_timer_isr, timer);
	if (ret < 0) {
		error_printk("cannot request isr for irq line: %d\n", irq_line);
		ret = stm32_rcc_disable_clk(timer->base_reg);
	}

	list_initialize(&action_list);

	return ret;

clk_disable:
	stm32_rcc_disable_clk(timer->base_reg);
	return ret;
}

static short stm32_timer_find_best_pres(unsigned long parent_rate, unsigned long rate)
{
	unsigned short pres;
	unsigned short best_pres;
	unsigned short max_pres = 0xffff;
	unsigned int diff;
	unsigned int best_diff;
	unsigned long curr_rate;


	best_diff = parent_rate - rate;

	for (pres = 1; pres < max_pres; pres++) {
		curr_rate = parent_rate / pres;

		if (curr_rate < rate)
			diff = rate - curr_rate;
		else
			diff = curr_rate - rate;

		if (diff < best_diff) {
			best_pres = pres;
			best_diff = diff;
		}

		if (!best_diff || curr_rate < rate)
			break;
	}

	return best_pres;
}

static void stm32_timer_set_rate(struct timer *timer, unsigned long rate)
{
	TIM_TypeDef *tim = NULL;
	unsigned short pres;

	pres = stm32_timer_find_best_pres(timer->rate, rate);

	/* The counter clock frequency CK_CNT is equal to fCK_PSC / (PSC[15:0] + 1) */
	pres -= 1;

	tim = (TIM_TypeDef *)timer->base_reg;

	tim->PSC = pres;
}

static void stm32_timer_set_counter(struct timer *timer, unsigned short counter)
{
	TIM_TypeDef *tim = NULL;

	tim = (TIM_TypeDef *)timer->base_reg;

	tim->ARR = counter;
}

static void stm32_timer_enable(struct timer *timer)
{
	TIM_TypeDef *tim = NULL;
	int nvic = stm32_timer_get_nvic_number(timer);

	if (nvic < 0) {
		error_printk("invalid nvic line\n");
		return;
	}

	tim = (TIM_TypeDef *)timer->base_reg;

	if (timer->one_pulse)
		tim->CR1 |= TIM_CR1_OPM;

	if (!timer->count_up)
		tim->CR1 |= TIM_CR1_DIR;
	else
		tim->CR1 &= ~TIM_CR1_DIR;

	/* and interrupt */
	tim->DIER |= TIM_DIER_UIE;

	/* finally, enable counter */
	tim->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE;

	nvic_enable_interrupt(nvic);
}

static void stm32_timer_disable(struct timer *timer)
{
	TIM_TypeDef *tim = NULL;
	int nvic = stm32_timer_get_nvic_number(timer);

	if (nvic < 0) {
		error_printk("invalid nvic line\n");
		return;
	}

	tim = (TIM_TypeDef *)timer->base_reg;

	tim->CR1 &= ~TIM_CR1_CEN;

	nvic_clear_interrupt(nvic);
	nvic_disable_interrupt(nvic);
}

static int stm32_timer_request_irq(struct timer *timer, void (*handler)(void *), void *arg)
{
	int ret = 0;
	struct action *action = NULL;

	action = (struct action *)kmalloc(sizeof(struct action));
	if (!action) {
		error_printk("cannot allocate exti irq action\n");
		ret = -ENOMEM;
		goto fail;
	}

	action->irq_action = handler;
	action->arg = arg;
	action->irq = timer->num + 2;
	
	list_add_tail(&action_list, &action->node);

fail:
	return ret;	
}

static int stm32_timer_release_irq(struct timer *timer)
{
	int ret = 0;
	struct action *action = NULL;

	list_for_every_entry(&action_list, action, struct action, node)
		if (action->irq == timer->num + 2)
			break;

	if (action)
		list_delete(&action->node);
	else
		ret = -ENOENT;

	kfree(action);

	return ret;
}

struct timer_operations tim_ops = {
	.init = stm32_timer_init,
	.set_rate = stm32_timer_set_rate,
	.set_counter = stm32_timer_set_counter,
	.enable = stm32_timer_enable,
	.disable = stm32_timer_disable,
	.clear_it_flags = stm32_timer_clear_it_flags,
	.request_irq = stm32_timer_request_irq,
	.release_irq = stm32_timer_release_irq,
};
