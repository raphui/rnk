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
#include <utils.h>
#include <mach/rcc-stm32.h>

#define GPIO_MODER(pin)			(3 << (pin * 2))
#define GPIO_MODER_OUTPUT(pin)		(1 << (pin * 2))
#define GPIO_MODER_ALTERNATE(pin)	(2 << (pin * 2))
#define GPIO_OSPEEDR(pin)		(3 << (pin * 2))
#define GPIO_PUPDR_PULLUP(pin)		(1 << (pin * 2))

static void stm32_pio_set_clock(unsigned int port)
{
	int ret = 0;

	ret = stm32_rcc_enable_clk(port);
	if (ret < 0)
		error_printk("cannot enable GPIO periph clock\r\n");

}

void stm32_pio_set_output(unsigned int port, unsigned int mask, int pull_up)
{
	GPIO_TypeDef *base = (GPIO_TypeDef *)port;

	stm32_pio_set_clock(port);

	base->MODER |= GPIO_MODER_OUTPUT(mask);
	base->OSPEEDR |= GPIO_OSPEEDR(mask);

	if (pull_up)
		base->PUPDR |= GPIO_PUPDR_PULLUP(mask);
	else
		base->PUPDR &= ~(GPIO_PUPDR_PULLUP(mask));
}

void stm32_pio_set_input(unsigned int port, unsigned int mask, int pull_up, int filter)
{
	GPIO_TypeDef *base = (GPIO_TypeDef *)port;
	stm32_pio_set_clock(port);
	base->MODER &= ~(GPIO_MODER(mask));
}

void stm32_pio_set_alternate(unsigned int port, unsigned int mask, unsigned int num)
{
	GPIO_TypeDef *base = (GPIO_TypeDef *)port;
	unsigned int afr_high_base = 8;

	stm32_pio_set_clock(port);

	base->MODER |= GPIO_MODER_ALTERNATE(mask);
	base->OSPEEDR |= GPIO_OSPEEDR(mask);

	if (mask > 7)
		base->AFR[1] |= (num << ((mask - afr_high_base) * 4));
	else
		base->AFR[0] |= (num << (mask * 4));
}

void stm32_pio_set_value(unsigned int port, unsigned int mask)
{
	GPIO_TypeDef *base = (GPIO_TypeDef *)port;
	base->ODR |= (1 << mask);
}

void stm32_pio_clear_value(unsigned int port, unsigned int mask)
{
	GPIO_TypeDef *base = (GPIO_TypeDef *)port;
	base->ODR &= ~(1 << mask);
}

void stm32_pio_toggle_value(unsigned int port, unsigned int mask)
{
	GPIO_TypeDef *base = (GPIO_TypeDef *)port;
	base->ODR ^= (1 << mask);
}

static int stm32_pio_request_interrupt(unsigned int port, unsigned int mask, void (*handler)(void), int flags, void *arg)
{

}

static void stm32_pio_enable_interrupt(unsigned int port, unsigned int mask)
{
	
}

static void stm32_pio_disable_interrupt(unsigned int port, unsigned int mask)
{
}

struct pio_operations pio_ops = {
	.set_output = stm32_pio_set_output,
	.set_input = stm32_pio_set_input,
	.set_alternate = stm32_pio_set_alternate,
	.set_value = stm32_pio_set_value,
	.clear_value = stm32_pio_clear_value,
	.toggle_value = stm32_pio_toggle_value,
	.request_interrupt = stm32_pio_request_interrupt,
	.enable_interrupt = stm32_pio_enable_interrupt,
	.disable_interrupt = stm32_pio_disable_interrupt,
};
