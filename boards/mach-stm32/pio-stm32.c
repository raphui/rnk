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
#include <mach/exti-stm32.h>
#include <errno.h>
#include <fdtparse.h>
#include <stdio.h>

#define GPIO_MODER(pin)			(3 << (pin * 2))
#define GPIO_MODER_OUTPUT(pin)		(1 << (pin * 2))
#define GPIO_MODER_ALTERNATE(pin)	(2 << (pin * 2))
#define GPIO_OSPEEDR(pin)		(3 << (pin * 2))
#define GPIO_PUPDR_PULLUP(pin)		(1 << (pin * 2))

struct gpio_options
{
	unsigned char val:1;			/* 0: low, 1: high */
	unsigned char mode:1;			/* 0: in, 1: out */
	unsigned char output:1;			/* 0: pp, 1: od */
	unsigned char speed:2;			/* 0 slow ... 3 fast */
	unsigned char pull:2;			/* 0: no, 1 pu, 2, pd */
	unsigned char analog:1;
	unsigned char edge:2;
};

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
	return stm32_exti_request_irq(port, mask, handler, flags, arg);
}

static void stm32_pio_enable_interrupt(unsigned int port, unsigned int mask)
{
	
}

static void stm32_pio_disable_interrupt(unsigned int port, unsigned int mask)
{
}

int stm32_pio_of_configure(int fdt_offset)
{
	const void *fdt_blob = fdtparse_get_blob();
	const struct fdt_property *prop;
	fdt32_t *cell;
	struct gpio_options *options;
	unsigned int base;
	int gpio;
	int gpio_num;
	int alt_func;
	unsigned short flags;
	int len, num, i;
	int parent_phandle, parent_offset;

	prop = fdt_get_property(fdt_blob, fdt_offset, "gpios", &len);
	if (len < 0) {
		return len;
	}

	num = len / (3 * sizeof(fdt32_t));

	cell = (fdt32_t *)prop->data;

	for(i = 0; i < num; i++, cell += 3) {
		options = (struct gpio_options *)&flags;

		parent_phandle = fdt32_to_cpu(cell[0]);
		parent_offset = fdt_node_offset_by_phandle(fdt_blob, parent_phandle);

		base = (unsigned int)fdtparse_get_addr32(parent_offset, "reg");
		gpio = fdt32_to_cpu(cell[1]);
		flags = fdt32_to_cpu(cell[2]);

		gpio_num = gpio & 0xFF;	

		if (options->mode)
			stm32_pio_set_output(base, gpio_num, options->pull);

		if(gpio & 0xF00) {
			alt_func = (gpio >> 8) & 0xF;
			stm32_pio_set_alternate(base, gpio_num, alt_func);
		} else {
			if(options->val)
				stm32_pio_set_value(base, gpio_num);
			else
				stm32_pio_clear_value(base, gpio_num);
		}
		
		if(options->edge)
		{
		}
	}

	return num;
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
	.of_configure = stm32_pio_of_configure,
};
