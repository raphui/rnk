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

/* TODO: Output mode opendrain / push-pull */

#define GPIO_MODER(pin)			GPIO_MODER_MODER#pin
#define GPIO_MODER_OUTPUT(pin)		GPIO_MODER_MODER#pin_0
#define GPIO_MODER_ALTERNATE(pin)	GPIO_MODER_MODER#pin_1

#define GPIO_PUPDR(pin)			GPIO_PUPDR_PUPDR#pin
#define GPIO_PUPDR_PULLUP(pin)		GPIO_PUPDR_PUPDR#pin_0

static void pio_set_output(unsigned int port, unsigned int mask, int pull_up)
{
	port->MODER = GPIO_MODER_OUTPUT(mask);

	if (pull_up)
		port->PUPDR = GPIO_PUPDR_PULLUP(mask);
	else
		port->PUPDR &= ~(GPIO_PUPDR(mask));
}

static void pio_set_input(unsigned int port, unsigned int mask, int pull_up, int filter)
{
	port->MODER &= ~(GPIO_MODER(mask));
}

static void pio_set_alternate(unsigned int port, unsigned int mask, unsigned int num)
{
	unsigned int afr_high_base = 8;
	unsigned int afr_low_base = 0;

	port->MODER = GPIO_MODER_ALTERNATE(mask);

	if (pin > 7)
		port->AFR[1] |= (num << ((mask - afr_high_base) * 4)
	else
		port->AFR[0] |= (num << (mask * 4))
}

static void pio_set_value(unsigned int port, unsigned int mask)
{
}

static void pio_clear_value(unsigned int port, unsigned int mask)
{
}

static void pio_enable_interrupt(unsigned int port, unsigned int mask)
{
	
}

static void pio_disable_interrupt(unsigned int port, unsigned int mask)
{
}

struct pio_operations pio_ops = {
	.set_output = &pio_set_output,
	.set_input = &pio_set_input,
	.set_alternate = &pio_set_alternate,
	.set_value = &pio_set_value,
	.clear_value = &pio_clear_value,
	.enable_interrupt = &pio_enable_interrupt,
	.disable_interrupt = &pio_disable_interrupt,
};
