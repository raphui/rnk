/*
 * Copyright (C) 2014  Raphaël Poggi <poggi.raph@gmail.com>
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

static void pio_set_output(unsigned int port, unsigned int mask, int pull_up)
{
	writel(port + PIO_IDR, mask);

	if (pull_up)
		writel(port + PIO_PPUER, mask);
	else
		writel(port + PIO_PPUDR, mask);

	writel(port + PIO_MDDR, mask);
	writel(port + PIO_OER, mask);
	writel(port + PIO_PER, mask);
}

static void pio_set_input(unsigned int port, unsigned int mask, int pull_up, int filter)
{
	if (pull_up)
		writel(port + PIO_PPUER, mask);
	else
		writel(port + PIO_PPUDR, mask);

	if (filter)
		writel(port + PIO_IFER, mask);
	else
		writel(port + PIO_IFDR, mask);

	writel(port + PIO_ODR, mask);
	writel(port + PIO_PER, mask);
}

static void pio_set_value(unsigned int port, unsigned int mask)
{
	writel(port + PIO_SODR, mask);
}

static void pio_clear_value(unsigned int port, unsigned int mask)
{
	writel(port + PIO_CODR, mask);
}

static void pio_enable_interrupt(unsigned int port, unsigned int mask)
{
	writel(port + PIO_IER, mask);
}

static void pio_disable_interrupt(unsigned int port, unsigned int mask)
{
	writel(port + PIO_IDR, mask);
}

struct pio_operations pio_ops = {
	.set_output = &pio_set_output,
	.set_input = &pio_set_input,
	.set_value = &pio_set_value,
	.clear_value = &pio_clear_value,
	.enable_interrupt = &pio_enable_interrupt,
	.disable_interrupt = &pio_disable_interrupt,
};
