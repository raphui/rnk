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
#include <pio.h>

void pio_set_output(unsigned int port, unsigned int mask, int pull_up)
{
	pio_ops.set_output(port, mask, pull_up);
}

void pio_set_input(unsigned int port, unsigned int mask, int pull_up, int filter)
{
	pio_ops.set_input(port, mask, pull_up, filter);
}

void pio_set_alternate(unsigned int port, unsigned int mask, unsigned int num)
{
	pio_ops.set_alternate(port, mask, num);
}

void pio_set_value(unsigned int port, unsigned int mask)
{
	pio_ops.set_value(port, mask);
}

void pio_clear_value(unsigned int port, unsigned int mask)
{
	pio_ops.clear_value(port, mask);
}

void pio_toggle_value(unsigned int port, unsigned int mask)
{
	pio_ops.toggle_value(port, mask);
}

void pio_enable_interrupt(unsigned int port, unsigned int mask)
{
	pio_ops.enable_interrupt(port, mask);
}

void pio_disable_interrupt(unsigned int port, unsigned int mask)
{
	pio_ops.disable_interrupt(port, mask);
}
