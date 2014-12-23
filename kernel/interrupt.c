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
#include <stdio.h>
#include <pio.h>
#include <utils.h>
#include <interrupt.h>

void pio_isr(void)
{
	unsigned int mask = readl(AT91C_BASE_PIOA + PIO_ISR);

	printk("pio_isr\r\n");

	/* Clear all led value */
	pio_set_value(AT91C_BASE_PIOA, 0xf);
	
	/* Set led depending on button pressed */
	if (mask & (1 << 19))
		pio_clear_value(AT91C_BASE_PIOA, (1 << 0));
	else if (mask & (1 << 20))
		pio_clear_value(AT91C_BASE_PIOA, (1 << 1));
	else if (mask & (1 << 15))
		pio_clear_value(AT91C_BASE_PIOA, (1 << 2));
	else if (mask & (1 << 14))
		pio_clear_value(AT91C_BASE_PIOA, (1 << 3));

	printk("good bye\n\r");
}
