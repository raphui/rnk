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

#ifndef PIO_STM32_H
#define PIO_STM32_H

extern void stm32_pio_set_output(unsigned int port, unsigned int mask, int pull_up);
extern void stm32_pio_set_input(unsigned int port, unsigned int mask, int pull_up, int filter);
extern void stm32_pio_set_alternate(unsigned int port, unsigned int mask, unsigned int num);
extern void stm32_pio_set_value(unsigned int port, unsigned int mask);
extern void stm32_pio_clear_value(unsigned int port, unsigned int mask);
extern void stm32_pio_toggle_value(unsigned int port, unsigned int mask);

extern int stm32_pio_of_configure(int fdt_offset);

#endif /* PIO_STM32_H */
