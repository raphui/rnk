/*
 * Copyright (C) 2018  Raphaël Poggi <poggi.raph@gmail.com>
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

#ifndef GPIOLIB_H
#define GPIOLIB_H

#include <pio.h>

struct pio_desc *gpiolib_export(unsigned int gpio_num);
int gpiolib_set_output(struct pio_desc *desc, int gpio_value);
int gpiolib_set_input(struct pio_desc *desc);
int gpiolib_output_set_value(struct pio_desc *desc, int gpio_value);

#endif /* GPIOLIB_H */
