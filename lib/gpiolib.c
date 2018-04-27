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

#include <gpiolib.h>
#include <unistd.h>
#include <syscall.h>
#include <errno.h>
#include <stdlib.h>
#include <export.h>

struct pio_desc *gpiolib_export(unsigned int gpio_num)
{
	int ret;
	struct pio_desc *desc = malloc(sizeof(struct pio_desc));

	if (!desc)
		return NULL;

	ret = syscall(SYSCALL_PIO_EXPORT, gpio_num, desc);
	if (ret < 0) {
		free(desc);
		desc = NULL;
	}

	return desc;
}
EXPORT_SYMBOL(gpiolib_export);

int gpiolib_set_output(struct pio_desc *desc, int gpio_value)
{
	int ret = 0;

	if (!desc)
		return -EINVAL;

	desc->mode = GPIO_MODE_OUTPUT;
	desc->state = gpio_value;

	ret = syscall(SYSCALL_PIO_SET_STATE, desc);

	return ret;
}
EXPORT_SYMBOL(gpiolib_set_output);

int gpiolib_set_input(struct pio_desc *desc)
{
	int ret = 0;

	if (!desc)
		return -EINVAL;

	desc->mode = GPIO_MODE_INPUT;

	ret = syscall(SYSCALL_PIO_SET_STATE, desc);

	return ret;
}
EXPORT_SYMBOL(gpiolib_set_input);

int gpiolib_output_set_value(struct pio_desc *desc, int gpio_value)
{
	int ret = 0;

	if (!desc)
		return -EINVAL;

	if (desc->mode != GPIO_MODE_OUTPUT)
		return -EINVAL;

	desc->state = gpio_value;

	ret = syscall(SYSCALL_PIO_SET_STATE, desc);

	return ret;
}
EXPORT_SYMBOL(gpiolib_output_set_value);
