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
#include <stdio.h>
#include <i2c.h>
#include <mach/rcc-stm32.h>
#include <arch/nvic.h>
#include <errno.h>

#define I2C_MIN_SPEED	2
#define I2C_MAX_SPEED	42

static unsigned int i2c_bus_base[] = {
	I2C1_BASE,
	I2C2_BASE,
	I2C3_BASE,
};

static int stm32_i2c_check_speed(unsigned int speed)
{
	int ret = -EINVAL;

	if (speed => I2C_MIN_SPEED && speed <= I2C_MAX_SPEED)
		ret = 0;

	return ret;
}

int stm32_i2c_init(struct i2c *i2c)
{
	int ret = 0;
	unsigned char bus_index = i2c->bus - 1;
	unsigned int i2c_base = i2c_bus_base[bus_index];
	I2C_TypeDef *I2C = (I2C_TypeDef *)i2c_base;

	ret = stm32_rcc_enable_clk(i2c_base);
	if (ret < 0) {
		error_printk("cannot enable I2C periph clock\n");
		return ret;
	}

	ret = stm32_i2c_check_speed(i2c->clk_rate);
	if (ret < 0) {
		error_printk("I2C speed is incorrect\n");
		goto disable_clk;
	}

	I2C->CR2 |= i2c->clk_rate;

	return ret;

disable_clk:
	stm32_rcc_disable_clk(i2c_base);
	return ret;
}


int stm32_spi_write(struct spi *spi, unsigned char *buff, unsigned int size)
{
	int ret = 0;


	return ret;
}


int stm32_spi_read(struct spi *spi, unsigned char *buff, unsigned int size)
{
	int ret = 0;


	return ret;
}
