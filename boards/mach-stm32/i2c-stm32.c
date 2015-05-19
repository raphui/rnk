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
#include <i2c.h>

static unsigned int i2c_rcc_bit[] = {
	RCC_APB1ENR_I2C1EN,
	RCC_APB1ENR_I2C2EN,
	RCC_APB1ENR_I2C3EN,
};

static unsigned int i2c_bus_base[] = {
	I2C1_BASE,
	I2C2_BASE,
	I2C3_BASE,
};

void stm32_i2c_init(struct i2c *i2c)
{
	unsigned char bus_index = i2c->bus - 1;
	unsigned int rcc_off = bus_index;
	unsigned int i2c_base = i2c_bus_base[bus_index];
	I2C_TypeDef *i2c_reg = (I2C_TypeDef *)i2c_base;

	RCC->APB1ENR |= i2c_rcc_bit[rcc_off];

	i2c_reg->CCR |= I2C_CCR_FS; 
}
