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
#include <fsmc.h>


static unsigned int fsmc_rcc_bit[] = {
	RCC_APB1ENR_I2C1EN,
	RCC_APB1ENR_I2C2EN,
	RCC_APB1ENR_I2C3EN,
};

static unsigned int fsmc_bus_base[] = {
	FSMC_Bank1_R_BASE,
	FSMC_Bank1E_R_BASE,
	FSMC_Bank2_3_R_BASE,
	FSMC_Bank4_R_BASE,
};

void stm32_fsmc_init(struct fsmc *fsmc)
{

}
