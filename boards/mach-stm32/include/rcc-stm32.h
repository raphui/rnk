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

#ifndef RCC_STM32_H
#define RCC_STM32_H

#define SYSCLK		0
#define AHB_CLK		1
#define APB1_CLK	2
#define APB2_CLK	3

extern int stm32_rcc_get_freq_clk(unsigned int clk);
extern int stm32_rcc_get_pres_clk(unsigned int clk);
extern int stm32_rcc_enable_clk(int periph_base);
extern int stm32_rcc_disable_clk(int periph_base);
extern int stm32_rcc_enable_sys_clk(void);

#endif  /* RCC_STM32_H */
