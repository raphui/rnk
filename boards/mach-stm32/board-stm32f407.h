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

#ifndef BOARD_STM32F407_H
#define BOARD_STM32F407_H

#include <stm32f407.h>


/* SYSCLK = PLL_VCO / PLL_P =======> 168000000*/
/* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N  =======> 336000000 */

#define SYSCLK		168000000
#define HCLK		168000000
#define AHB_PRES	1
#define APB1_PRES	4
#define APB2_PRES	2
#define HSE		25000000

/*#define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET 0x00
#define PLL_M		25
#define PLL_Q		7
#define PLL_N		336
#define PLL_P		2
#define HSE_STARTUP_TIMEOUT    0x05000
#define RESET		0

#define APB1_CLK	(SYSCLK / AHB_PRES) / APB1_PRES
#define APB2_CLK	(SYSCLK / AHB_PRES) / APB2_PRES
 
struct uart_operations
{
	void (*init)(void);
	void (*print)(unsigned char byte);
	int (*printl)(const char *string);
};

struct uart_operations uart_ops;

struct pio_operations
{
	void (*set_output)(unsigned int port, unsigned int mask, int pull_up);
	void (*set_input)(unsigned int port, unsigned int mask, int pull_up, int filter);
	void (*set_alternate)(unsigned int port, unsigned int mask);
	void (*set_value)(unsigned int port, unsigned int mask);
	void (*clear_value)(unsigned int port, unsigned int mask);
	void (*enable_interrupt)(unsigned int port, unsigned int mask);
	void (*disable_interrupt)(unsigned int port, unsigned int mask);
};

struct pio_operations pio_ops;

#endif /* BOARD_STM32F407_H */
