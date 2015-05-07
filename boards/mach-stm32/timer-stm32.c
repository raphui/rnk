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
#include <errno.h>

static int stm32_timer_init(unsigned int num)
{
	TIM_TypeDef *tim = NULL;
	unsigned int rcc_en = 0;

	if (num < 2 || num > 5)
		return -EINVAL;

	switch (num) {
		case 2:
			tim = TIM2;
			rcc_en = RCC_APB1ENR_TIM2EN;
			break;
		case 3:
			tim = TIM3;
			rcc_en = RCC_APB1ENR_TIM3EN;
			break;
		case 4:
			tim = TIM4;
			rcc_en = RCC_APB1ENR_TIM4EN;
			break;
		case 5:
			tim = TIM5;
			rcc_en = RCC_APB1ENR_TIM5EN;
			break;
	}

	RCC->APB1ENR |= rcc_en;
}
