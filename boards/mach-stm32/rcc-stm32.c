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

struct clk
{
	unsigned int periph_base;
	unsigned int reg_base;
	unsigned int mask;
};

static struct clk clk_lut[] = {
	{TIM2_BASE, (unsigned int)&RCC->APB1ENR, RCC_APB1ENR_TIM2EN}, 
	{TIM3_BASE, (unsigned int)&RCC->APB1ENR, RCC_APB1ENR_TIM3EN}, 
	{TIM4_BASE, (unsigned int)&RCC->APB1ENR, RCC_APB1ENR_TIM4EN}, 
	{TIM5_BASE, (unsigned int)&RCC->APB1ENR, RCC_APB1ENR_TIM5EN}, 
	{TIM6_BASE, (unsigned int)&RCC->APB1ENR, RCC_APB1ENR_TIM6EN}, 
	{TIM7_BASE, (unsigned int)&RCC->APB1ENR, RCC_APB1ENR_TIM7EN}, 
	{TIM12_BASE, (unsigned int)&RCC->APB1ENR, RCC_APB1ENR_TIM12EN},
	{TIM13_BASE, (unsigned int)&RCC->APB1ENR, RCC_APB1ENR_TIM13EN},
	{TIM14_BASE, (unsigned int)&RCC->APB1ENR, RCC_APB1ENR_TIM14EN},
	{SPI2_BASE, (unsigned int)&RCC->APB1ENR, RCC_APB1ENR_SPI2EN},
	{SPI3_BASE, (unsigned int)&RCC->APB1ENR, RCC_APB1ENR_SPI3EN},
	{USART2_BASE, (unsigned int)&RCC->APB1ENR, RCC_APB1ENR_USART2EN},
	{USART3_BASE, (unsigned int)&RCC->APB1ENR, RCC_APB1ENR_USART3EN},
	{I2C1_BASE, (unsigned int)&RCC->APB1ENR, RCC_APB1ENR_I2C1EN},
	{I2C2_BASE, (unsigned int)&RCC->APB1ENR, RCC_APB1ENR_I2C2EN},
	{I2C3_BASE, (unsigned int)&RCC->APB1ENR, RCC_APB1ENR_I2C3EN},
	{TIM1_BASE, (unsigned int)&RCC->APB2ENR, RCC_APB2ENR_TIM1EN},
	{TIM8_BASE, (unsigned int)&RCC->APB2ENR, RCC_APB2ENR_TIM8EN},
	{USART1_BASE, (unsigned int)&RCC->APB2ENR, RCC_APB2ENR_USART1EN},
	{USART6_BASE, (unsigned int)&RCC->APB2ENR, RCC_APB2ENR_USART6EN},
	{SPI1_BASE, (unsigned int)&RCC->APB2ENR, RCC_APB2ENR_SPI1EN},
	{SYSCFG_BASE, (unsigned int)&RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN},
	{TIM9_BASE, (unsigned int)&RCC->APB2ENR, RCC_APB2ENR_TIM9EN},
	{TIM10_BASE, (unsigned int)&RCC->APB2ENR, RCC_APB2ENR_TIM10EN},
	{TIM11_BASE, (unsigned int)&RCC->APB2ENR, RCC_APB2ENR_TIM11EN},
	{GPIOA_BASE, (unsigned int)&RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN},
	{GPIOB_BASE, (unsigned int)&RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN},
	{GPIOC_BASE, (unsigned int)&RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN},
	{GPIOD_BASE, (unsigned int)&RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN},
	{GPIOE_BASE, (unsigned int)&RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN},
	{GPIOF_BASE, (unsigned int)&RCC->AHB1ENR, RCC_AHB1ENR_GPIOFEN},
	{GPIOG_BASE, (unsigned int)&RCC->AHB1ENR, RCC_AHB1ENR_GPIOGEN},
	{GPIOH_BASE, (unsigned int)&RCC->AHB1ENR, RCC_AHB1ENR_GPIOHEN},
	{GPIOI_BASE, (unsigned int)&RCC->AHB1ENR, RCC_AHB1ENR_GPIOIEN},
	{DMA1_BASE, (unsigned int)&RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN},
	{DMA2_BASE, (unsigned int)&RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN},
#ifdef CONFIG_STM32F429
	{SPI4_BASE, (unsigned int)&RCC->APB2ENR, RCC_APB2ENR_SPI4EN},
	{SPI5_BASE, (unsigned int)&RCC->APB2ENR, RCC_APB2ENR_SPI5EN},
	{SPI6_BASE, (unsigned int)&RCC->APB2ENR, RCC_APB2ENR_SPI6EN},
	{LTDC_BASE, (unsigned int)&RCC->APB2ENR, RCC_APB2ENR_LTDCEN},
	{GPIOJ_BASE, (unsigned int)&RCC->AHB1ENR, RCC_AHB1ENR_GPIOJEN},
	{GPIOK_BASE, (unsigned int)&RCC->AHB1ENR, RCC_AHB1ENR_GPIOKEN},
#endif /* CONFIG_STM32F429 */
	{ /* sentinel */ }
};

static int stm32_rcc_find_periph(int periph_base)
{
	int ret = -ENODEV;
	int i = 0;
	int size = sizeof(clk_lut) / sizeof(struct clk);

	for (i = 0; i < size; i++) {
		if (periph_base == clk_lut[i].periph_base) {
			ret = i;
			break;
		}
	}

	return ret;
}

int stm32_rcc_enable_clk(int periph_base)
{
	int ret = 0;
	int i = 0;
	unsigned int *reg_base;
	unsigned int mask;

	i = stm32_rcc_find_periph(periph_base);
	if (i < 0) {
		error_printk("cannot find periph base: 0x%x\r\n", periph_base);
		return i;
	}

	reg_base = (unsigned int *)clk_lut[i].reg_base;
	mask = clk_lut[i].mask;

	*reg_base |= mask;

	return ret;
}
