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
#include <mach/rcc-stm32.h>
#include <errno.h>
#include <fdtparse.h>

#ifdef CONFIG_INITCALL
#include <init.h>
#endif /* CONFIG_INITCALL */

/* system clock source */
#define RCC_CFGR_HSI      	0
#define RCC_CFGR_HSE      	1
#define RCC_CFGR_PLL      	2


struct clk
{
	unsigned int periph_base;
	unsigned int reg_base;
	unsigned int mask;
};

struct clk_div_table {
        unsigned int val;
        unsigned int div;
};

static const struct clk_div_table ahb_div_table[] = {
        { 0x0,   1 }, { 0x1,   1 }, { 0x2,   1 }, { 0x3,   1 },
        { 0x4,   1 }, { 0x5,   1 }, { 0x6,   1 }, { 0x7,   1 },
        { 0x8,   2 }, { 0x9,   4 }, { 0xa,   8 }, { 0xb,  16 },
        { 0xc,  64 }, { 0xd, 128 }, { 0xe, 256 }, { 0xf, 512 },
        { 0 },
};

static const struct clk_div_table apb_div_table[] = {
        { 0,  1 }, { 0,  1 }, { 0,  1 }, { 0,  1 },
        { 4,  2 }, { 5,  4 }, { 6,  8 }, { 7, 16 },
        { 0 },
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

static int sysclk_freq;
static int ahb_freq;
static int apb1_freq;
static int apb2_freq;

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

static int stm32_rcc_find_ahb_div(int pres)
{
	int i;
	int ret = -EINVAL;
	int size = sizeof(ahb_div_table) / sizeof(struct clk_div_table);

	for (i = 0; i < size; i++) {
		if (pres == ahb_div_table[i].div) {
			ret = ahb_div_table[i].val;
			break;
		}
	}

	return ret;
}

static int stm32_rcc_find_apb_div(int pres)
{
	int i;
	int ret = -EINVAL;
	int size = sizeof(apb_div_table) / sizeof(struct clk_div_table);

	for (i = 0; i < size; i++) {
		if (pres == apb_div_table[i].div) {
			ret = apb_div_table[i].val;
			break;
		}
	}

	return ret;
}

int stm32_rcc_get_freq_clk(unsigned int clk)
{
	int ret = 0;

	switch (clk) {
	case SYSCLK:
		ret = sysclk_freq;
		break;
	case AHB_CLK:
		ret = ahb_freq;
		break;
	case APB1_CLK:
		ret = apb1_freq;
		break;
	case APB2_CLK:
		ret = apb2_freq;
		break;
	default:
		ret = -EINVAL;
		break;
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

int stm32_rcc_disable_clk(int periph_base)
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

	*reg_base &= ~mask;

	return ret;
}

int stm32_rcc_enable_sys_clk(void)
{
	int ret = 0;
	int offset;
	int len;
	int pll_source, pll_m, pll_q, pll_n, pll_p;
	int pres_ahb, pres_apb1, pres_apb2;
	int div_ahb, div_apb1, div_apb2;
	int source, source_freq;
	const void *fdt_blob = fdtparse_get_blob();
	const struct fdt_property *prop;
	fdt32_t *cell;

	offset = fdt_path_offset(fdt_blob, "/clocks/fast");
	if (offset < 0) {
		error_printk("cannot find clock definition in fdt\n");
		ret = -ENOENT;
		goto out;
	}

	prop = fdt_get_property(fdt_blob, offset, "pll", &len);
	if (!prop) {
		error_printk("cannot find pll clocks in fdt\n");
		ret = -ENOENT;
		goto out;
	}

	if (len < 5) {
		error_printk("not enough parameters for pll (has to be 5)\n");
		ret = -EINVAL;
		goto out;
	}

	fdtparse_get_int(offset, "source", &source);

	cell = (fdt32_t *)prop->data;

	pll_source = fdt32_to_cpu(cell[0]);
	pll_m = fdt32_to_cpu(cell[1]);
	pll_n = fdt32_to_cpu(cell[2]);
	pll_p = fdt32_to_cpu(cell[3]);
	pll_q = fdt32_to_cpu(cell[4]);

	prop = fdt_get_property(fdt_blob, offset, "prescaler", &len);
	if (!prop) {
		error_printk("cannot find prescaler clocks in fdt\n");
		ret = -ENOENT;
		goto out;
	}

	if (len < 5) {
		error_printk("not enough parameters for prescaler (has to be 3)\n");
		ret = -EINVAL;
		goto out;
	}

	cell = (fdt32_t *)prop->data;

	pres_ahb = fdt32_to_cpu(cell[0]);
	div_ahb = stm32_rcc_find_ahb_div(pres_ahb);
	if (div_ahb < 0) {
		error_printk("cannot find ahb pres value\n");
		ret = div_ahb;
		goto out;
	}

	pres_apb1 = fdt32_to_cpu(cell[1]);
	div_apb1 = stm32_rcc_find_apb_div(pres_apb1);
	if (div_apb1 < 0) {
		error_printk("cannot find apb1 pres value\n");
		ret = div_apb1;
		goto out;
	}

	pres_apb2 = fdt32_to_cpu(cell[2]);
	div_apb2 = stm32_rcc_find_apb_div(pres_apb2);
	if (div_apb2 < 0) {
		error_printk("cannot find ap2 pres value\n");
		ret = div_apb2;
		goto out;
	}

	RCC->CFGR |= (div_ahb << 4);
	RCC->CFGR |= (div_apb1 << 10);
	RCC->CFGR |= (div_apb2 << 13);

	offset = fdt_path_offset(fdt_blob, "/clocks/sources");
	if (offset < 0) {
		error_printk("cannot find clock sources in fdt\n");
		ret = -ENOENT;
		goto out;
	}


	switch (source) {
	case RCC_CFGR_HSI:
		ret = fdtparse_get_int(offset, "hsi", &source_freq);
		if (ret < 0) {
			error_printk("failed to retrieve hsi freq\n");
			ret = -EIO;
			goto out;
		}
		break;
	case RCC_CFGR_HSE:
		ret = fdtparse_get_int(offset, "hse", &source_freq);
		if (ret < 0) {
			error_printk("failed to retrieve hsi freq\n");
			ret = -EIO;
			goto out;
		}

		sysclk_freq = source_freq;
		break;
	case RCC_CFGR_PLL:
		if (pll_source == 0) {
			ret = fdtparse_get_int(offset, "hse", &source_freq);
			if (ret < 0) {
				error_printk("failed to retrieve hsi freq\n");
				ret = -EIO;
				goto out;
			}
			RCC->CR |= RCC_CFGR_HSI;
			RCC->PLLCFGR = pll_m | (pll_n << 6) | (((pll_p >> 1) -1) << 16) | (pll_q << 24);
		} else {
			ret = fdtparse_get_int(offset, "hse", &source_freq);
			if (ret < 0) {
				error_printk("failed to retrieve hsi freq\n");
				ret = -EIO;
				goto out;
			}

			RCC->CR |= RCC_CR_HSEON;

			// Wait till HSE is ready
			while(!(RCC->CR & RCC_CR_HSERDY))
				;

			RCC->PLLCFGR = pll_m | (pll_n << 6) | (((pll_p >> 1) -1) << 16) | (RCC_PLLCFGR_PLLSRC_HSE) | (pll_q << 24);
		}

		RCC->CR |= RCC_CR_PLLON;

		while(!(RCC->CR & RCC_CR_PLLRDY))
			; // Wait till the main PLL is ready

		sysclk_freq = ((source_freq / pll_m) * pll_n) / pll_p;

		break;
	}

	ahb_freq = sysclk_freq / pres_ahb;
	apb1_freq = (sysclk_freq / pres_ahb) / pres_apb1;
	apb2_freq = (sysclk_freq / pres_ahb) / pres_apb2;

	/* Select regulator voltage output Scale 1 mode */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_VOS;

	/* Configure Flash prefetch, Instruction cache, Data cache and wait state */
	FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;


	/* Select the main PLL as system clock source */
	RCC->CFGR &= (unsigned int)((unsigned int)~(RCC_CFGR_SW));
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	/* Wait till the main PLL is used as system clock source */
	while ((RCC->CFGR & (unsigned int)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL);
		;
out:
	return ret;
}
#ifdef CONFIG_INITCALL
pure_initcall(stm32_rcc_enable_sys_clk);
#endif /* CONFIG_INITCALL */
