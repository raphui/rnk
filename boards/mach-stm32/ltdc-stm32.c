/*
 * Copyright (C) 2014  Raphaël Poggi <poggi.raph@gmail.com>
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

#include <board.h>
#include <ltdc.h>
#include <utils.h>
#include <stdio.h>

#define GCR_MASK		((unsigned int)0x0FFE888F)
#define RCC_PLLSAIDivR_Div4	((unsigned int)0x00010000)

static void stm32_ltdc_pll_sai_config(unsigned int n, unsigned int q, unsigned int r)
{
	RCC->PLLSAICFGR = (n << 6) | (q << 24) | (r << 28);
}

static void stm32_ltdc_clk_divconfig(unsigned int div_r)
{
	RCC->DCKCFGR &= ~RCC_DCKCFGR_PLLSAIDIVR;
	RCC->DCKCFGR |= div_r;
}

static void stm32_ltdc_enable_fb(struct ltdc *ltdc)
{
	LTDC_Layer1->WHPCR = ((ltdc->hsync + ltdc->hbp) << 0) | ((ltdc->hsync + ltdc->hbp + ltdc->width - 1) << 16);
	LTDC_Layer1->WHPCR = ((ltdc->vsync + ltdc->vbp) << 0) | ((ltdc->vsync + ltdc->vbp + ltdc->height - 1) << 16);

	switch (ltdc->bpp) {
		case 2:
			LTDC_Layer1->PFCR = 2;
			break;
		default:
			debug_printk("Unknow bits per pixel\r\n");
			return;
	}

	LTDC_Layer1->CFBAR = ltdc->fb_addr;
	LTDC_Layer1->CFBLR = ((ltdc->width * ltdc->bpp) << 16) | ((ltdc->width * ltdc->bpp + 3) << 0);
	LTDC_Layer1->CFBLNR = (ltdc->height << 0);

//	LTDC_Layer1->CACR &= ~LTDC_LxCACR_CONSTA;

	LTDC_Layer1->CR |= LTDC_LxCR_LEN;

}

void stm32_ltdc_init(struct ltdc *ltdc)
{
	unsigned int h_cycles;
	unsigned int v_cycles;

	stm32_ltdc_pll_sai_config(192, 7, 4);
	stm32_ltdc_clk_divconfig(RCC_PLLSAIDivR_Div4);

	RCC->CR |= RCC_CR_PLLSAION;

	while (!(RCC->CR & RCC_CR_PLLSAIRDY))
		;

	RCC->APB2ENR |= RCC_APB2ENR_LTDCEN;

	LTDC->GCR &= ~LTDC_GCR_LTDCEN;

	h_cycles = ltdc->hsync - 1;
	v_cycles = ltdc->vsync - 1;

	LTDC->SSCR = (h_cycles << 16) | (v_cycles << 0);

	h_cycles += ltdc->hbp;
	v_cycles += ltdc->vbp;

	LTDC->BPCR = (h_cycles << 16) | (v_cycles << 0);

	h_cycles += ltdc->width;
	v_cycles += ltdc->height;

	LTDC->AWCR = (h_cycles << 16) | (v_cycles << 0);

	h_cycles += ltdc->hfp;
	v_cycles += ltdc->vfp;

	LTDC->TWCR = (h_cycles << 16) | (v_cycles << 0);

	/* Clean general config */
	LTDC->GCR &= ~(GCR_MASK);

	/* Background color to blue */
	LTDC->BCCR = (0xFF << 0);

//	stm32_ltdc_enable_fb(ltdc);

	LTDC->SRCR = LTDC_SRCR_IMR;

	/* Enable LCD Controller */
	LTDC->GCR |= LTDC_GCR_LTDCEN;

}

struct lcd_operations lcd_ops = {
	.init = stm32_ltdc_init,	
};
