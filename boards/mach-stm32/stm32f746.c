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
#include <armv7m/system.h>
#include <utils.h>
#include <mach/fmc-stm32.h>
#include <mach/pio-stm32.h>
#include <mach/exti-stm32.h>

void set_sys_clock(void)
{
	/******************************************************************************/
	/*            PLL (clocked by HSE) used as System clock source                */
	/******************************************************************************/
	unsigned int StartUpCounter = 0, HSEStatus = 0;

	/* Enable HSE */
	RCC->CR |= ((unsigned int)RCC_CR_HSEON);

	/* Wait till HSE is ready and if Time out is reached exit */
	do
	{
		HSEStatus = RCC->CR & RCC_CR_HSERDY;
		StartUpCounter++;
	} while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

	if ((RCC->CR & RCC_CR_HSERDY) != RESET)
	{
		HSEStatus = (unsigned int)0x01;
	}
	else
	{
		HSEStatus = (unsigned int)0x00;
	}

	if (HSEStatus == (unsigned int)0x01)
	{
		/* Select regulator voltage output Scale 1 mode */
		RCC->APB1ENR |= RCC_APB1ENR_PWREN;
		PWR->CR |= PWR_CR_VOS;

		/* HCLK = SYSCLK / 1*/
		RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

		/* PCLK2 = HCLK / 2*/
		RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

		/* PCLK1 = HCLK / 4*/
		RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

		/* Configure the main PLL */
		RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) |
			(RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);

		/* Enable the main PLL */
		RCC->CR |= RCC_CR_PLLON;

		/* Wait till the main PLL is ready */
		while((RCC->CR & RCC_CR_PLLRDY) == 0)
			;

		/* Enable the Over-drive to extend the clock frequency to 180 Mhz */
		PWR->CR |= PWR_CR_ODEN;
		while((PWR->CSR & PWR_CSR_ODRDY) == 0)
			;
	
		PWR->CR |= PWR_CR_ODSWEN;
		while((PWR->CSR & PWR_CSR_ODSWRDY) == 0)
			;

		/* Configure Flash prefetch, Instruction cache, Data cache and wait state */
		FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;


		/* Select the main PLL as system clock source */
		RCC->CFGR &= (unsigned int)((unsigned int)~(RCC_CFGR_SW));
		RCC->CFGR |= RCC_CFGR_SW_PLL;

		/* Wait till the main PLL is used as system clock source */
		while ((RCC->CFGR & (unsigned int)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL);
			;
	}
	else
	{ /* If HSE fails to start-up, the application will have wrong clock
	     configuration. User can add here some code to deal with this error */
		while(1)
			;
	}
}

void low_level_init(void)
{
	/* Reset RCC clock */
	RCC->CR |= RCC_CR_HSION;
	RCC->CFGR = 0x00000000;
	/* Reset HSEON, CSSON and PLLON bits */
	RCC->CR &= (unsigned int)0xFEF6FFFF;
	/* Reset PLLCFGR register */
	RCC->PLLCFGR = 0x24003010;
	/* Reset HSEBYP bit */
	RCC->CR &= (unsigned int)0xFFFBFFFF;
	/* Disable all interrupts */
	RCC->CIR = 0x00000000;

	/* Configure the System clock source, PLL Multiplier and Divider factors, 
	   AHB/APBx prescalers and Flash settings ----------------------------------*/
	set_sys_clock();

	/* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM
	writel(SCB_VTOR, SRAM_BASE | VECT_TAB_OFFSET); /* Vector Table Relocation in Internal SRAM */
#else
	writel(SCB_VTOR, FLASH_BASE | VECT_TAB_OFFSET); /* Vector Table Relocation in Internal FLASH */
#endif
//	stm32_pio_set_alternate(GPIOB_BASE, 5, 0xC);
//	stm32_pio_set_alternate(GPIOB_BASE, 6, 0xC);
//
//	stm32_pio_set_alternate(GPIOC_BASE, 0, 0xC);
//
//	stm32_pio_set_alternate(GPIOD_BASE, 0, 0xC);
//	stm32_pio_set_alternate(GPIOD_BASE, 1, 0xC);
//	stm32_pio_set_alternate(GPIOD_BASE, 8, 0xC);
//	stm32_pio_set_alternate(GPIOD_BASE, 9, 0xC);
//	stm32_pio_set_alternate(GPIOD_BASE, 10, 0xC);
//	stm32_pio_set_alternate(GPIOD_BASE, 14, 0xC);
//	stm32_pio_set_alternate(GPIOD_BASE, 15, 0xC);
//
//	stm32_pio_set_alternate(GPIOE_BASE, 0, 0xC);
//	stm32_pio_set_alternate(GPIOE_BASE, 1, 0xC);
//	stm32_pio_set_alternate(GPIOE_BASE, 7, 0xC);
//	stm32_pio_set_alternate(GPIOE_BASE, 8, 0xC);
//	stm32_pio_set_alternate(GPIOE_BASE, 9, 0xC);
//	stm32_pio_set_alternate(GPIOE_BASE, 10, 0xC);
//	stm32_pio_set_alternate(GPIOE_BASE, 11, 0xC);
//	stm32_pio_set_alternate(GPIOE_BASE, 12, 0xC);
//	stm32_pio_set_alternate(GPIOE_BASE, 13, 0xC);
//	stm32_pio_set_alternate(GPIOE_BASE, 14, 0xC);
//	stm32_pio_set_alternate(GPIOE_BASE, 15, 0xC);
//
//	stm32_pio_set_alternate(GPIOF_BASE, 0, 0xC);
//	stm32_pio_set_alternate(GPIOF_BASE, 1, 0xC);
//	stm32_pio_set_alternate(GPIOF_BASE, 2, 0xC);
//	stm32_pio_set_alternate(GPIOF_BASE, 3, 0xC);
//	stm32_pio_set_alternate(GPIOF_BASE, 4, 0xC);
//	stm32_pio_set_alternate(GPIOF_BASE, 5, 0xC);
//	stm32_pio_set_alternate(GPIOF_BASE, 11, 0xC);
//	stm32_pio_set_alternate(GPIOF_BASE, 12, 0xC);
//	stm32_pio_set_alternate(GPIOF_BASE, 13, 0xC);
//	stm32_pio_set_alternate(GPIOF_BASE, 14, 0xC);
//	stm32_pio_set_alternate(GPIOF_BASE, 15, 0xC);
//
//	stm32_pio_set_alternate(GPIOG_BASE, 0, 0xC);
//	stm32_pio_set_alternate(GPIOG_BASE, 1, 0xC);
//	stm32_pio_set_alternate(GPIOG_BASE, 4, 0xC);
//	stm32_pio_set_alternate(GPIOG_BASE, 5, 0xC);
//	stm32_pio_set_alternate(GPIOG_BASE, 8, 0xC);
//	stm32_pio_set_alternate(GPIOG_BASE, 15, 0xC);
//
//	sdram_cmd_conf.cmd_mode = 0x1;		/* Clock enable */
//	sdram_cmd_conf.cmd_target = 0x8;	/* Bank 1 */
//	sdram_cmd_conf.auto_refresh_num = 0x1;
//	sdram_cmd_conf.mode = 0x0;
//
//	sdram_timing.load_to_active_delay = 2;
//	sdram_timing.exit_self_refresh_delay = 7;
//	sdram_timing.self_refresh_time = 4;
//	sdram_timing.row_cycle_delay = 7;
//	sdram_timing.write_recovery_time = 2;
//	sdram_timing.rp_delay = 2;
//	sdram_timing.rc_delay = 2;
//
//	sdram.num_bank = 0x2;
//	sdram.column = 0x0;			/* 8 bit */
//	sdram.row = 0x4;			/* 12 bit */
//	sdram.data_width = 0x10;		/* 16 bit */
//	sdram.internal_bank = 0x40;		/* Bank 4 */
//	sdram.cas = 0x180;			/* CAS Latency 3 */
//	sdram.write_protection = 0x0;		/* Disable */
//	sdram.clk_period = 0x800;		/* Clk Period 2 */
//	sdram.read_burst = 0x0;			/* Disable */
//	sdram.read_pipe_delay = 0x2000;		/* Delay 1 */
//	sdram.fmc_sdram_timing = &sdram_timing;
//	sdram.fmc_sdram_cmd_config = &sdram_cmd_conf;
//
//	stm32_fmc_init(&sdram);
//
//	/* Configure user button */
//	stm32_pio_set_input(GPIOA_BASE, 0, 0, 0);
//	stm32_exti_init(GPIOA_BASE, 0);
//	stm32_exti_enable_falling(GPIOA_BASE, 0);

	init_systick();
}
