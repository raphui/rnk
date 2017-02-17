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
#include <init.h>
#include <armv7m/system.h>
#include <mach/rcc-stm32.h>

#ifdef CONFIG_STM32F429
#include <mach/fmc-stm32.h>
#endif /* CONFIG_STM32F429 */

#ifdef CONFIG_SWO_DEBUG
#include <mach/pio-stm32.h>
#include <armv7m/swo.h>
#endif /* CONFIG_SWO_DEBUG */

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

	/* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM
	writel(SCB_VTOR, SRAM_BASE | VECT_TAB_OFFSET); /* Vector Table Relocation in Internal SRAM */
#else
	writel(SCB_VTOR, FLASH_BASE | VECT_TAB_OFFSET); /* Vector Table Relocation in Internal FLASH */
#endif
}

int device_init(void)
{
	int ret = 0;

//	struct mtd mtd;

//	mtd.base_addr = 0x08000000;
//	mtd.sector_size[0] = SZ_16K;
//	mtd.sector_size[1] = SZ_16K;
//	mtd.sector_size[2] = SZ_16K;
//	mtd.sector_size[3] = SZ_16K;
//	mtd.sector_size[4] = SZ_64K;
//	mtd.sector_size[5] = SZ_128K;
//	mtd.sector_size[6] = SZ_128K;
//	mtd.sector_size[7] = SZ_128K;
//	mtd.sector_size[8] = SZ_128K;
//	mtd.sector_size[9] = SZ_128K;
//	mtd.sector_size[10] = SZ_128K;
//	mtd.sector_size[11] = SZ_128K;
//	mtd.num_sectors = 12;
//
//	mtd_init(&mtd);

//	usart_init(3, USART3_BASE, 115200);
//	pio_set_alternate(GPIOC_BASE, 10, 0x7);
//	pio_set_alternate(GPIOC_BASE, 11, 0x7);

//	stm32_exti_init();
//	/* Configure wakeup button interrupt */
//	stm32_exti_init(GPIOA_BASE, 0);
//	stm32_exti_enable_falling(GPIOA_BASE, 0);
//
//	/* Configure anti-tamper button interrupt */
//	stm32_exti_init(GPIOC_BASE, 13);
//	stm32_exti_enable_falling(GPIOC_BASE, 13);

#ifdef CONFIG_SWO_DEBUG

	stm32_pio_set_alternate(GPIOB_BASE, 3, 0x0);
	swo_init(stm32_rcc_get_freq_clk(SYSCLK));
	DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN; // Enable IO trace pins
#endif /* CONFIG_SWO_DEBUG */

	return ret;
}
coredevice_initcall(device_init);
