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
#include <sizes.h>
#include <armv7m/system.h>
#include <armv7m/mpu.h>
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

	mpu_map_from_low((void *)SRAM_BASE, SZ_128K, MPU_RASR_SHARE_CACHE | MPU_RASR_AP_PRIV_RW_UN_RO);
	mpu_map_from_low((void *)FLASH_BASE, SZ_1M, MPU_RASR_NORMAL_CACHE | MPU_RASR_AP_PRIV_RW_UN_RW);
	mpu_map_from_low((void *)PERIPH_BASE, SZ_512M, MPU_RASR_DEVICE_SHARE | MPU_RASR_AP_PRIV_RW_UN_RW);
	//mpu_map_from_low((void *)CCMDATARAM_BASE, SZ_64K, MPU_RASR_SHARE_CACHE | MPU_RASR_AP_PRIV_RW_UN_RW);

#ifdef CONFIG_SWO_DEBUG

	stm32_pio_set_alternate(GPIOB_BASE, 3, 0x0);
	swo_init(stm32_rcc_get_freq_clk(SYSCLK));
	DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN; // Enable IO trace pins
#endif /* CONFIG_SWO_DEBUG */

	return ret;
}
coredevice_initcall(device_init);
