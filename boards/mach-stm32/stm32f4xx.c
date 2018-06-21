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

#ifdef CONFIG_SWO_DEBUG
	stm32_pio_set_alternate(GPIOB_BASE, 3, 0x0);
	swo_init(stm32_rcc_get_freq_clk(SYSCLK));
	DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN; // Enable IO trace pins
#endif /* CONFIG_SWO_DEBUG */

	return ret;
}
coredevice_initcall(device_init);
