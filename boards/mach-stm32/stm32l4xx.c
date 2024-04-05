#include <board.h>
#include <utils.h>
#include <init.h>
#include <sizes.h>
#include <armv7m/system.h>
#include <armv7m/mpu.h>

void low_level_init(void)
{
	/* Set MSION bit */
	RCC->CR |= RCC_CR_MSION;

	/* Reset CFGR register */
	RCC->CFGR = 0x00000000;

	/* Reset HSEON, CSSON , HSION, and PLLON bits */
	RCC->CR &= (uint32_t)0xEAF6FFFF;

	/* Reset PLLCFGR register */
	RCC->PLLCFGR = 0x00001000;

	/* Reset HSEBYP bit */
	RCC->CR &= (uint32_t)0xFFFBFFFF;

	/* Disable all interrupts */
	RCC->CIER = 0x00000000;

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

	return ret;
}
coredevice_initcall(device_init);
