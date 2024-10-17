#include <board.h>
#include <utils.h>
#include <init.h>
#include <sizes.h>
#include <mach/pwr-stm32.h>
#include <mach/rcc-stm32.h>
#include <armv7m/system.h>
#include <armv7m/mpu.h>

extern unsigned int g_pfnVectors[];

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
	writel(SCB_VTOR, (unsigned int)g_pfnVectors); /* Vector Table Relocation in Internal FLASH */
}

void board_enter_low_power(void)
{
	/* XXX: Enable DBG under sleep mode */
	*(volatile unsigned int *)0xE0042004 |= (1 << 1);

	stm32_rcc_set_sysclk_freq(200000);

	stm32_pwr_enter_lpsleep(STOP1_MODE);

	EXTI->PR1 = 0xFFFFFFFF;
	EXTI->PR2 = 0xFFFFFFFF;
}

void board_exit_low_power(void)
{
	stm32_pwr_exit_lpsleep(STOP1_MODE);

	stm32_rcc_set_sysclk_freq(32000000);
}

int device_init(void)
{
	int ret = 0;

	return ret;
}
coredevice_initcall(device_init);
