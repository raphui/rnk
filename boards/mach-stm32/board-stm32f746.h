#ifndef BOARD_STM32F746_H
#define BOARD_STM32F746_H

#include <stm32f746.h>
#include <usart.h>
#include <pio.h>
#include <timer.h>
#include <ltdc.h>
#include <spi.h>
#include <dma.h>
#include <mtd.h>


/* SYSCLK = PLL_VCO / PLL_P =======> 168000000*/
/* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N  =======> 336000000 */

#define SYSCLK		168000000
#define HCLK		168000000
#define AHB_PRES	1
#define APB1_PRES	4
#define APB2_PRES	2
#define HSE		8000000

/*#define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET 0x00
#define PLL_M		10
#define PLL_Q		2
#define PLL_N		210
#define PLL_P		2
#define HSE_STARTUP_TIMEOUT    0x05000
#define RESET		0

#define APB1_CLK	(SYSCLK / AHB_PRES) / APB1_PRES
#define APB2_CLK	(SYSCLK / AHB_PRES) / APB2_PRES
 
struct usart_operations usart_ops;
struct pio_operations pio_ops;
struct timer_operations tim_ops;
struct spi_operations spi_ops;
struct lcd_operations lcd_ops;
struct dma_operations dma_ops;
struct mtd_operations mtd_ops;

#endif /* BOARD_STM32F746_H */
