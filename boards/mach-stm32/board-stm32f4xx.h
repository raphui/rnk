#ifndef BOARD_STM32F4XX_H
#define BOARD_STM32F4XX_H

#ifdef CONFIG_STM32F401
#include <stm32f401.h>
#elif defined(CONFIG_STM32F407)
#include <stm32f407.h>
#elif defined(CONFIG_STM32F429)
#include <stm32f429.h>
#endif

#include <pio.h>

/*#define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET 0x00

#ifdef CONFIG_USB_STACK
#define __IO volatile
#define USE_USB_OTG_FS
#endif /* CONFIG_USB_STACK */
 
struct pio_operations pio_ops;

#endif /* BOARD_STM32F4XX_H */
