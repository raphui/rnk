#ifndef BOARD_STM32L4XX_H
#define BOARD_STM32L4XX_H

#ifdef CONFIG_STM32L443
#include <platform/stm32l443.h>
#elif defined(CONFIG_STM32L476)
#include <platform/stm32l476.h>
#elif defined(CONFIG_STM32L442)
#include <platform/stm32l442.h>
#endif

#include <drv/pio.h>

/*#define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET 0x00

#ifdef CONFIG_USB_STACK
#define __IO volatile
#define USE_USB_OTG_FS
#endif /* CONFIG_USB_STACK */
 
extern struct pio_operations pio_ops;

void board_enter_low_power(void);
void board_exit_low_power(void);

#endif /* BOARD_STM32L4XX_H */
