#ifndef RCC_STM32_H
#define RCC_STM32_H

#include <drv/clk.h>

#if defined(CONFIG_STM32F4XX) | defined(CONFIG_STM32F7xx)
#include <dt-bindings/stm32fx-clock.h>
#elif defined(CONFIG_STM32L4XX)
#include <dt-bindings/stm32lx-clock.h>
#endif


extern int stm32_rcc_get_freq_clk(unsigned int clk);
extern int stm32_rcc_get_pres_clk(unsigned int clk);
extern int stm32_rcc_enable_clk(int secondary, int id);
extern int stm32_rcc_disable_clk(int secondary, int id);
extern int stm32_rcc_of_enable_clk(int offset, struct clk *clk);
extern int stm32_rcc_enable_sys_clk(void);

#endif  /* RCC_STM32_H */
