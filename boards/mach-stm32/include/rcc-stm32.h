#ifndef RCC_STM32_H
#define RCC_STM32_H

#define SYSCLK		0
#define AHB_CLK		1
#define APB1_CLK	2
#define APB2_CLK	3

extern int stm32_rcc_get_freq_clk(unsigned int clk);
extern int stm32_rcc_get_pres_clk(unsigned int clk);
extern int stm32_rcc_enable_clk(int periph_base);
extern int stm32_rcc_disable_clk(int periph_base);
extern int stm32_rcc_enable_sys_clk(void);

#endif  /* RCC_STM32_H */
