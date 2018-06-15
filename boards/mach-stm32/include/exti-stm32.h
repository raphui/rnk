#ifndef EXTI_STM32_H
#define EXTI_STM32_H

extern void stm32_exti_init(void);
extern int stm32_exti_configure(unsigned int line, unsigned int edge);
extern int stm32_exti_configure_line(unsigned int gpio_base, unsigned int gpio_num);
extern void stm32_exti_enable_falling(unsigned int gpio_base, unsigned int gpio_num);
extern void stm32_exti_enable_rising(unsigned int gpio_base, unsigned int gpio_num);
int stm32_exti_request_irq(unsigned int gpio_base, unsigned int gpio_num, void (*handler)(void *), int flags, void *arg);
extern void stm32_exti_disable_falling(unsigned int gpio_base, unsigned int gpio_num);
extern void stm32_exti_disable_rising(unsigned int gpio_base, unsigned int gpio_num);

#endif /* EXTI_STM32_H */
