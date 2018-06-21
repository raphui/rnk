#ifndef PIO_STM32_H
#define PIO_STM32_H

extern void stm32_pio_set_output(unsigned int port, unsigned int mask, int pull_up);
extern void stm32_pio_set_input(unsigned int port, unsigned int mask, int pull_up, int filter);
extern void stm32_pio_set_alternate(unsigned int port, unsigned int mask, unsigned int num);
extern void stm32_pio_set_value(unsigned int port, unsigned int mask);
extern void stm32_pio_clear_value(unsigned int port, unsigned int mask);
extern void stm32_pio_toggle_value(unsigned int port, unsigned int mask);

extern int stm32_pio_of_configure(int fdt_offset);

#endif /* PIO_STM32_H */
