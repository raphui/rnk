#ifndef GPIOLIB_H
#define GPIOLIB_H

#include <drv/pio.h>

#define IRQF_RISING	1
#define IRQF_FALLING	2

struct pio_desc *gpiolib_export(unsigned int gpio_num);
int gpiolib_request_irq(struct pio_desc *desc, void (*handler)(void *), int flags, void *arg);
int gpiolib_set_pull_up(struct pio_desc *desc, int value);
int gpiolib_set_output(struct pio_desc *desc, int gpio_value);
int gpiolib_set_input(struct pio_desc *desc);
int gpiolib_output_set_value(struct pio_desc *desc, int gpio_value);
int gpiolib_get_value(struct pio_desc *desc);
int gpiolib_enable_irq(struct pio_desc *desc);
int gpiolib_disable_irq(struct pio_desc *desc);

#endif /* GPIOLIB_H */
