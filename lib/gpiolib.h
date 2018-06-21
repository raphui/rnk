#ifndef GPIOLIB_H
#define GPIOLIB_H

#include <pio.h>

struct pio_desc *gpiolib_export(unsigned int gpio_num);
int gpiolib_set_output(struct pio_desc *desc, int gpio_value);
int gpiolib_set_input(struct pio_desc *desc);
int gpiolib_output_set_value(struct pio_desc *desc, int gpio_value);

#endif /* GPIOLIB_H */
