#ifndef PIO_H
#define PIO_H

void pio_set_output(unsigned int port, unsigned int mask);
void pio_set_input(unsigned int port, unsigned int mask);
void pio_set_value(unsigned int port, unsigned int mask);
void pio_clear_value(unsigned int port, unsigned int mask);

#endif /* PIO_H */
