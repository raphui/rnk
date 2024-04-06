#ifndef PIO_H
#define PIO_H

#include <drv/device.h>

#define IRQF_RISING	1
#define IRQF_FALLING	2

#define GPIO_MODE_OUTPUT	0
#define GPIO_MODE_INPUT		1

struct pio_operations
{
	void (*set_output)(unsigned int port, unsigned int mask, int pull_up);
	void (*set_input)(unsigned int port, unsigned int mask, int pull_up, int filter);
	void (*set_alternate)(unsigned int port, unsigned int mask, unsigned int num);
	void (*set_value)(unsigned int port, unsigned int mask);
	void (*clear_value)(unsigned int port, unsigned int mask);
	void (*toggle_value)(unsigned int port, unsigned int mask);
	int (*get_input_value)(unsigned int port, unsigned int mask);
	int (*get_output_value)(unsigned int port, unsigned int mask);
	int (*request_interrupt)(unsigned int port, unsigned int mask, void (*handler)(void *), int flags, void *arg);
	void (*enable_interrupt)(unsigned int port, unsigned int mask);
	void (*disable_interrupt)(unsigned int port, unsigned int mask);
	int (*of_configure)(int fdt_offset);
	int (*of_configure_name)(int fdt_offset, char *name);
	int (*of_get)(int fdt_offset, char *name, unsigned int *port, unsigned int *pin);
	int (*export_pio)(unsigned int pin_num, unsigned int *port, unsigned int *pin);
};

struct pio {
	struct pio_operations *pio_ops;
	struct device dev;
};

struct pio_irq {
	void (*handler)(void *);
	void *arg;
	int flags;
};

struct pio_desc {
	unsigned int pin;
	unsigned int port;
	unsigned int mode;
	unsigned int state;
	struct pio_irq irq;
};

int pio_init(struct pio *pio);
void pio_set_output(unsigned int port, unsigned int mask, int pull_up);
void pio_set_input(unsigned int port, unsigned int mask, int pull_up, int filter);
void pio_set_alternate(unsigned int port, unsigned int mask, unsigned int num);
void pio_set_value(unsigned int port, unsigned int mask);
void pio_clear_value(unsigned int port, unsigned int mask);
void pio_toggle_value(unsigned int port, unsigned int mask);
int pio_request_interrupt(unsigned int port, unsigned int mask, struct pio_irq *irq);
void pio_enable_interrupt(unsigned int port, unsigned int mask);
void pio_disable_interrupt(unsigned int port, unsigned int mask);
int pio_of_configure(int fdt_offset);
int pio_of_configure_name(int fdt_offset, char *name);
int pio_of_get(int fdt_offset, char *name, unsigned int *port, unsigned int *pin);
int pio_export(unsigned int pin, struct pio_desc *desc);
int pio_set_state(struct pio_desc *desc);
int pio_get_state(struct pio_desc *desc);

#endif /* PIO_H */
