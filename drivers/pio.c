#include <board.h>
#include <drv/pio.h>
#include <mm/mm.h>
#include <kernel/printk.h>
#include <string.h>
#include <errno.h>

void pio_set_output(unsigned int port, unsigned int mask, int pull_up)
{
	pio_ops.set_output(port, mask, pull_up);
}

void pio_set_input(unsigned int port, unsigned int mask, int pull_up, int filter)
{
	pio_ops.set_input(port, mask, pull_up, filter);
}

void pio_set_alternate(unsigned int port, unsigned int mask, unsigned int num)
{
	pio_ops.set_alternate(port, mask, num);
}

void pio_set_value(unsigned int port, unsigned int mask)
{
	pio_ops.set_value(port, mask);
}

void pio_clear_value(unsigned int port, unsigned int mask)
{
	pio_ops.clear_value(port, mask);
}

void pio_toggle_value(unsigned int port, unsigned int mask)
{
	pio_ops.toggle_value(port, mask);
}

int pio_request_interrupt(unsigned int port, unsigned int mask, struct pio_irq *irq)
{
	int ret = 0;

	ret =  pio_ops.request_interrupt(port, mask, irq->handler, irq->flags, irq->arg);
	if (ret < 0)
		return ret;

	pio_enable_interrupt(port, mask);

	return ret;
}

void pio_enable_interrupt(unsigned int port, unsigned int mask)
{
	pio_ops.enable_interrupt(port, mask);
}

void pio_disable_interrupt(unsigned int port, unsigned int mask)
{
	pio_ops.disable_interrupt(port, mask);
}

int pio_of_configure(int fdt_offset)
{
	return pio_ops.of_configure(fdt_offset);
}

int pio_of_configure_name(int fdt_offset, char *name)
{
	return pio_ops.of_configure_name(fdt_offset, name);
}

int pio_of_get(int fdt_offset, char *name, unsigned int *port, unsigned int *pin)
{
	return pio_ops.of_get(fdt_offset, name, port, pin);
}

int pio_set_state(struct pio_desc *desc)
{
	int ret = 0;

	if (!desc)
		return -EINVAL;

	if (desc->mode == GPIO_MODE_OUTPUT) {
		pio_set_output(desc->port, desc->pin, desc->pull_up);

		if (desc->state)
			pio_set_value(desc->port, desc->pin);
		else
			pio_clear_value(desc->port, desc->pin);
	}
	else if (desc->mode == GPIO_MODE_INPUT)
		pio_set_input(desc->port, desc->pin, desc->pull_up, 0);
	else
		ret = -EINVAL;

	return ret;
}

int pio_get_state(struct pio_desc *desc)
{
	int ret = 0;

	if (!desc)
		return -EINVAL;

	if (desc->mode == GPIO_MODE_OUTPUT)
		ret = pio_ops.get_output_value(desc->port, desc->pin);
	else if (desc->mode == GPIO_MODE_INPUT)
		ret = pio_ops.get_input_value(desc->port, desc->pin);
	else
		ret = -EINVAL;

	return ret;
}


int pio_export(unsigned int pin, struct pio_desc *desc)
{
	int ret;
	unsigned int gpio, port;

	if (!desc)
		return -EINVAL;

	ret = pio_ops.export_pio(pin, &port, &gpio);
	if (ret < 0)
		return ret;

	desc->pin = gpio;
	desc->port = port;
	desc->pull_up = GPIO_NO_PULL;

	return 0;
}

int pio_init(struct pio *pio)
{
	int ret = 0;
	struct pio *piodev = NULL;

	piodev = (struct pio *)kmalloc(sizeof(struct pio));
	if (!piodev) {
		error_printk("cannot allocate pio\n");
		return -ENOMEM;
	}

	memcpy(piodev, pio, sizeof(struct pio));

	ret = device_register(&piodev->dev);
	if (ret < 0) {
		error_printk("failed to register device\n");
		ret = -ENOMEM;
		goto failed_out;
	}

	return ret;

failed_out:
	kfree(piodev);
	return ret;
}
