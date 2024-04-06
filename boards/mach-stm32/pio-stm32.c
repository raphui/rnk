#include <board.h>
#include <utils.h>
#include <mach/rcc-stm32.h>
#include <mach/exti-stm32.h>
#include <errno.h>
#include <fdtparse.h>
#include <kernel/printk.h>
#include <init.h>
#include <drv/clk.h>

#define GPIO_MODER(pin)			(3 << (pin * 2))
#define GPIO_MODER_OUTPUT(pin)		(1 << (pin * 2))
#define GPIO_MODER_ALTERNATE(pin)	(2 << (pin * 2))
#define GPIO_OSPEEDR(pin)		(3 << (pin * 2))
#define GPIO_PUPDR_PULLUP(pin)		(1 << (pin * 2))

#define GPIO_OUTPUT_TYPE_PP	0
#define GPIO_OUTPUT_TYPE_OD	1

#define PIN_PER_PORT	CONFIG_GPIO_PER_PORT
#define PORT_NUM	CONFIG_GPIO_PORT_NUM

struct gpio_options
{
	unsigned char val:1;			/* 0: low, 1: high */
	unsigned char mode:1;			/* 0: in, 1: out */
	unsigned char output:1;			/* 0: pp, 1: od */
	unsigned char speed:2;			/* 0 slow ... 3 fast */
	unsigned char pull:2;			/* 0: no, 1 pu, 2, pd */
	unsigned char analog:1;
	unsigned char edge:2;
};

void stm32_pio_set_output(unsigned int port, unsigned int mask, int pull_up)
{
	GPIO_TypeDef *base = (GPIO_TypeDef *)port;

	base->MODER &= ~GPIO_MODER(mask);
	base->MODER |= GPIO_MODER_OUTPUT(mask);
	base->OSPEEDR |= GPIO_OSPEEDR(mask);

	if (pull_up)
		base->PUPDR |= GPIO_PUPDR_PULLUP(mask);
	else
		base->PUPDR &= ~(GPIO_PUPDR_PULLUP(mask));
}

static void stm32_pio_set_output_type(unsigned int port, unsigned int mask, int type)
{
	GPIO_TypeDef *base = (GPIO_TypeDef *)port;

	if (type == GPIO_OUTPUT_TYPE_OD)
		base->OTYPER |= (1 << mask);
	else if (type == GPIO_OUTPUT_TYPE_PP)
		base->OTYPER &= ~(1 << mask);

}

void stm32_pio_set_input(unsigned int port, unsigned int mask, int pull_up, int filter)
{
	GPIO_TypeDef *base = (GPIO_TypeDef *)port;
	base->MODER &= ~(GPIO_MODER(mask));

	if (pull_up)
		base->PUPDR |= GPIO_PUPDR_PULLUP(mask);
	else
		base->PUPDR &= ~(GPIO_PUPDR_PULLUP(mask));

}

void stm32_pio_set_analog(unsigned int port, unsigned int mask)
{
	GPIO_TypeDef *base = (GPIO_TypeDef *)port;
	base->MODER |= GPIO_MODER(mask);
}

void stm32_pio_set_alternate(unsigned int port, unsigned int mask, unsigned int num)
{
	GPIO_TypeDef *base = (GPIO_TypeDef *)port;
	unsigned int afr_high_base = 8;

	base->MODER &= ~GPIO_MODER(mask);
	base->MODER |= GPIO_MODER_ALTERNATE(mask);
	base->OSPEEDR |= GPIO_OSPEEDR(mask);

	if (mask > 7)
		base->AFR[1] |= (num << ((mask - afr_high_base) * 4));
	else
		base->AFR[0] |= (num << (mask * 4));
}

void stm32_pio_set_value(unsigned int port, unsigned int mask)
{
	GPIO_TypeDef *base = (GPIO_TypeDef *)port;
	base->ODR |= (1 << mask);
}

int stm32_pio_get_input_value(unsigned int port, unsigned int mask)
{
	GPIO_TypeDef *base = (GPIO_TypeDef *)port;
	return (base->IDR >> mask) & 0x1;
}

int stm32_pio_get_output_value(unsigned int port, unsigned int mask)
{
	GPIO_TypeDef *base = (GPIO_TypeDef *)port;
	return (base->ODR >> mask) & 0x1;
}


void stm32_pio_clear_value(unsigned int port, unsigned int mask)
{
	GPIO_TypeDef *base = (GPIO_TypeDef *)port;
	base->ODR &= ~(1 << mask);
}

void stm32_pio_toggle_value(unsigned int port, unsigned int mask)
{
	GPIO_TypeDef *base = (GPIO_TypeDef *)port;
	base->ODR ^= (1 << mask);
}

static int stm32_pio_request_interrupt(unsigned int port, unsigned int mask, void (*handler)(void *), int flags, void *arg)
{
	return stm32_exti_request_irq(port, mask, handler, flags, arg);
}

static void stm32_pio_enable_interrupt(unsigned int port, unsigned int mask)
{
	
}

static void stm32_pio_disable_interrupt(unsigned int port, unsigned int mask)
{
}

int stm32_pio_of_configure_name(int fdt_offset, char *name)
{
	const void *fdt_blob = fdtparse_get_blob();
	const struct fdt_property *prop;
	fdt32_t *cell;
	struct gpio_options *options;
	unsigned int base;
	int gpio;
	int gpio_num;
	int alt_func;
	unsigned short flags;
	int len, num, i;
	int parent_phandle, parent_offset;

	prop = fdt_get_property(fdt_blob, fdt_offset, name, &len);
	if (len < 0) {
		return len;
	}

	num = len / (3 * sizeof(fdt32_t));

	cell = (fdt32_t *)prop->data;

	for(i = 0; i < num; i++, cell += 3) {
		options = (struct gpio_options *)&flags;

		parent_phandle = fdt32_to_cpu(cell[0]);
		parent_offset = fdt_node_offset_by_phandle(fdt_blob, parent_phandle);

		base = (unsigned int)fdtparse_get_addr32(parent_offset, "reg");
		gpio = fdt32_to_cpu(cell[1]);
		flags = fdt32_to_cpu(cell[2]);

		gpio_num = gpio & 0xFF;

		if (options->analog)
			stm32_pio_set_analog(base, gpio_num);
		else {

			if (gpio & 0xF00) {
				alt_func = (gpio >> 8) & 0xF;
				stm32_pio_set_alternate(base, gpio_num, alt_func);
			} else {
				if (options->mode) {
					if (options->val)
						stm32_pio_set_value(base, gpio_num);
					else
						stm32_pio_clear_value(base, gpio_num);

					stm32_pio_set_output_type(base, gpio_num, options->output);

					stm32_pio_set_output(base, gpio_num, options->pull);
				}
				else
					stm32_pio_set_input(base, gpio_num, options->pull, 0);
			}
		}
	}

	return num;
}

int stm32_pio_of_configure(int fdt_offset)
{
	return stm32_pio_of_configure_name(fdt_offset, "gpios");
}

int stm32_pio_of_get(int fdt_offset, char *name, unsigned int *port, unsigned int *pin)
{
	const void *fdt_blob = fdtparse_get_blob();
	const struct fdt_property *prop;
	fdt32_t *cell;
	unsigned int base;
	int gpio;
	int gpio_num;
	int len, num;
	int parent_phandle, parent_offset;

	prop = fdt_get_property(fdt_blob, fdt_offset, name, &len);
	if (len < 0) {
		return len;
	}

	num = len / (3 * sizeof(fdt32_t));
	if (num > 1)
		return -ENOTSUP;

	cell = (fdt32_t *)prop->data;

	parent_phandle = fdt32_to_cpu(cell[0]);
	parent_offset = fdt_node_offset_by_phandle(fdt_blob, parent_phandle);

	base = (unsigned int)fdtparse_get_addr32(parent_offset, "reg");
	gpio = fdt32_to_cpu(cell[1]);

	gpio_num = gpio & 0xFF;

	*port = base;
	*pin = gpio_num;

	return 0;
}

int stm32_pio_export(unsigned int pin_num, unsigned int *port, unsigned int *pin)
{
	int v;

	*port = pin_num / PIN_PER_PORT;

	if (*port > PORT_NUM)
		return -EINVAL;

	*port = GPIOA_BASE + (*port * 0x400);

	*pin = PIN_PER_PORT - (pin_num % PIN_PER_PORT);

	v = pin_num % PIN_PER_PORT;
	if (v)
		*pin = v - 1;
	else
		*pin = PIN_PER_PORT;

	return 0;
}

struct pio_operations pio_ops = {
	.set_output = stm32_pio_set_output,
	.set_input = stm32_pio_set_input,
	.set_alternate = stm32_pio_set_alternate,
	.set_value = stm32_pio_set_value,
	.clear_value = stm32_pio_clear_value,
	.toggle_value = stm32_pio_toggle_value,
	.get_input_value = stm32_pio_get_input_value,
	.get_output_value = stm32_pio_get_output_value,
	.request_interrupt = stm32_pio_request_interrupt,
	.enable_interrupt = stm32_pio_enable_interrupt,
	.disable_interrupt = stm32_pio_disable_interrupt,
	.of_configure = stm32_pio_of_configure,
	.of_configure_name = stm32_pio_of_configure_name,
	.of_get = stm32_pio_of_get,
	.export_pio = stm32_pio_export,
};

static int stm32_pio_init(struct device *dev)
{
	int offset;
	struct clk clock;
	int ret = 0;
	const void *fdt_blob = fdtparse_get_blob();

	offset = fdt_path_offset(fdt_blob, dev->of_path);
	if (offset < 0) {
		ret = -ENOENT;
		goto err;
	}

	ret = fdt_node_check_compatible(fdt_blob, offset, dev->of_compat);
	if (ret < 0)
		goto err;


	ret = stm32_rcc_of_enable_clk(offset, &clock);
	if (ret < 0) {
		error_printk("failed to retrieve pio clock\n");
		ret = -EIO;
	}

err:
	return ret;
}

struct device stm32_pio_driver = {
	.of_compat = "st,stm32-pio",
	.probe = stm32_pio_init,
};

static int stm32_pio_register(void)
{
	int ret = 0;

	ret = device_of_register(&stm32_pio_driver);
	if (ret < 0)
		error_printk("failed to register stm32_pio device\n");
	return ret;
}
postarch_initcall(stm32_pio_register);
