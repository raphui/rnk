#include <board.h>
#include <mach/rcc-stm32.h>
#include <mach/pio-stm32.h>
#include <errno.h>
#include <fdtparse.h>
#include <drv/device.h>
#include <init.h>
#include <string.h>
#include <drv/usart.h>

/* Calculates the value for the USART_BRR */
static unsigned short stm32_baud_rate(long clock, unsigned int baud)
{
	unsigned int divisor = 16 * baud;
	unsigned short mantissa = clock / divisor;
	unsigned int remainder = clock % divisor;
	unsigned short fraction = (16 * remainder) / divisor;

	return (mantissa << 4) | (fraction & 0xf);
}

static void stm32_usart_print(struct usart_master *usart, unsigned char byte)
{
	USART_TypeDef *USART = (USART_TypeDef *)usart->base_reg;

#ifdef CONFIG_STM32F4XX
	while(!(USART->SR & USART_SR_TXE))
		;

	USART->DR = byte;
#else
	while(!(USART->ISR & USART_ISR_TXE))
		;

	USART->TDR = byte;

#endif
}

static int stm32_usart_printl(struct usart_master *usart, const char *string)
{
	int size = 0;

	while (*string) {
		stm32_usart_print(usart, *string++);
		size++;
	}

	return size;
}

static int stm32_usart_write(struct usart_device *usartdev, unsigned char *buff, unsigned int len)
{
	struct usart_master *usart = usartdev->master;
	USART_TypeDef *USART = (USART_TypeDef *)usart->base_reg;
	int i = 0;
	int ret = 0;

	for (i = 0; i < len;  i++) {
#ifdef CONFIG_STM32F4XX
		while(!(USART->SR & USART_SR_TXE))
			;

		USART->DR = buff[i];
#else
		while(!(USART->ISR & USART_ISR_TXE))
			;

		USART->TDR = buff[i];
#endif
	}


	return ret;
}

static int stm32_usart_read(struct usart_device *usartdev, unsigned char *buff, unsigned int len)
{
	struct usart_master *usart = usartdev->master;
	USART_TypeDef *USART = (USART_TypeDef *)usart->base_reg;
	int i = 0;
	int ret = 0;

	for (i = 0; i < len;  i++) {
#ifdef CONFIG_STM32F4XX
		while(!(USART->SR & USART_SR_RXNE))
			;

		buff[i] = USART->DR;
#else
		while(!(USART->ISR & USART_ISR_RXNE))
			;

		buff[i] = USART->RDR;
#endif
	}


	return ret;
}

struct usart_operations usart_ops = {
	.read = stm32_usart_read,
	.write = stm32_usart_write,
	.print = stm32_usart_print,
	.printl = stm32_usart_printl,
};

static int stm32_usart_of_init(struct usart_master *usart)
{
	int offset;
	int ret = 0;
	const void *fdt_blob = fdtparse_get_blob();

	offset = fdt_path_offset(fdt_blob, usart->dev.of_path);
	if (offset < 0) {
		ret = -ENOENT;
		goto out;
	}

	ret = fdt_node_check_compatible(fdt_blob, offset, usart->dev.of_compat);
	if (ret < 0)
		goto out;

	usart->base_reg = (unsigned int)fdtparse_get_addr32(offset, "reg");
	if (!usart->base_reg) {
		error_printk("failed to retrieve usart base reg from fdt\n");
		ret = -ENOENT;
		goto out;
	}

	ret = stm32_pio_of_configure(offset);
	if (ret < 0) {
		error_printk("failed to configure usart gpio\n");
		goto out;
	}

	ret = fdtparse_get_int(offset, "baudrate", (int *)&usart->baud_rate);
	if (ret < 0) {
		error_printk("failed to retrieve usart baudrate\n");
		ret = -EIO;
		goto out;
	}

	ret = fdtparse_get_int(offset, "mode", (int *)&usart->mode);
	if (ret < 0) {
		error_printk("failed to retrieve usart mode\n");
		ret = -EIO;
		goto out;
	}

	ret = stm32_rcc_of_enable_clk(offset, &usart->clock);

out:
	return ret;
}

static int stm32_usart_init(struct device *dev)
{
	int ret = 0;
	struct usart_master *usart = NULL;
	USART_TypeDef *USART = NULL;

	usart = usart_new_master();
	if (!usart) {
		error_printk("failed to retrieve new usart master\n");
		ret = -EIO;
		goto err;
	}

	memcpy(&usart->dev, dev, sizeof(struct device));
	usart->usart_ops = &usart_ops;

	ret = stm32_usart_of_init(usart);
	if (ret < 0) {
		error_printk("failed to init usart with fdt data\n");
		goto err;
	}

	USART = (USART_TypeDef *)usart->base_reg;

	USART->CR1 &= ~USART_CR1_M;
	USART->CR1 &= ~USART_CR1_UE;

	USART->BRR = stm32_baud_rate(usart->clock.source_clk, usart->baud_rate);

	USART->CR1 |= usart->mode << 2;
	USART->CR1 |= USART_CR1_UE;

	ret = usart_register_master(usart);
	if (ret < 0) {
		error_printk("failed to register stm32 usart\n");
		goto disable_clk;
	}

	return 0;

disable_clk:
	stm32_rcc_disable_clk(usart->clock.gated, usart->clock.id);
err:
	return ret;
}

struct device stm32_usart_driver = {
	.of_compat = "st,stm32-usart",
	.probe = stm32_usart_init,
};

static int stm32_usart_register(void)
{
	int ret = 0;

	ret = device_of_register(&stm32_usart_driver);
	if (ret < 0)
		error_printk("failed to register stm32_usart device\n");
	return ret;
}
postarch_initcall(stm32_usart_register);
