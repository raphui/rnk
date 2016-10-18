/*
 * Copyright (C) 2015  Raphaël Poggi <poggi.raph@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Frrestore * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <board.h>
#include <mach/rcc-stm32.h>
#include <mach/pio-stm32.h>
#include <errno.h>
#include <fdtparse.h>
#include <device.h>
#include <init.h>
#include <string.h>
#include <usart.h>

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

	while(!(USART->SR & USART_SR_TXE))
		;

	USART->DR = byte;
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

static int stm32_usart_write(struct usart_master *usart, unsigned char *buff, unsigned int len)
{
	USART_TypeDef *USART = (USART_TypeDef *)usart->base_reg;
	int i = 0;
	int ret = 0;
	int timeout = 1000;

	for (i = 0; i < len;  i++) {
		while(!(USART->SR & USART_SR_TXE) && timeout--)
			;

		if (!timeout) {
			ret = -EIO;
			break;
		}

		USART->DR = buff[i];
	}


	return ret;
}

static int stm32_usart_read(struct usart_master *usart, unsigned char *buff, unsigned int len)
{
	USART_TypeDef *USART = (USART_TypeDef *)usart->base_reg;
	int i = 0;
	int ret = 0;
	int timeout = 1000;

	for (i = 0; i < len;  i++) {
		while(!(USART->SR & USART_SR_RXNE) && timeout--)
			;

		if (!timeout) {
			ret = -EIO;
			break;
		}

		buff[i] = USART->DR;
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

	ret = fdtparse_get_int(offset, "clock", (int *)&usart->source_clk);
	if (ret < 0) {
		error_printk("failed to retrieve usart source clk\n");
		ret = -EIO;
		goto out;
	}

	usart->source_clk = stm32_rcc_get_freq_clk(usart->source_clk);

	ret = fdtparse_get_int(offset, "baudrate", (int *)&usart->baud_rate);
	if (ret < 0) {
		error_printk("failed to retrieve usart baudrate\n");
		ret = -EIO;
		goto out;
	}

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

	ret = stm32_rcc_enable_clk(usart->base_reg);
	if (ret < 0) {
		error_printk("cannot enable clk for usart\r\n");
		goto err;
	}

	USART->CR1 &= ~USART_CR1_M;
	USART->CR1 &= ~USART_CR1_UE;

	USART->BRR = stm32_baud_rate(usart->source_clk, usart->baud_rate);

	USART->CR1 |= USART_CR1_RE;
	USART->CR1 |= USART_CR1_TE;
	USART->CR1 |= USART_CR1_UE;

	ret = usart_register_master(usart);
	if (ret < 0) {
		error_printk("failed to register stm32 usart\n");
		goto disable_clk;
	}

	return 0;

disable_clk:
	stm32_rcc_disable_clk(usart->base_reg);
err:
	return ret;
}

struct device stm32_usart_driver = {
	.of_compat = "st,stm32f4xx-usart",
	.probe = stm32_usart_init,
};

static int stm32_usart_register(void)
{
	int ret = 0;

	ret = device_register(&stm32_usart_driver);
	if (ret < 0)
		error_printk("failed to register stm32_usart device\n");
	return ret;
}
#ifdef CONFIG_INITCALL
pure_initcall(stm32_usart_register);
#endif /* CONFIG_INITCALL */
