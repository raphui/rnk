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
#include <utils.h>
#include <printk.h>
#include <mach/dma-stm32.h>
#include <mach/rcc-stm32.h>
#include <mach/pio-stm32.h>
#include <armv7m/nvic.h>
#include <errno.h>
#include <fdtparse.h>
#include <init.h>
#include <device.h>
#include <spi.h>

static short stm32_spi_find_best_pres(unsigned long parent_rate, unsigned long rate)
{
	unsigned int i;
	unsigned short pres[] = {2, 4, 8, 16, 32, 64, 128, 256};
	unsigned int diff;
	unsigned int best_diff;
	unsigned long curr_rate;


	best_diff = parent_rate - rate;

	for (i = 0; i < 8; i++) {
		curr_rate = parent_rate / pres[i];

		if (curr_rate < rate)
			diff = rate - curr_rate;
		else
			diff = curr_rate - rate;

		if (diff < best_diff) {
			best_diff = diff;
		}

		if (!best_diff || curr_rate < rate)
			break;
	}

	return i;
}

static int stm32_spi_dma_write(struct spi_device *spidev, unsigned char *buff, unsigned int size)
{
	struct spi_master *spi = spidev->master;
	SPI_TypeDef *SPI = (SPI_TypeDef *)spi->base_reg;

	struct dma_stream *dma = &spi->dma_stream[SPI_TRANSFER_WRITE];
	struct dma_transfer *dma_trans = &spi->dma_trans_w;

	int ret = size;

	SPI->CR1 &= ~SPI_CR1_SPE;

	stm32_dma_stream_init(dma);

	dma_trans->src_addr = (unsigned int)buff;
	dma_trans->dest_addr = (unsigned int)&SPI->DR;
	dma_trans->size = size;

	stm32_dma_transfer(dma, dma_trans);

	SPI->CR2 |= SPI_CR2_TXDMAEN;
	SPI->CR2 |= SPI_CR2_RXDMAEN;

	SPI->CR1 |= SPI_CR1_SPE;

	dma->enable_interrupt = 1;

	stm32_dma_enable(dma);

	ksem_wait(&spidev->master->sem);

	stm32_dma_disable(dma);

	return ret;
}

static int stm32_spi_dma_read(struct spi_device *spidev, unsigned char *buff, unsigned int size)
{
	struct spi_master *spi = spidev->master;
	SPI_TypeDef *SPI = (SPI_TypeDef *)spi->base_reg;

	struct dma_stream *dma = &spi->dma_stream[SPI_TRANSFER_READ];
	struct dma_transfer *dma_trans = &spi->dma_trans_r;

	int ret = size;

	SPI->CR1 &= ~SPI_CR1_SPE;

	stm32_dma_stream_init(dma);

	dma_trans->src_addr = (unsigned int)&SPI->DR;
	dma_trans->dest_addr = (unsigned int)buff;
	dma_trans->size = size;

	stm32_dma_transfer(dma, dma_trans);

	SPI->CR2 |= SPI_CR2_TXDMAEN;
	SPI->CR2 |= SPI_CR2_RXDMAEN;

	SPI->CR1 |= SPI_CR1_SPE;

	dma->enable_interrupt = 1;

	stm32_dma_enable(dma);

	ksem_wait(&spidev->master->sem);

	stm32_dma_disable(dma);

	return ret;
}

static int stm32_spi_dma_exchange(struct spi_device *spidev, unsigned char *in, unsigned char *out, unsigned int size)
{
	struct spi_master *spi = spidev->master;
	SPI_TypeDef *SPI = (SPI_TypeDef *)spi->base_reg;

	struct dma_stream *dma_r = &spi->dma_stream[SPI_TRANSFER_READ];
	struct dma_stream *dma_w = &spi->dma_stream[SPI_TRANSFER_WRITE];
	struct dma_transfer *dma_trans_r = &spi->dma_trans_r;
	struct dma_transfer *dma_trans_w = &spi->dma_trans_w;

	int ret = size;

	SPI->CR1 &= ~SPI_CR1_SPE;

	stm32_dma_stream_init(dma_r);
	stm32_dma_stream_init(dma_w);

	dma_trans_w->src_addr = (unsigned int)out;
	dma_trans_w->dest_addr = (unsigned int)&SPI->DR;
	dma_trans_w->size = size;

	dma_trans_r->src_addr = (unsigned int)&SPI->DR;
	dma_trans_r->dest_addr = (unsigned int)in;
	dma_trans_r->size = size;

	stm32_dma_transfer(dma_w, dma_trans_w);
	stm32_dma_transfer(dma_r, dma_trans_r);

	SPI->CR2 |= SPI_CR2_TXDMAEN;
	SPI->CR2 |= SPI_CR2_RXDMAEN;

	dma_r->enable_interrupt = 1;
	dma_w->enable_interrupt = 0;

	stm32_dma_enable(dma_w);
	stm32_dma_enable(dma_r);

	SPI->CR1 |= SPI_CR1_SPE;

	ksem_wait(&spidev->master->sem);

	stm32_dma_disable(dma_w);
	stm32_dma_disable(dma_r);

	return ret;
}

static int stm32_spi_write(struct spi_device *spidev, unsigned char *buff, unsigned int size)
{
	struct spi_master *spi = spidev->master;
	SPI_TypeDef *SPI = (SPI_TypeDef *)spi->base_reg;
	int i;

	for (i = 0; i < size; i++) {
		while (!(SPI->SR & SPI_SR_TXE))
			;

		while (SPI->SR & SPI_SR_BSY)
			;

		SPI->DR = buff[i];

		while (!(SPI->SR & SPI_SR_RXNE))
			;

		while (SPI->SR & SPI_SR_BSY)
			;

		buff[i] = SPI->DR;
	}

	return i;
}

static int stm32_spi_read(struct spi_device *spidev, unsigned char *buff, unsigned int size)
{
	struct spi_master *spi = spidev->master;
	SPI_TypeDef *SPI = (SPI_TypeDef *)spi->base_reg;
	int i;

	for (i = 0; i < size; i++) {
		while (!(SPI->SR & SPI_SR_TXE))
			;

		while (SPI->SR & SPI_SR_BSY)
			;

		/* write dummy bytes */
		SPI->DR = 0xFF;

		while (!(SPI->SR & SPI_SR_RXNE))
			;

		while (SPI->SR & SPI_SR_BSY)
			;

		buff[i] = SPI->DR;
	}

	return i;
}

static int stm32_spi_exchange(struct spi_device *spidev, unsigned char *in, unsigned char *out, unsigned int size)
{
	struct spi_master *spi = spidev->master;
	SPI_TypeDef *SPI = (SPI_TypeDef *)spi->base_reg;
	int i;

	for (i = 0; i < size; i++) {
		while (!(SPI->SR & SPI_SR_TXE))
			;

		while (SPI->SR & SPI_SR_BSY)
			;

		SPI->DR = out[i];

		while (!(SPI->SR & SPI_SR_RXNE))
			;

		while (SPI->SR & SPI_SR_BSY)
			;

		in[i] = SPI->DR;
	}

	return i;
}

static void stm32_spi_isr(void *arg)
{
	struct spi_master *spi = (struct spi_master *)arg;

	ksem_post(&spi->sem);
}

struct spi_operations spi_ops = {
	.write = stm32_spi_write,
	.read = stm32_spi_read,
	.exchange = stm32_spi_exchange,
};

int stm32_spi_of_init(struct spi_master *spi)
{
	int i;
	int offset;
	int ret = 0;
	const void *fdt_blob = fdtparse_get_blob();

	offset = fdt_path_offset(fdt_blob, spi->dev.of_path);
	if (offset < 0) {
		ret = -ENOENT;
		goto out;
	}

	ret = fdt_node_check_compatible(fdt_blob, offset, spi->dev.of_compat);
	if (ret < 0)
		goto out;

	spi->base_reg = (unsigned int)fdtparse_get_addr32(offset, "reg");
	if (!spi->base_reg) {
		error_printk("failed to retrieve spi base reg from fdt\n");
		ret = -ENOENT;
		goto out;
	}

	ret = stm32_pio_of_configure(offset);
	if (ret < 0) {
		error_printk("failed to configure spi gpio\n");
		goto out;
	}

	ret = fdtparse_get_int(offset, "clock", (int *)&spi->source_clk);
	if (ret < 0) {
		error_printk("failed to retrieve spi source clk\n");
		ret = -EIO;
		goto out;
	}

	spi->source_clk = stm32_rcc_get_freq_clk(spi->source_clk);

	ret = fdtparse_get_int(offset, "interrupts", (int *)&spi->irq);
	if (ret < 0) {
		error_printk("failed to retrieve spi irq\n");
		ret = -EIO;
		goto out;
	}

	ret = fdtparse_get_int(offset, "mode", (int *)&spi->mode);
	if (ret < 0) {
		error_printk("failed to retrieve spi mode\n");
		ret = -EIO;
		goto out;
	}

	ret = fdtparse_get_int(offset, "speed", (int *)&spi->speed);
	if (ret < 0) {
		error_printk("failed to retrieve spi speed\n");
		ret = -EIO;
		goto out;
	}

	ret = stm32_dma_stream_of_configure(offset, stm32_spi_isr, spi, spi->dma_stream, 2);
	if (ret < 0) {
		error_printk("failed to retrieve spi dma conf, switching to polling\n");
		spi->use_dma = 0;
		ret = 0;
		goto out;
	}

	spi->use_dma = 1;

	/* XXX: make this based on device tree or deduced */
	spi->dma_stream[SPI_TRANSFER_READ].dir = DMA_P_M;
	spi->dma_stream[SPI_TRANSFER_WRITE].dir = DMA_M_P;

	for (i = 0; i < 2; i++) {
		spi->dma_stream[i].mdata_size = DATA_SIZE_BYTE;
		spi->dma_stream[i].pdata_size = DATA_SIZE_BYTE;
		spi->dma_stream[i].mburst = INCR0;
		spi->dma_stream[i].pburst = INCR0;
		spi->dma_stream[i].use_fifo = 0;
	}

out:
	return ret;
}

int stm32_spi_init(struct device *device)
{
	int ret = 0;
	struct spi_master *spi = NULL;
	SPI_TypeDef *SPI = NULL;

	spi = spi_new_master();
	if (!spi) {
		error_printk("failed to retrieve new spi master\n");
		ret = -EIO;
		goto err;
	}

	memcpy(&spi->dev, device, sizeof(struct device));

	ret = stm32_spi_of_init(spi);
	if (ret < 0) {
		error_printk("failed to init spi with fdt data\n");
		goto err;
	}

	SPI = (SPI_TypeDef *)spi->base_reg;

	ret = stm32_rcc_enable_clk(spi->base_reg);
	if (ret < 0) {
		error_printk("cannot enable SPI periph clock\r\n");
		goto err;
	}

	spi->rate = spi->source_clk;
	spi->rate = stm32_spi_find_best_pres(spi->rate, spi->speed);

	spi->spi_ops = &spi_ops;

	if (spi->use_dma) {
		spi->spi_ops->write = stm32_spi_dma_write;
		spi->spi_ops->read = stm32_spi_dma_read;
		spi->spi_ops->exchange = stm32_spi_dma_exchange;
	}

	SPI->CR1 &= ~SPI_CR1_SPE;

	SPI->CR1 |= (spi->rate << 3);

	/* Set master mode */
	SPI->CR1 |= SPI_CR1_MSTR;

	/* Handle slave selection via software */
	SPI->CR1 |= SPI_CR1_SSM;
	SPI->CR1 |= SPI_CR1_SSI;

	SPI->CR1 |= spi->mode;

	SPI->CR1 |= SPI_CR1_SPE;

	ret = spi_register_master(spi);
	if (ret < 0) {
		error_printk("failed to register spi master\n");
		goto disable_clk;
	}

	return ret;

disable_clk:
	stm32_rcc_disable_clk(spi->base_reg);
err:
	return ret;
}

struct device stm32_spi_driver = {
	.of_compat = "st,stm32f4xx-spi",
	.probe = stm32_spi_init,
};

static int stm32_spi_register(void)
{
	int ret = 0;

	ret = device_of_register(&stm32_spi_driver);
	if (ret < 0)
		error_printk("failed to register stm32_spi device\n");
	return ret;
}
postarch_initcall(stm32_spi_register);
