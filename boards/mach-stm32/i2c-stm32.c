#include <board.h>
#include <utils.h>
#include <kernel/printk.h>
#include <drv/i2c.h>
#include <drv/device.h>
#include <mach/rcc-stm32.h>
#include <mach/pio-stm32.h>
#include <mach/dma-stm32.h>
#include <armv7m/nvic.h>
#include <fdtparse.h>
#include <init.h>
#include <errno.h>
#include <ioctl.h>

static void stm32_i2c_set_transfer_size(struct i2c_device *i2cdev, int size)
{
	struct i2c_master *i2c = i2cdev->master;
	I2C_TypeDef *I2C = (I2C_TypeDef *)i2c->base_reg;

	I2C->CR2 &= ~I2C_CR2_NBYTES;
	I2C->CR2 |= (size << 16);
}

static int stm32_i2c_write_reg(struct i2c_device *i2cdev, unsigned short reg)
{
	int ret = 0;
	struct i2c_master *i2c = i2cdev->master;
	I2C_TypeDef *I2C = (I2C_TypeDef *)i2c->base_reg;

	while (!(I2C->ISR & I2C_ISR_TXIS))
		;

	I2C->TXDR = (reg & 0xFF);

	while (!(I2C->ISR & I2C_ISR_TCR))
		;

	return ret;
}

static int stm32_i2c_read_reg(struct i2c_device *i2cdev, unsigned short reg)
{
	int ret = 0;
	struct i2c_master *i2c = i2cdev->master;
	I2C_TypeDef *I2C = (I2C_TypeDef *)i2c->base_reg;

	while (!(I2C->ISR & I2C_ISR_TXIS))
		;

	I2C->TXDR = (reg & 0xFF);

	while (!(I2C->ISR & I2C_ISR_TC))
		;

	return ret;
}

static int stm32_i2c_dma_write(struct i2c_device *i2cdev, unsigned char *buff, unsigned int size)
{
	int ret = 0;
	struct i2c_master *i2c = i2cdev->master;
	I2C_TypeDef *I2C = (I2C_TypeDef *)i2c->base_reg;

	struct dma_stream *dma = &i2c->dma_stream[I2C_TRANSFER_READ];
	struct dma_transfer *dma_trans = &i2c->dma_trans_r;

	return ret;
}

static int stm32_i2c_dma_read(struct i2c_device *i2cdev, unsigned char *buff, unsigned int size)
{
	int ret = 0;
	struct i2c_master *i2c = i2cdev->master;
	I2C_TypeDef *I2C = (I2C_TypeDef *)i2c->base_reg;

	struct dma_stream *dma = &i2c->dma_stream[I2C_TRANSFER_READ];
	struct dma_transfer *dma_trans = &i2c->dma_trans_r;

	return ret;
}

static int stm32_i2c_write(struct i2c_device *i2cdev, unsigned char *buff, unsigned int size)
{
	int ret = 0;
	int n_bytes_to_write = 0;
	unsigned char *p = buff;
	struct i2c_master *i2c = i2cdev->master;
	I2C_TypeDef *I2C = (I2C_TypeDef *)i2c->base_reg;

	I2C->CR2 &= ~I2C_CR2_RD_WRN;

	if (size > 0xFF) {
		stm32_i2c_set_transfer_size(i2cdev, 0xFF);
		I2C->CR2 |= I2C_CR2_RELOAD;
		n_bytes_to_write = 0xFF;
	} else {
		stm32_i2c_set_transfer_size(i2cdev, size);
		I2C->CR2 |= I2C_CR2_AUTOEND;
		n_bytes_to_write = size;
	}

	while (size) {

		if (size && !n_bytes_to_write) {
			while (!(I2C->ISR & I2C_ISR_TCR))
				;

			if (size > 0xFF) {
				stm32_i2c_set_transfer_size(i2cdev, 0xFF);
				I2C->CR2 |= I2C_CR2_RELOAD;
				n_bytes_to_write = 0xFF;
			} else {
				stm32_i2c_set_transfer_size(i2cdev, size);
				I2C->CR2 |= I2C_CR2_AUTOEND;
				n_bytes_to_write = size;
			}
		}

		while (!(I2C->ISR & I2C_ISR_TXE))
			;

		I2C->TXDR = *p;

		p++;
		size--;
		n_bytes_to_write--;
	}

	I2C->ISR |= I2C_ISR_STOPF;

	return ret;
}


static int stm32_i2c_read(struct i2c_device *i2cdev, unsigned char *buff, unsigned int size)
{
	int ret = 0;
	int n_bytes_to_read = 0;
	unsigned char *p = buff;
	struct i2c_master *i2c = i2cdev->master;
	I2C_TypeDef *I2C = (I2C_TypeDef *)i2c->base_reg;

	I2C->CR2 |= I2C_CR2_RD_WRN;

	if (size > 0xFF) {
		stm32_i2c_set_transfer_size(i2cdev, 0xFF);
		I2C->CR2 |= I2C_CR2_RELOAD;
		n_bytes_to_read = 0xFF;
	} else {
		stm32_i2c_set_transfer_size(i2cdev, size);
		I2C->CR2 |= I2C_CR2_AUTOEND;
		n_bytes_to_read = size;
	}

	I2C->CR2 |= I2C_CR2_START;

	while (size) {

		if (size && !n_bytes_to_read) {
			while (!(I2C->ISR & I2C_ISR_TCR))
				;

			if (size > 0xFF) {
				stm32_i2c_set_transfer_size(i2cdev, 0xFF);
				I2C->CR2 |= I2C_CR2_RELOAD | I2C_CR2_START;
				n_bytes_to_read = 0xFF;
			} else {
				stm32_i2c_set_transfer_size(i2cdev, size);
				I2C->CR2 |= I2C_CR2_AUTOEND | I2C_CR2_START;
				n_bytes_to_read = size;
			}
		}

		while (!(I2C->ISR & I2C_ISR_RXNE))
			;

		*p = I2C->RXDR;

		p++;
		size--;
		n_bytes_to_read--;
	}

	I2C->ISR |= I2C_ISR_STOPF;

	return ret;
}

static int stm32_i2c_transfer(struct i2c_device *i2cdev, struct i2c_msg *msg, int direction)
{
	int ret = 0;
	int n_bytes_to_write = 0;
	int n_bytes_to_read = 0;
	unsigned char *p = msg->buff;
	struct i2c_master *i2c = i2cdev->master;
	I2C_TypeDef *I2C = (I2C_TypeDef *)i2c->base_reg;


	if (direction == I2C_TRANSFER_READ) {
		I2C->CR2 &= ~(I2C_CR2_NBYTES | I2C_CR2_RD_WRN | I2C_CR2_AUTOEND);
		stm32_i2c_set_transfer_size(i2cdev, 0x1);
		I2C->CR2 |= I2C_CR2_START;

		stm32_i2c_read_reg(i2cdev, msg->reg);

		I2C->CR2 |= I2C_CR2_RD_WRN;

		if (msg->size > 0xFF) {
			stm32_i2c_set_transfer_size(i2cdev, 0xFF);
			I2C->CR2 |= I2C_CR2_RELOAD;
			n_bytes_to_read = 0xFF;
		} else {
			stm32_i2c_set_transfer_size(i2cdev, msg->size);
			I2C->CR2 |= I2C_CR2_AUTOEND;
			n_bytes_to_read = msg->size;
		}

		I2C->CR2 &= ~(I2C_CR2_START | I2C_CR2_STOP);
		I2C->CR2 |= I2C_CR2_START;

		while (msg->size) {
			if (msg->size && !n_bytes_to_read) {
				while (!(I2C->ISR & I2C_ISR_TCR))
					;

				if (msg->size > 0xFF) {
					stm32_i2c_set_transfer_size(i2cdev, 0xFF);
					I2C->CR2 |= I2C_CR2_RELOAD;
					n_bytes_to_read = 0xFF;
				} else {
					stm32_i2c_set_transfer_size(i2cdev, msg->size);
					I2C->CR2 |= I2C_CR2_AUTOEND;
					n_bytes_to_read = msg->size;
				}
			}

			while (!(I2C->ISR & I2C_ISR_RXNE))
				;

			*p = I2C->RXDR;

			p++;
			msg->size--;
			n_bytes_to_read--;
		}

	} else if (direction == I2C_TRANSFER_WRITE) {
		I2C->CR2 &= ~(I2C_CR2_NBYTES | I2C_CR2_RD_WRN);
		I2C->CR2 |= I2C_CR2_RELOAD;
		stm32_i2c_set_transfer_size(i2cdev, 0x1);
		I2C->CR2 |= I2C_CR2_START;

		stm32_i2c_write_reg(i2cdev, msg->reg);

		I2C->CR2 &= ~(I2C_CR2_START | I2C_CR2_STOP);

		if (msg->size > 0xFF) {
			stm32_i2c_set_transfer_size(i2cdev, 0xFF);
			I2C->CR2 |= I2C_CR2_RELOAD;
			n_bytes_to_write = 0xFF;
		} else {
			stm32_i2c_set_transfer_size(i2cdev, msg->size);
			I2C->CR2 |= I2C_CR2_AUTOEND;
			n_bytes_to_write = msg->size;
		}

		while (msg->size) {
			if (msg->size && !n_bytes_to_write) {
				while (!(I2C->ISR & I2C_ISR_TCR))
					;

				if (msg->size > 0xFF) {
					stm32_i2c_set_transfer_size(i2cdev, 0xFF);
					I2C->CR2 |= I2C_CR2_RELOAD;
					n_bytes_to_write = 0xFF;
				} else {
					stm32_i2c_set_transfer_size(i2cdev, msg->size);
					I2C->CR2 |= I2C_CR2_AUTOEND;
					n_bytes_to_write = msg->size;
				}
			}

			while (!(I2C->ISR & I2C_ISR_TXIS))
				;

			I2C->TXDR = *p;

			p++;
			msg->size--;
			n_bytes_to_write--;
		}

	} else {
		error_printk("invalid direction\n");
		ret = -EINVAL;
		goto err;
	}

	I2C->ISR |= I2C_ISR_STOPF;

err:
	return ret;
}

int stm32_i2c_ioctl(struct i2c_device *i2cdev, int request, char *arg)
{
	int ret = 0;
	struct i2c_master *i2c = i2cdev->master;
	I2C_TypeDef *I2C = (I2C_TypeDef *)i2c->base_reg;

	switch (request) {
	case IOCTL_SET_ADDRESS:
			I2C->CR1 &= ~I2C_CR1_PE;
			I2C->CR2 |= (((unsigned int)arg) & I2C_CR2_SADD);
			I2C->CR1 |= I2C_CR1_PE;
	default:
		ret = -EINVAL;	
	}


	return ret;
}

static void stm32_i2c_isr(void *arg)
{
	struct i2c_master *i2c = (struct i2c_master *)arg;

	ksem_post_isr(&i2c->sem);
}

struct i2c_operations i2c_ops = {
	.write = stm32_i2c_write,
	.read = stm32_i2c_read,
	.ioctl = stm32_i2c_ioctl,
	.transfer = stm32_i2c_transfer,
};

int stm32_i2c_of_init(struct i2c_master *i2c)
{
	int i;
	int offset;
	int ret = 0;
	const void *fdt_blob = fdtparse_get_blob();

	offset = fdt_path_offset(fdt_blob, i2c->dev.of_path);
	if (offset < 0) {
		ret = -ENOENT;
		goto out;
	}

	ret = fdt_node_check_compatible(fdt_blob, offset, i2c->dev.of_compat);
	if (ret < 0)
		goto out;

	i2c->base_reg = (unsigned int)fdtparse_get_addr32(offset, "reg");
	if (!i2c->base_reg) {
		error_printk("failed to retrieve i2c base reg from fdt\n");
		ret = -ENOENT;
		goto out;
	}

	ret = stm32_pio_of_configure(offset);
	if (ret < 0) {
		error_printk("failed to configure i2c gpio\n");
		goto out;
	}

	ret = fdtparse_get_int(offset, "interrupts", (int *)&i2c->irq);
	if (ret < 0) {
		error_printk("failed to retrieve i2c irq\n");
		ret = -EIO;
		goto out;
	}

	ret = fdtparse_get_int(offset, "master", (int *)&i2c->master);
	if (ret < 0) {
		error_printk("failed to retrieve i2c master\n");
		ret = -EIO;
		goto out;
	}

	ret = fdtparse_get_int(offset, "addressing-mode", (int *)&i2c->addressing_mode);
	if (ret < 0) {
		error_printk("failed to retrieve i2c addressing_mode\n");
		ret = -EIO;
		goto out;
	}

	ret = fdtparse_get_int(offset, "timing", (int *)&i2c->timing);
	if (ret < 0) {
		error_printk("failed to retrieve i2c timing\n");
		ret = -EIO;
		goto out;
	}

	ret = stm32_rcc_of_enable_clk(offset, &i2c->clock);
	if (ret < 0) {
		error_printk("failed to retrieve spi periph clock\n");
		ret = -EIO;
		goto out;
	}

	ret = stm32_dma_stream_of_configure(offset, stm32_i2c_isr, i2c, i2c->dma_stream, 2);
	if (ret < 0) {
		error_printk("failed to retrieve spi dma conf, switching to polling\n");
		i2c->use_dma = 0;
		ret = 0;
		goto out;
	}

	i2c->use_dma = 0;

	/* XXX: make this based on device tree or deduced */
	i2c->dma_stream[I2C_TRANSFER_READ].dir = DMA_P_M;
	i2c->dma_stream[I2C_TRANSFER_WRITE].dir = DMA_M_P;

	for (i = 0; i < 2; i++) {
		i2c->dma_stream[i].mdata_size = DATA_SIZE_BYTE;
		i2c->dma_stream[i].pdata_size = DATA_SIZE_BYTE;
		i2c->dma_stream[i].mburst = INCR0;
		i2c->dma_stream[i].pburst = INCR0;
		i2c->dma_stream[i].use_fifo = 0;
	}

out:
	return ret;


	return ret;
}

int stm32_i2c_init(struct device *device)
{
	int ret = 0;
	struct i2c_master *i2c = NULL;
	I2C_TypeDef *I2C = NULL;

	i2c = i2c_new_master();
	if (!i2c) {
		error_printk("failed to retrieve new i2c master\n");
		ret = -EIO;
		goto err;
	}

	memcpy(&i2c->dev, device, sizeof(struct device));

	ret = stm32_i2c_of_init(i2c);
	if (ret < 0) {
		error_printk("failed to init i2c with fdt data\n");
		goto err;
	}

	I2C = (I2C_TypeDef *)i2c->base_reg;

	I2C->TIMINGR = i2c->timing;

	i2c->rate = i2c->clock.source_clk;

	i2c->i2c_ops = &i2c_ops;

	if (i2c->use_dma) {
		i2c->i2c_ops->write = stm32_i2c_dma_write;
		i2c->i2c_ops->read = stm32_i2c_dma_read;
	}

	if (i2c->addressing_mode == 10)
		I2C->CR2 |= I2C_CR2_ADD10;

	I2C->CR1 |= I2C_CR1_PE;

	ret = i2c_register_master(i2c);
	if (ret < 0) {
		error_printk("failed to register i2c master\n");
		goto disable_clk;
	}

	return ret;

disable_clk:
	stm32_rcc_disable_clk(i2c->clock.gated, i2c->clock.id);
err:
	return ret;
}

struct device stm32_i2c_driver = {
	.of_compat = "st,stm32-i2c",
	.probe = stm32_i2c_init,
};

static int stm32_i2c_register(void)
{
	int ret = 0;

	ret = device_of_register(&stm32_i2c_driver);
	if (ret < 0)
		error_printk("failed to register stm32_i2c device\n");
	return ret;
}
postarch_initcall(stm32_i2c_register);
