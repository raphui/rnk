#include <kernel/printk.h>
#include <kernel/ktime.h>
#include <drv/spi.h>
#include <mm/mm.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <utils.h>
#include <init.h>
#include <ioctl.h>
#include <fdtparse.h>
#include <drv/pio.h>
#include <kernel/ksem.h>
#include <drv/irq.h>
#include <drv/lr1110.h>

enum {
	NSS_GPIO,
	RESET_GPIO,
	GPIO_N,
};

struct lr1110_transfer {
	const uint8_t *command;
	uint16_t command_length;
	const uint8_t *data;
	uint16_t data_length;
};

struct lr1110_gpio {
	unsigned int pin;
	unsigned int port;
};

struct lr1110_priv {
	struct lr1110_gpio gpios[GPIO_N];
};

static uint8_t lr1110_compute_crc(const uint8_t crc_initial_value, const uint8_t* buffer, uint16_t length)
{
	uint8_t crc = crc_initial_value;
	uint8_t extract;
	uint8_t sum;

	for (int i = 0; i < length; i++) {
		extract = *buffer;

		for (uint8_t j = 8; j; j--) {
			sum = (crc ^ extract) & 0x01;
			crc >>= 1;

			if (sum) {
				crc ^= 0x65;
			}

			extract >>= 1;
		}

		buffer++;
	}

	return crc;
}


static int lr1110_read(struct device *dev, unsigned char *buff, unsigned int size)
{
	int ret = 0;
	unsigned char crc = 0;
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct lr1110_priv *priv = (struct lr1110_priv *)spi->priv;
	struct lr1110_transfer *transfer = (struct lr1110_transfer *)buff;

	/* NSS low */
	pio_clear_value(priv->gpios[NSS_GPIO].port, priv->gpios[NSS_GPIO].pin);

	/* Send CMD */
	for (int i = 0; i < transfer->command_length; i++) {
		spi_transfer(spi, (unsigned char *)&transfer->command[i], sizeof(unsigned char));
	}

	/* Compute and send CRC */
	crc = lr1110_compute_crc(0xFF, transfer->command, transfer->command_length);

	/* Send CRC */
	spi_transfer(spi, (unsigned char *)&crc, sizeof(unsigned char));

	/* NSS high */
	pio_set_value(priv->gpios[NSS_GPIO].port, priv->gpios[NSS_GPIO].pin);

	return ret;
}

static int lr1110_write(struct device *dev, unsigned char *buff, unsigned int size)
{
	int ret = 0;
	unsigned char crc = 0;
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct lr1110_priv *priv = (struct lr1110_priv *)spi->priv;
	struct lr1110_transfer *transfer = (struct lr1110_transfer *)buff;

	/* NSS low */
	pio_clear_value(priv->gpios[NSS_GPIO].port, priv->gpios[NSS_GPIO].pin);

	/* Send CMD */
	for (int i = 0; i < transfer->command_length; i++) {
		spi_transfer(spi, (unsigned char *)&transfer->command[i], sizeof(unsigned char));
	}

	/* Send Data */
	for (int i = 0; i < transfer->data_length; i++) {
		spi_transfer(spi, (unsigned char *)&transfer->data[i], sizeof(unsigned char));
	}

	/* Compute and send CRC */
	crc = lr1110_compute_crc(0xFF, transfer->command, transfer->command_length);
	crc = lr1110_compute_crc(crc, transfer->data, transfer->data_length);

	/* Send CRC */
	spi_transfer(spi, (unsigned char *)&crc, sizeof(unsigned char));

	/* NSS high */
	pio_set_value(priv->gpios[NSS_GPIO].port, priv->gpios[NSS_GPIO].pin);

	return ret;
}

static int lr1110_ioctl(struct device *dev, int request, char *arg)
{
	int ret = 0;
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct lr1110_priv *priv = (struct lr1110_priv *)spi->priv;

	switch (request) {
	case IOCTL_RESET:
		pio_clear_value(priv->gpios[RESET_GPIO].port, priv->gpios[RESET_GPIO].pin);
		ktime_usleep(1000);
		pio_set_value(priv->gpios[RESET_GPIO].port, priv->gpios[RESET_GPIO].pin);
		break;
	case IOCTL_WAKEUP:
		pio_clear_value(priv->gpios[NSS_GPIO].port, priv->gpios[NSS_GPIO].pin);
		ktime_usleep(100);
		pio_set_value(priv->gpios[NSS_GPIO].port, priv->gpios[NSS_GPIO].pin);
		break;
	case IOCTL_WRITE_CHAR:
		spi_transfer(spi, (unsigned char *)arg, sizeof(unsigned char));
		break;
	case IOCTL_SPI_SET_NSS:
		pio_clear_value(priv->gpios[NSS_GPIO].port, priv->gpios[NSS_GPIO].pin);
		break;
	case IOCTL_SPI_CLEAR_NSS:
		pio_set_value(priv->gpios[NSS_GPIO].port, priv->gpios[NSS_GPIO].pin);
		break;
	default:
		error_printk("IOCTL %d not supported\n", request);
	}

	return ret;
}

static int lr1110_of_init_pio(struct lr1110_priv *priv, int offset, char *pio_name, int gpio)
{
	int ret = 0;

	ret = pio_of_configure_name(offset, pio_name);
	if (ret < 0)
		goto out;

	ret = pio_of_get(offset, pio_name, &priv->gpios[gpio].port, &priv->gpios[gpio].pin);
	if (ret < 0) {
		error_printk("failed to init %s\n", pio_name);
		goto out;
	}

out:
	return ret;
}

static int lr1110_of_init(struct spi_device *spi, int offset)
{
	int ret = 0;
	struct lr1110_priv *priv = (struct lr1110_priv *)spi->priv;

	ret = lr1110_of_init_pio(priv, offset, "nss-gpio", NSS_GPIO);
	if (ret < 0) {
		goto out;
	}

	ret = lr1110_of_init_pio(priv, offset, "reset-gpio", RESET_GPIO);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static struct device_operations dev_ops = {
	.read = lr1110_read,
	.write = lr1110_write,
	.ioctl = lr1110_ioctl,
};

int lr1110_init(struct device *dev)
{
	int offset;
	int ret = 0;
	struct spi_device *spi = NULL;
	struct lr1110_priv *priv = NULL;
	const void *fdt_blob = fdtparse_get_blob();

	offset = fdt_path_offset(fdt_blob, dev->of_path);
	if (offset < 0) {
		ret = -ENOENT;
		goto err;
	}

	priv = (struct lr1110_priv *)kmalloc(sizeof(struct lr1110_priv));
	if (!priv) {
		error_printk("failed to allocate lr1110 private struct\n");
		ret = -ENOMEM;
		goto err;
	}

	spi = spi_new_device_with_master(offset);
	if (!spi) {
		error_printk("failed to retrive new spi device\n");
		ret = -EIO;
		goto free_priv;
	}

	memcpy(&spi->dev, dev, sizeof(struct device));

	spi->priv = priv;

	ret = lr1110_of_init(spi, offset);
	if (ret < 0) {
		error_printk("failed to init fdt data\n");
		goto free_spi;
	}

	ret = spi_register_device(spi, &dev_ops);
	if (ret < 0) {
		error_printk("failed to register spi device\n");
		goto free_spi;
	}

	return 0;

free_spi:
	kfree(spi);
free_priv:
	kfree(priv);
err:
	return ret;
}

struct device lr1110_driver = {
	.of_compat = "spi,lr1110",
	.probe = lr1110_init,
};

static int lr1110_register(void)
{
	int ret = 0;

	ret = device_of_register(&lr1110_driver);
	if (ret < 0)
		error_printk("failed to register lr1110 device\n");
	return ret;
}
coredevice_initcall(lr1110_register);
