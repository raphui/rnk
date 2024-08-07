#include <kernel/printk.h>
#include <drv/spi.h>
#include <mm/mm.h>
#include <errno.h>
#include <string.h>
#include <utils.h>
#include <init.h>
#include <fdtparse.h>
#include <drv/pio.h>

static int spidev_of_init(struct spi_device *spi, int offset)
{
	int ret = 0;

	ret = pio_of_configure(offset);
	if (ret < 0) {
		error_printk("failed to init gpio skipping error\n");
		ret = 0;
	}

	ret = pio_of_configure_name(offset, "cs-gpio");
	if (ret < 0) {
		error_printk("failed to init cs gpio, skipping error\n");
		spi->use_cs = 0;
		ret = 0;
		goto out;
	}

	ret = pio_of_get(offset, "cs-gpio", &spi->cs_port, &spi->cs_pin);
	if (ret < 0) {
		error_printk("failed to init cs gpio\n");
		goto out;
	}

out:
	return ret;
}

int spidev_init(struct device *dev)
{
	int offset;
	int ret = 0;
	struct spi_device *spi = NULL;
	const void *fdt_blob = fdtparse_get_blob();

	offset = fdt_path_offset(fdt_blob, dev->of_path);
	if (offset < 0) {
		ret = -ENOENT;
		goto err;
	}

	spi = spi_new_device_with_master(offset);
	if (!spi) {
		error_printk("failed to retrive new spi device\n");
		ret = -EIO;
		goto err;
	}

	memcpy(&spi->dev, dev, sizeof(struct device));

	ret = spidev_of_init(spi, offset);
	if (ret < 0) {
		error_printk("failed to init fdt data\n");
		goto free_spi;
	}

	ret = spi_register_device(spi, NULL);
	if (ret < 0) {
		error_printk("failed to register spi device\n");
		goto free_spi;
	}

	return 0;

free_spi:
	kfree(spi);
err:
	return ret;
}

struct device spidev_driver = {
	.of_compat = "spi,spidev",
	.probe = spidev_init,
};

static int spidev_register(void)
{
	int ret = 0;

	ret = device_of_register(&spidev_driver);
	if (ret < 0)
		error_printk("failed to register spidev device\n");
	return ret;
}
coredevice_initcall(spidev_register);
